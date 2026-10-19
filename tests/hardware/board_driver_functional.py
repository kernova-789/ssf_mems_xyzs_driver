#!/usr/bin/env python3
"""实机 IIO/API 回归：所有传感器通信都通过正式驱动，不打开串口。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import errno
import json
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import threading
import time

from iio_devices import find_pair


def find(name):
    """按名称发现 IIO 设备，不依赖设备编号。"""
    return find_pair(os.environ.get('SSF_MEMS_TEST_SERDEV'))[name]


def read(path):
    """读取驱动公开属性；配置属性内部可能由驱动发起 Modbus。"""
    return path.read_text().strip()


def wait_online(feature, timeout=60):
    """等待驱动后台自动连接完成，不替驱动发送匹配帧。"""
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if read(feature / 'sensor_online') == '1':
            return
        time.sleep(0.1)
    raise TimeoutError('driver did not become online')


def settings(feature):
    """保存驱动读回的配置，便于验证非法写没有改变传感器。"""
    keys = ('sensor_baudrate', 'sensor_sampling_rate_index',
            'sensor_sampling_frequency', 'sensor_sampling_length_index',
            'sensor_parameter_switch', 'sensor_feature_enable',
            'sensor_firmware_version')
    values = {}
    for key in keys:
        started = time.monotonic()
        try:
            values[key] = read(feature / key)
        except OSError as error:
            print(json.dumps(dict(attribute=key, thread=threading.get_ident(),
                                  errno=error.errno,
                                  elapsed_ms=round((time.monotonic() - started) * 1000, 3))), flush=True)
            raise
    return values


def api_case(feature):
    """检查配置读回、非法参数、同值写入和多线程配置读取。"""
    wait_online(feature)
    before = settings(feature)
    assert before['sensor_baudrate'] == '128000', before
    assert before['sensor_sampling_rate_index'] == '6', before
    cases = []
    invalid = {
        'sensor_baudrate': ('0', '12345', '-1', 'invalid', '4294967296'),
        'sensor_sampling_rate_index': ('10', '65536', '-1', 'invalid'),
        'sensor_feature_enable': ('64', '65536', '-1', 'invalid'),
    }
    for key, values in invalid.items():
        for value in values:
            try:
                (feature / key).write_text(value)
            except OSError as error:
                assert error.errno in (errno.EINVAL, errno.ERANGE), (key, error)
                cases.append(dict(attribute=key, value=value, errno=error.errno))
            else:
                raise AssertionError((key, value, 'invalid write accepted'))
    assert settings(feature) == before
    # 同值采样率/特征开关写只读确认，不修改原有配置。
    for key in ('sensor_sampling_rate_index', 'sensor_feature_enable'):
        (feature / key).write_text(before[key])
    wait_online(feature)
    assert settings(feature) == before

    # 已匹配的波特率同值写应只读确认；检查当前实例没有新增保存/重启日志。
    instance = feature.resolve().parent.name
    baud_started = time.monotonic()
    kernel_started = time.clock_gettime(time.CLOCK_BOOTTIME)
    (feature / 'sensor_baudrate').write_text(before['sensor_baudrate'])
    wait_online(feature)
    assert read(feature / 'sensor_baudrate') == before['sensor_baudrate']
    log = subprocess.check_output(['dmesg'], text=True)
    for line in log.splitlines():
        stamp = re.search(r'\[\s*([\d.]+)\]', line)
        if stamp and float(stamp[1]) >= kernel_started and instance + ':' in line:
            assert 'baud change:' not in line and 'saving/rebooting via' not in line, line
    same_baud = dict(value=before['sensor_baudrate'], saved_or_rebooted=False,
                     elapsed_seconds=round(time.monotonic() - baud_started, 3))

    def reader(worker):
        """模拟多个应用同时经驱动读取配置，验证请求串行化。"""
        rows = []
        for _ in range(3):
            row = settings(feature)
            assert row == before, (worker, row)
            rows.append(row)
        return len(rows)

    with ThreadPoolExecutor(max_workers=4) as pool:
        concurrent = sum(pool.map(reader, range(4)))
    scales = {path.name: read(path) for path in feature.glob('*_scale')}
    assert scales and '0.098066500' in scales.values(), scales
    return dict(before=before, after=settings(feature), invalid=cases,
                concurrent_snapshots=concurrent, scales=scales,
                same_baud_write=same_baud)


def channels(device):
    """读取 IIO 扫描类型，用实际 ABI 计算子集扫描布局。"""
    result = []
    for path in (device / 'scan_elements').glob('*_index'):
        name = path.name[:-6]
        kind = read(path.with_name(name + '_type'))
        match = re.fullmatch(r'(le|be):([su])(\d+)/(\d+)>>(\d+)', kind)
        assert match is not None, kind
        result.append(dict(index=int(read(path)), name=name,
                           endian='little' if match[1] == 'le' else 'big',
                           signed=match[2] == 's', width=int(match[4]) // 8,
                           shift=int(match[5])))
    return sorted(result, key=lambda field: field['index'])


def configure(device, selected, length=8192):
    """仅在 buffer 关闭时设置扫描掩码；返回字段和对齐后的扫描大小。"""
    assert read(device / 'buffer/enable') == '0', device
    fields = []
    offset = 0
    maximum = 1
    for field in channels(device):
        enabled = field['index'] in selected
        (device / 'scan_elements' / (field['name'] + '_en')).write_text(str(int(enabled)))
        if enabled:
            width = field['width']
            maximum = max(maximum, width)
            offset = (offset + width - 1) // width * width
            fields.append(dict(field, offset=offset))
            offset += width
    (device / 'buffer/length').write_text(str(length))
    return fields, (offset + maximum - 1) // maximum * maximum


def decode(blob, fields, size, raw):
    """检查子集扫描的填充、时间戳和可用的包号/点号连续性。"""
    assert len(blob) % size == 0, (len(blob), size)
    previous_stamp = 0
    previous_pair = None
    records = len(blob) // size
    padding = set(range(size))
    for field in fields:
        padding.difference_update(range(field['offset'], field['offset'] + field['width']))
    for start in range(0, len(blob), size):
        row = blob[start:start + size]
        assert all(row[index] == 0 for index in padding), 'nonzero scan padding'
        values = {}
        for field in fields:
            offset = field['offset']
            value = int.from_bytes(row[offset:offset + field['width']],
                                   field['endian'], signed=field['signed']) >> field['shift']
            values[field['index']] = value
            if 'timestamp' in field['name']:
                assert value > 0 and value >= previous_stamp, value
                previous_stamp = value
        if raw and 3 in values and 4 in values:
            pair = values[3], values[4]
            assert 0 <= pair[0] < 16 and 0 <= pair[1] < 64, pair
            if previous_pair is not None:
                expected = ((previous_pair[0] + (previous_pair[1] == 63)) % 16,
                            (previous_pair[1] + 1) % 64)
                assert pair == expected, (pair, expected)
            previous_pair = pair
    return dict(records=records, scan_bytes=size,
                last_position=previous_pair, last_timestamp=previous_stamp)


def collect_subset(device, selected, output, raw, minimum):
    """记录一次真实内核子集扫描；测试程序不向串口发送采集命令。"""
    fields, size = configure(device, selected)
    fd = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
    blob = bytearray()
    try:
        (device / 'buffer/enable').write_text('1')
        poller = select.poll()
        poller.register(fd, select.POLLIN)
        until = time.monotonic() + 35
        while len(blob) // size < minimum and time.monotonic() < until:
            for _, event in poller.poll(100):
                assert event & (select.POLLERR | select.POLLHUP | select.POLLNVAL) == 0
                try:
                    blob.extend(os.read(fd, size * 4096))
                except BlockingIOError:
                    pass
        assert len(blob) // size >= minimum, (device, len(blob), size)
    finally:
        (device / 'buffer/enable').write_text('0')
        os.close(fd)
    output.write_bytes(blob)
    return dict(decode(blob, fields, size, raw), indices=selected, fields=fields)


def layout_case(feature, raw, output):
    """覆盖单通道、混合宽度、无时间戳和 8 位标志加时间戳布局。"""
    rows = []
    last = channels(feature)[-1]['index']
    cases = [('raw_x', raw, [0], True, 1024),
             ('raw_mixed', raw, [1, 3, 4, 5], True, 1024),
             ('raw_no_timestamp', raw, [0, 1, 2, 3, 4], True, 1024),
             ('feature_small', feature, [0, last], False, 3),
             ('feature_no_timestamp', feature, list(range(last)), False, 3),
             ('feature_flags_timestamp', feature, [last - 1, last], False, 3)]
    for name, device, selected, is_raw, minimum in cases:
        started = time.monotonic()
        row = collect_subset(device, selected, output / (name + '.bin'), is_raw, minimum)
        row.update(name=name, elapsed=round(time.monotonic() - started, 3))
        rows.append(row)
        print(json.dumps(row), flush=True)
    return rows


def invalid_buffer_case(feature, raw):
    """核对内核对空掩码、纯时间戳和运行中改掩码的正常拒绝。"""
    rows = []
    for device in (feature, raw):
        last = channels(device)[-1]['index']
        for selected in ([], [last]):
            configure(device, selected)
            try:
                (device / 'buffer/enable').write_text('1')
            except OSError as error:
                assert error.errno == errno.EINVAL, error
                rows.append(dict(device=device.name, indices=selected, errno=error.errno))
            else:
                (device / 'buffer/enable').write_text('0')
                raise AssertionError('kernel accepted an empty data scan mask')
        configure(device, [0, last])
        try:
            (device / 'buffer/enable').write_text('1')
            path = next(path for path in (device / 'scan_elements').glob('*_en')
                        if read(path.with_name(path.name[:-3] + '_index')) == '0')
            try:
                path.write_text('0')
            except OSError as error:
                assert error.errno == errno.EBUSY, error
                rows.append(dict(device=device.name, live_mask_errno=error.errno))
            else:
                raise AssertionError('kernel allowed changing an active scan mask')
        finally:
            (device / 'buffer/enable').write_text('0')
    return rows


def lifecycle_case(feature, raw, module):
    """核对打开者保护，再解绑唤醒两个阻塞读者，关闭后卸载重载。"""
    opened = []
    threads = []
    outcomes = {}
    unbound = False
    parent = feature.resolve().parent
    driver = (parent / 'driver').resolve()
    assert driver.name == 'ssf_mems_xyzs_driver'

    def reader(name, fd, size):
        """解绑应以 ENODEV 结束阻塞读取，而不是永久等待。"""
        try:
            outcomes[name] = dict(bytes=len(os.read(fd, size)))
        except OSError as error:
            outcomes[name] = dict(errno=error.errno)

    try:
        for device in (feature, raw):
            indices = [field['index'] for field in channels(device)]
            _, size = configure(device, indices)
            fd = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
            opened.append((device, fd, size))
            (device / 'buffer/enable').write_text('1')
            # 同一 IIO 字符设备只允许一个打开者，这是内核正常语义。
            try:
                another = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as error:
                assert error.errno == errno.EBUSY, error
            else:
                os.close(another)
                raise AssertionError('duplicate IIO open was accepted')
        until = time.monotonic() + 40
        saw_modbus = False
        while time.monotonic() < until:
            mode = read(raw / 'raw_link_mode')
            if mode == '0':
                saw_modbus = True
            if saw_modbus and mode == '1' and read(raw / 'raw_batch_packets') == '0':
                break
            time.sleep(0.01)
        else:
            raise TimeoutError('no before-first-packet phase for blocking-reader unload')
        for device, fd, size in opened:
            while True:
                try:
                    if not os.read(fd, size * 4096):
                        break
                except BlockingIOError:
                    break
            os.set_blocking(fd, True)
            thread = threading.Thread(target=reader, args=(device.name, fd, size), daemon=True)
            thread.start()
            threads.append(thread)
        time.sleep(0.05)
        assert not outcomes, ('reader was not blocked', outcomes)
        guarded = subprocess.run(['rmmod', 'ssf_mems'], capture_output=True,
                                 text=True, timeout=10)
        assert guarded.returncode != 0 and 'in use' in guarded.stderr, guarded
        started = time.monotonic()
        subprocess.run([sys.executable, '-c',
                        'from pathlib import Path; import sys; Path(sys.argv[1]).write_text(sys.argv[2])',
                        str(driver / 'unbind'), parent.name], check=True, timeout=45)
        unbound = True
        unbind_seconds = round(time.monotonic() - started, 3)
        for thread in threads:
            thread.join(5)
            assert not thread.is_alive(), 'blocking reader was not woken by unbind'
        assert all(row.get('errno') == errno.ENODEV for row in outcomes.values()), outcomes
        assert len(outcomes) == 2 and not feature.exists() and not raw.exists(), outcomes
        return dict(unbind_seconds=unbind_seconds, blocking_readers=outcomes,
                    duplicate_open_errno=errno.EBUSY,
                    open_fd_rmmod_rejected=guarded.stderr.strip())
    finally:
        for device, fd, _ in opened:
            if not unbound and device.exists():
                (device / 'buffer/enable').write_text('0')
            os.close(fd)
        if unbound:
            subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=20)
            subprocess.run(['insmod', str(module)], check=True, timeout=20)
            restored = find('ssf_mems_xyzs')
            wait_online(restored)
            assert read(restored / 'sensor_baudrate') == '128000'


def early_remove_case(module):
    """在后台首次通信前卸载，用 UART TX 计数确认 probe 未发 Modbus。"""
    def tx_count():
        """仅读取内核串口统计，不打开串口、不改变 UART。"""
        line = next(line for line in Path('/proc/tty/driver/serial').read_text().splitlines()
                    if line.startswith('4:'))
        return int(re.search(r'\btx:(\d+)', line)[1])

    subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=45)
    before = tx_count()
    try:
        started = time.monotonic()
        subprocess.run(['insmod', str(module)], check=True, timeout=20)
        time.sleep(0.05)
        subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=20)
        after = tx_count()
        assert before == after, (before, after)
        return dict(tx_before=before, tx_after=after,
                    load_remove_seconds=round(time.monotonic() - started, 3))
    finally:
        if not Path('/sys/module/ssf_mems').exists():
            subprocess.run(['insmod', str(module)], check=True, timeout=20)
        feature = find('ssf_mems_xyzs')
        wait_online(feature)
        assert read(feature / 'sensor_baudrate') == '128000'


def overflow_case(feature, raw):
    """故意不消费小 buffer，验证 IIO 丢样不会使传感器通信卡死。"""
    configure(raw, [0, 1, 2, 3, 4, 5], length=2)
    fd = os.open('/dev/' + raw.name, os.O_RDONLY | os.O_NONBLOCK)
    peak = 0
    phases = []
    try:
        (raw / 'buffer/enable').write_text('1')
        until = time.monotonic() + 35
        while time.monotonic() < until:
            peak = max(peak, int(read(raw / 'raw_buffer_drops')))
            mode = int(read(raw / 'raw_link_mode'))
            if not phases or phases[-1] != mode:
                phases.append(mode)
            if peak > 0 and mode == 0:
                break
            time.sleep(0.02)
        assert peak > 0, 'tiny raw buffer did not overflow'
        assert int(read(raw / 'raw_link_mode')) == 0, phases
        data = os.read(fd, 24 * 4096)
        assert len(data) % 24 == 0 and len(data) > 0
    finally:
        (raw / 'buffer/enable').write_text('0')
        os.close(fd)
    wait_online(feature)
    assert read(feature / 'sensor_baudrate') == '128000'
    # 配置读取成功后继续等一份新鲜特征，区分旧缓存与链路已恢复。
    until = time.monotonic() + 30
    while time.monotonic() < until:
        try:
            value = read(feature / 'in_accel_x_rms_raw')
            age = int(read(feature / 'sensor_sample_age_ms'))
            if age < 2000:
                return dict(intentional_raw_drops=peak, phases=phases,
                            retained_records=len(data) // 24, feature_value=value,
                            feature_age_ms=age)
        except OSError as error:
            assert error.errno == errno.ENODATA, error
        time.sleep(0.1)
    raise TimeoutError('feature cache did not refresh after raw overflow')


def feature_overflow_case(feature):
    """故意塞满特征 buffer，确认缓存更新和串口采集仍继续。"""
    indices = [field['index'] for field in channels(feature)]
    fields, size = configure(feature, indices, length=2)
    fd = os.open('/dev/' + feature.name, os.O_RDONLY | os.O_NONBLOCK)
    started = time.clock_gettime(time.CLOCK_BOOTTIME)
    try:
        (feature / 'buffer/enable').write_text('1')
        time.sleep(20)
        wait_online(feature)
        cache = read(feature / 'in_accel_x_rms_raw')
        age = int(read(feature / 'sensor_sample_age_ms'))
        assert age < 2000, age
        data = os.read(fd, size * 4096)
        assert len(data) == size * 2, len(data)
        decode(data, fields, size, False)
        lines = subprocess.check_output(['dmesg'], text=True).splitlines()
        warnings = [line for line in lines if
                    'failed to push feature sample to IIO buffer: -16' in line
                    and float(re.search(r'\[\s*([\d.]+)\]', line)[1]) >= started]
        assert warnings, 'feature buffer never reported the intentional overflow'
        return dict(retained_records=2, intentional_overflow_warnings=len(warnings),
                    feature_value=cache, feature_age_ms=age)
    finally:
        (feature / 'buffer/enable').write_text('0')
        os.close(fd)


def main():
    """按选定用例执行，失败保留 JSON 和数据，finally 关闭本测试的 buffer。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('case', choices=('api', 'layouts', 'overflow', 'invalid-buffer',
                                        'lifecycle', 'early-remove', 'feature-overflow'))
    parser.add_argument('--module', type=Path, help='lifecycle 用例重载的正式模块路径')
    parser.add_argument('--serdev', help='只测试该 serdev 实例，例如 serial0-0')
    args = parser.parse_args()
    if args.serdev is not None:
        os.environ['SSF_MEMS_TEST_SERDEV'] = args.serdev
    feature, raw = find('ssf_mems_xyzs'), find('ssf_mems_xyzs_raw')
    assert read(feature / 'buffer/enable') == read(raw / 'buffer/enable') == '0'
    result = dict(case=args.case, passed=False)
    started = time.monotonic()
    try:
        wait_online(feature)
        if args.case == 'api':
            result['details'] = api_case(feature)
        elif args.case == 'layouts':
            result['details'] = layout_case(feature, raw, args.output)
        elif args.case == 'invalid-buffer':
            result['details'] = invalid_buffer_case(feature, raw)
        elif args.case == 'lifecycle':
            assert args.module is not None and args.module.is_file()
            result['details'] = lifecycle_case(feature, raw, args.module)
        elif args.case == 'early-remove':
            assert args.module is not None and args.module.is_file()
            result['details'] = early_remove_case(args.module)
        elif args.case == 'feature-overflow':
            result['details'] = feature_overflow_case(feature)
        else:
            result['details'] = overflow_case(feature, raw)
        result['passed'] = True
        print(json.dumps(result, ensure_ascii=False), flush=True)
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        for device in (feature, raw):
            if device.exists():
                (device / 'buffer/enable').write_text('0')
        result['elapsed'] = round(time.monotonic() - started, 3)
        (args.output / (args.case + '_summary.json')).write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
