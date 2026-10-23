#!/usr/bin/env python3
"""双 XYZS 训练数据记录器（Python 标准库；默认仅运行 60 秒）。

只读枚举：python3 tests/training/collect_training_data.py --list
采集示例：sudo python3 tests/training/collect_training_data.py --output /data/mems \
    --raw-serdev serial0-0 --feature-serdev serial1-0 --seconds 60
离线校验：python3 tests/training/collect_training_data.py --verify /data/mems/<session>

长度轮换由本脚本发命令完成，驱动只执行请求；每档默认一个五秒窗口。
需要新版 sensor_raw_batch_samples / sensor_raw_manual / sensor_raw_capture 接口。
完整用法及数据格式见同目录 README.md 和 TRAINING_DATA_FORMAT.md。
"""
import argparse
import csv
import datetime as dt
import fcntl
import hashlib
import io
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import re
import select
import shutil
import signal
import sys
import tempfile
import time
import uuid
from concurrent.futures import ThreadPoolExecutor


SYSFS = Path('/sys/bus/iio/devices')
SUPPORTED_LENGTHS = [1024, 2048, 4096, 8192, 16384, 32768]
RAW_NAMES = ('in_accel_x', 'in_accel_y', 'in_accel_z',
             'in_count0_packet_sequence', 'in_count1_sample_index', 'in_timestamp')
ERROR_STATS = ('raw_sequence_gaps', 'raw_crc_errors', 'raw_buffer_drops',
               'raw_drain_sequence_gaps', 'raw_drain_crc_errors', 'raw_ignored_bytes')


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec='microseconds')


def read(path):
    return path.read_text().strip()


def write(path, value):
    path.write_text(str(value))


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_json(path, data):
    temporary = path.with_suffix(path.suffix + '.partial')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_directory(path.parent)


def append_json(path, data):
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(path.parent)


def index_entries(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def discover(root=SYSFS):
    pairs = {}
    for path in sorted(root.glob('iio:device*')):
        if not (path / 'name').exists():
            continue
        name = read(path / 'name')
        if name not in ('ssf_mems_xyzs', 'ssf_mems_xyzs_raw'):
            continue
        parent = path.resolve().parent
        role = 'raw' if name.endswith('_raw') else 'feature'
        pair = pairs.setdefault(parent.name, {'parent': str(parent)})
        if role in pair or pair['parent'] != str(parent):
            raise RuntimeError(f'重复或有歧义的设备实例：{parent.name}')
        pair[role] = str(path)
    return pairs


def layout(device):
    fields = []
    offset = 0
    for path in (device / 'scan_elements').glob('*_index'):
        name = path.name[:-6]
        kind = read(path.with_name(name + '_type'))
        match = re.fullmatch(r'(le|be):([su])(\d+)/(\d+)>>(\d+)', kind)
        if not match:
            raise ValueError(f'不支持的 IIO 类型：{name}: {kind}')
        endian, sign, bits, storage, shift = match.groups()
        bits, storage, shift = int(bits), int(storage), int(shift)
        if storage not in (8, 16, 32, 64) or not 0 < bits <= storage or bits + shift > storage:
            raise ValueError(f'非法扫描位宽：{kind}')
        field = dict(name=name, index=int(read(path)), type=kind,
                     width=storage // 8, bits=bits, shift=shift,
                     signed=sign == 's', byteorder='little' if endian == 'le' else 'big')
        scale = device / (name + '_scale')
        if scale.exists():
            field['scale'] = read(scale)
        field['unit_after_scale'] = channel_unit(name)
        fields.append(field)
    fields.sort(key=lambda field: field['index'])
    if not fields or len({f['index'] for f in fields}) != len(fields):
        raise ValueError('扫描通道为空或索引重复')
    for field in fields:
        width = field['width']
        offset = (offset + width - 1) // width * width
        field['offset'] = offset
        offset += width
    alignment = max(f['width'] for f in fields)
    size = (offset + alignment - 1) // alignment * alignment
    clock = device / 'current_timestamp_clock'
    return {'fields': fields, 'scan_bytes': size,
            'timestamp_clock': read(clock) if clock.exists() else 'not exposed'}


def channel_unit(name):
    if name.startswith('in_accel_'):
        return 'm/s^2'
    if name.startswith('in_velocity_'):
        return 'm/s'
    if name == 'in_temp':
        return 'millidegree Celsius'
    if name == 'in_timestamp':
        return 'ns; host publication time'
    if name.endswith('_db'):
        return 'device-defined dB'
    if 'zero_crossing_rate_percent' in name:
        return 'percent'
    if 'spectral_centroid_hz' in name:
        return 'Hz'
    return 'dimensionless or device-defined; see channel name'


class Decoder:
    def __init__(self, schema):
        self.schema = schema
        self.size = schema['scan_bytes']
        used = set()
        for f in schema['fields']:
            used.update(range(f['offset'], f['offset'] + f['width']))
        self.padding = sorted(set(range(self.size)) - used)

    def decode(self, row):
        if len(row) != self.size or any(row[i] for i in self.padding):
            raise ValueError('扫描记录长度或对齐填充错误')
        values = {}
        for f in self.schema['fields']:
            value = int.from_bytes(row[f['offset']:f['offset'] + f['width']], f['byteorder'])
            value = (value >> f['shift']) & ((1 << f['bits']) - 1)
            if f['signed'] and value & (1 << (f['bits'] - 1)):
                value -= 1 << f['bits']
            values[f['name']] = value
        return values


class BatchValidator:
    """只接受完整的一批，不跨批拼接；序号从 0 开始，每包 64 点。"""
    def __init__(self, decoder, samples):
        self.decoder = decoder
        self.samples = samples
        self.rows = bytearray()
        self.count = 0
        self.previous_timestamp = 0
        self.first_timestamp = None

    def feed(self, row):
        values = self.decoder.decode(row)
        pair = (values['in_count0_packet_sequence'], values['in_count1_sample_index'])
        expected = divmod(self.count, 64)
        stamp = values['in_timestamp']
        if pair != expected or stamp <= 0 or stamp < self.previous_timestamp:
            raise ValueError(f'批次序号或时间戳错误：{pair}, expected={expected}, timestamp={stamp}')
        if self.count == 0:
            self.first_timestamp = stamp
        self.previous_timestamp = stamp
        self.rows.extend(row)
        self.count += 1
        if self.count != self.samples:
            return None
        result = (bytes(self.rows), self.first_timestamp, stamp)
        self.rows.clear()
        self.count = 0
        return result


class Dataset:
    """每个角色使用独立索引；payload + 元数据完成后才追加索引。"""
    def __init__(self, root, role, min_free_mib):
        self.root = Path(root)
        self.role = role
        self.directory = self.root / role
        self.directory.mkdir(exist_ok=True)
        self.minimum = min_free_mib * 1024 * 1024
        self.sequence = 0

    def check_space(self, needed=0):
        if shutil.disk_usage(self.root).free < self.minimum + needed:
            raise OSError('剩余磁盘空间低于安全余量，停止采集')

    def commit(self, payload, extension, metadata, publish=True):
        self.check_space(len(payload) * 2 + 16384)
        self.sequence += 1
        directory = self.directory / dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%d/%H')
        directory.mkdir(parents=True, exist_ok=True)
        sync_directory(directory.parent)
        sync_directory(self.directory)
        path = directory / f'{self.sequence:012d}.{extension}'
        temporary = path.with_suffix(path.suffix + '.partial')
        if path.exists() or temporary.exists():
            raise FileExistsError(path)
        with temporary.open('xb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        metadata = dict(metadata, payload=str(path.relative_to(self.root)), bytes=len(payload),
                        sha256=hashlib.sha256(payload).hexdigest(), stored_utc=utc())
        atomic_json(path.with_suffix('.json'), metadata)
        directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        if publish:
            append_json(self.directory / 'index.jsonl', metadata)
        return metadata

    def reject(self, payload, reason):
        directory = self.root / 'rejected'
        directory.mkdir(exist_ok=True)
        token = f'{self.role}-{uuid.uuid4().hex}'
        # 磁盘已满时保留错误文本的最佳努力，不把残批加入训练索引。
        (directory / (token + '.bin')).write_bytes(payload)
        atomic_json(directory / (token + '.json'), {'role': self.role, 'reason': reason,
                                                   'bytes': len(payload), 'utc': utc()})


def validate_lengths(lengths, available):
    if not lengths or any(length not in available for length in lengths):
        raise ValueError(f'采集长度不受支持：{lengths}；可选长度：{available}')


def capture_raw_windows(pair, args, root, stop, fd, decoder, settings, status, dataset):
    """长度轮换只发生在用户态；每个同步驱动窗口结束后校验并提交。"""
    feature, raw = Path(pair['feature']), Path(pair['raw'])
    poller = select.poll()
    poller.register(fd, select.POLLIN | select.POLLERR | select.POLLHUP)
    window_id = 0
    executor = ThreadPoolExecutor(max_workers=1)
    previous_stamp = 0
    try:
        while not stop.is_set():
            samples = args.lengths[window_id % len(args.lengths)]
            window_id += 1
            write(feature / 'sensor_raw_batch_samples', samples)
            if int(read(feature / 'sensor_raw_batch_samples')) != samples:
                raise RuntimeError('长度配置读回不一致')
            try:
                leftover = os.read(fd, decoder.size)
            except BlockingIOError:
                leftover = b''
            if leftover:
                raise RuntimeError('窗口边界仍有未归属的旧 IIO 记录')
            batch = BatchValidator(decoder, samples)
            batch.previous_timestamp = previous_stamp
            pending, staged = b'', []
            records = 0
            first_utc = utc()
            first_monotonic_ns = time.monotonic_ns()
            # 考虑低采样率和串口上传耗时；五秒窗口到期也必须收完在途批次。
            fs = float(settings['sensor_sampling_frequency'])
            baud = int(settings['sensor_baudrate'])
            batch_seconds = samples / fs + (samples // 64) * 392 * 10 / baud + 12
            deadline = time.monotonic() + max(args.stall_seconds, args.window_seconds + batch_seconds)
            future = executor.submit(write, feature / 'sensor_raw_capture', 1)

            def consume(data):
                nonlocal pending, records
                pending += data
                complete = len(pending) // decoder.size * decoder.size
                rows, pending = pending[:complete], pending[complete:]
                for offset in range(0, len(rows), decoder.size):
                    row = rows[offset:offset + decoder.size]
                    try:
                        result = batch.feed(row)
                    except ValueError:
                        dataset.reject(bytes(batch.rows) + row, 'invalid_batch')
                        batch.rows.clear()
                        raise
                    records += 1
                    if result is not None:
                        payload, first, last = result
                        staged.append(dataset.commit(payload, 'bin', {
                            'validation': 'whole_batch_sequence_padding_timestamp_and_completed_driver_window',
                            'samples': samples, 'window_id': window_id,
                            'sampling_frequency_hz': settings['sensor_sampling_frequency'],
                            'first_iio_timestamp_ns': first, 'last_iio_timestamp_ns': last,
                            'schema': 'session.json:devices.raw.schema',
                            'contiguous_across_batches': False}, publish=False))
            try:
                next_health = 0
                while True:
                    now = time.monotonic()
                    if now > deadline:
                        raise TimeoutError('完整原始窗口超时')
                    if now >= next_health:
                        dataset.check_space()
                        if read(feature / 'sensor_acquisition_mode') != 'raw':
                            raise RuntimeError('原始采集模式被其他程序修改')
                        atomic_json(dataset.directory / 'status.json', dict(status,
                            state='recording', current_samples=samples, window_id=window_id,
                            current_window_records=records, updated_utc=utc()))
                        next_health = now + 1
                    for _, events in poller.poll(100):
                        if events & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                            raise OSError(f'原始 IIO 设备异常，poll={events}')
                        try:
                            data = os.read(fd, decoder.size * 4096)
                        except BlockingIOError:
                            continue
                        if not data:
                            raise OSError('原始 IIO EOF')
                        consume(data)
                    if future.done():
                        future.result()  # 驱动窗口错误不能被已收到的正确前缀掩盖。
                        while True:
                            try:
                                data = os.read(fd, decoder.size * 4096)
                            except BlockingIOError:
                                break
                            if not data:
                                raise OSError('窗口结束时 IIO EOF')
                            consume(data)
                        break
                health = {name: int(read(raw / name)) for name in
                          ERROR_STATS + ('raw_packets', 'raw_batches', 'raw_batch_samples')}
                health.update(window_id=window_id, utc=utc(),
                              raw_sampling_frequency=read(raw / 'raw_sampling_frequency'))
                append_json(dataset.directory / 'health.jsonl', health)
                if (any(health[name] for name in ERROR_STATS) or batch.count or pending or
                    not staged or health['raw_batches'] != len(staged) or
                    health['raw_packets'] * 64 != records or health['raw_batch_samples'] != samples or
                    health['raw_sampling_frequency'] != settings['sensor_sampling_frequency']):
                    raise ValueError('原始窗口统计、样本数、长度或采样率不一致')
                atomic_json(dataset.directory / f'window-{window_id:08d}.json', {
                    'samples': samples, 'requested_seconds': args.window_seconds,
                    'started_utc': first_utc, 'ended_utc': utc(),
                    'started_monotonic_ns': first_monotonic_ns,
                    'ended_monotonic_ns': time.monotonic_ns(), 'health': health,
                    'files': [entry['payload'] for entry in staged]})
                for entry in staged:
                    append_json(dataset.directory / 'index.jsonl', entry)
                status['records'] += records
                status['units'] += len(staged)
                previous_stamp = batch.previous_timestamp
            except BaseException:
                stop.set()
                # 未通过窗口级验收的文件整体隔离，不作为训练输入。
                rejected = []
                if staged:
                    quarantine = Path(root) / 'rejected' / f'raw-window-{window_id:08d}'
                    quarantine.mkdir(parents=True, exist_ok=False)
                    for entry in staged:
                        original = Path(root) / entry['payload']
                        target = quarantine / original.name
                        os.replace(original, target)
                        os.replace(original.with_suffix('.json'), target.with_suffix('.json'))
                        rejected.append(str(target.relative_to(root)))
                atomic_json(dataset.directory / f'failed-window-{window_id:08d}.json', {
                    'samples': samples, 'rejected_files': rejected})
                if batch.rows or pending:
                    dataset.reject(bytes(batch.rows) + pending, 'incomplete_window')
                raise
    finally:
        executor.shutdown(wait=True)


def config_snapshot(pair):
    feature = Path(pair['feature'])
    return {name: read(feature / name) for name in (
        'sensor_acquisition_mode', 'sensor_raw_duration_ms',
        'sensor_poll_mode', 'sensor_poll_fixed_interval_ms',
        'sensor_raw_batch_samples', 'sensor_raw_manual')}


def snapshot_buffer(device):
    saved = {'length': read(device / 'buffer/length'), 'scan': {}}
    for path in sorted((device / 'scan_elements').glob('*_en')):
        saved['scan'][path.name] = read(path)
    watermark = device / 'buffer/watermark'
    if watermark.exists():
        saved['watermark'] = read(watermark)
    # 保存动作由调用者在任何写入前完成，发生部分配置失败也可回滚。
    return saved


def worker(role, pair, args, root, stop):
    """两路独立进程，配置/文件 I/O 不阻塞另一条设备的读取。"""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    feature = Path(pair['feature'])
    device = Path(pair[role])
    directory = Path(root) / role
    dataset = Dataset(root, role, args.min_free_mib)
    status = {'state': 'starting', 'role': role, 'records': 0, 'units': 0, 'started_utc': utc()}
    saved = None
    saved_buffer = None
    fd = None
    pending = b''
    batch = None
    feature_rows = []
    feature_first = None
    previous_stamp = 0
    previous_values = None
    dirty = False
    failed = False
    try:
        saved = config_snapshot(pair)
        atomic_json(directory / 'original_config.json', saved)
        saved_buffer = snapshot_buffer(device)
        atomic_json(directory / 'original_buffer.json', saved_buffer)
        dirty = True
        write(feature / 'sensor_acquisition_mode', 'feature')
        deadline = time.monotonic() + args.startup_timeout
        while read(feature / 'sensor_online') != '1':
            if stop.wait(0.2) or time.monotonic() > deadline:
                raise TimeoutError('等待传感器上线超时或采集已取消')
        settings = {name: read(feature / name) for name in (
            'sensor_sampling_frequency', 'sensor_sampling_rate_index', 'sensor_firmware_version',
            'sensor_baudrate')}
        settings['raw_batch_samples'] = read(Path(pair['raw']) / 'raw_batch_samples')
        if role == 'raw':
            validate_lengths(args.lengths, [int(x) for x in
                read(feature / 'sensor_raw_batch_samples_available').split()])
            write(feature / 'sensor_raw_manual', 1)
            write(feature / 'sensor_raw_duration_ms', round(args.window_seconds * 1000))
        else:
            write(feature / 'sensor_poll_fixed_interval_ms', args.poll_ms)
            write(feature / 'sensor_poll_mode', 'fixed')
        for path in (device / 'scan_elements').glob('*_en'):
            write(path, 1)
        if (device / 'buffer/watermark').exists():
            write(device / 'buffer/watermark', 1)
        write(device / 'buffer/length', args.buffer_records if role == 'raw' else 256)
        decoder = Decoder(pair['schema'])
        batch = BatchValidator(decoder, 1024) if role == 'raw' else None
        fd = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
        write(device / 'buffer/enable', 1)
        write(feature / 'sensor_acquisition_mode', role)
        settings.update(mode=role, configuration_utc=utc())
        atomic_json(directory / 'capture_config.json', settings)
        status['state'] = 'recording'
        if role == 'raw':
            capture_raw_windows(pair, args, root, stop, fd, decoder, settings, status, dataset)
        else:
            poller = select.poll()
            poller.register(fd, select.POLLIN | select.POLLERR | select.POLLHUP)
            last_record = time.monotonic()
            next_health = last_record
            next_chunk = last_record + args.feature_chunk_seconds

            def save_features():
                nonlocal feature_rows, feature_first
                if not feature_rows:
                    return
                stream = io.StringIO(newline='')
                writer = csv.writer(stream)
                writer.writerow(['received_realtime_ns', 'same_payload_as_previous'] +
                                [f['name'] for f in pair['schema']['fields']])
                writer.writerows(feature_rows)
                dataset.commit(stream.getvalue().encode('utf-8'), 'csv', {
                    'validation': 'layout_padding_timestamp', 'records': len(feature_rows),
                    'first_iio_timestamp_ns': feature_first,
                    'last_iio_timestamp_ns': feature_rows[-1][-1],
                    'schema': 'session.json:devices.feature.schema',
                    'values': 'integer IIO raw counts; scales in session schema'})
                feature_rows, feature_first = [], None
                status['units'] += 1

            while not stop.is_set():
                now = time.monotonic()
                if now - last_record > args.stall_seconds:
                    raise TimeoutError('特征数据停止推进')
                if now >= next_health:
                    dataset.check_space()
                    if read(feature / 'sensor_acquisition_mode') != 'feature':
                        raise RuntimeError('特征采集模式被其他程序修改')
                    append_json(directory / 'health.jsonl', {
                        'utc': utc(), 'online': read(feature / 'sensor_online')})
                    atomic_json(directory / 'status.json', dict(status, updated_utc=utc()))
                    next_health = now + 1
                for _, events in poller.poll(100):
                    if events & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                        raise OSError(f'特征 IIO 设备异常，poll={events}')
                    try:
                        data = os.read(fd, decoder.size * 4096)
                    except BlockingIOError:
                        continue
                    if not data:
                        raise OSError('特征 IIO EOF')
                    pending += data
                    complete = len(pending) // decoder.size * decoder.size
                    rows, pending = pending[:complete], pending[complete:]
                    for offset in range(0, len(rows), decoder.size):
                        row = rows[offset:offset + decoder.size]
                        values = decoder.decode(row)
                        stamp = values['in_timestamp']
                        if stamp <= 0 or stamp < previous_stamp:
                            raise ValueError('特征记录时间戳错误')
                        previous_stamp = stamp
                        payload = tuple(values[f['name']] for f in pair['schema']['fields']
                                        if f['name'] != 'in_timestamp')
                        repeated = int(payload == previous_values)
                        previous_values = payload
                        if feature_first is None:
                            feature_first = stamp
                        feature_rows.append([time.time_ns(), repeated] +
                                            [values[f['name']] for f in pair['schema']['fields']])
                        status['records'] += 1
                        last_record = time.monotonic()
                        if last_record >= next_chunk or len(feature_rows) >= 10000:
                            save_features()
                            next_chunk = last_record + args.feature_chunk_seconds
            save_features()
        if status['records'] == 0:
            raise RuntimeError('本路没有采集到完整有效数据')
        status['state'] = 'completed'
    except BaseException as error:
        failed = True
        status.update(state='failed', error=repr(error))
        stop.set()
    finally:
        cleanup_errors = []
        if fd is not None:
            os.close(fd)
        if dirty:
            actions = [(device / 'buffer/enable', 0),
                       (feature / 'sensor_acquisition_mode', 'feature')]
            if saved_buffer:
                actions += [(device / 'scan_elements' / name, value)
                            for name, value in saved_buffer['scan'].items()]
                # 先减小 watermark，避免旧值大于恢复后的 length。
                if 'watermark' in saved_buffer:
                    actions.append((device / 'buffer/watermark', 1))
                actions.append((device / 'buffer/length', saved_buffer['length']))
                if 'watermark' in saved_buffer:
                    actions.append((device / 'buffer/watermark', saved_buffer['watermark']))
            if saved:
                actions += [(feature / name, value) for name, value in saved.items()
                            if name != 'sensor_acquisition_mode']
                actions.append((feature / 'sensor_acquisition_mode', saved['sensor_acquisition_mode']))
            for path, value in actions:
                try:
                    write(path, value)
                except Exception as error:
                    cleanup_errors.append(f'{path}: {error}')
        try:
            fragment = (bytes(batch.rows) if batch else b'') + pending
            if fragment:
                dataset.reject(fragment, 'unfinished_on_exit')
            if feature_rows:
                dataset.reject(json.dumps(feature_rows).encode(), 'uncommitted_feature_rows')
        except Exception as error:
            cleanup_errors.append(f'保存隔离数据失败：{error}')
        status.update(ended_utc=utc(), cleanup_errors=cleanup_errors)
        if cleanup_errors:
            failed = True
            status['state'] = 'failed'
        atomic_json(directory / 'status.json', status)
    if failed:
        raise SystemExit(1)


def checked_payload(root, role, entry):
    """用索引、旁车元数据和 SHA256 联合验证文件；返回有界分片。"""
    root = Path(root)
    path = (root / entry['payload']).resolve()
    if (root / role).resolve() not in path.parents:
        raise ValueError('索引含越界路径')
    if json.loads(path.with_suffix('.json').read_text()) != entry:
        raise ValueError('文件元数据与索引不一致')
    payload = path.read_bytes()
    if len(payload) != entry['bytes'] or hashlib.sha256(payload).hexdigest() != entry['sha256']:
        raise ValueError(f'文件校验失败：{path}')
    return payload


def verify(root):
    root = Path(root)
    session = json.loads((root / 'session.json').read_text())
    summary = json.loads((root / 'summary.json').read_text())
    if session.get('format_version') != 2:
        raise ValueError('数据格式版本不受支持；需要包含窗口验收记录的 format_version=2')
    if summary.get('state') != 'completed' or summary.get('manual_cleanup_required'):
        raise ValueError('会话未正常完成，不能当作已验收训练数据')
    counts = {}
    for role in ('raw', 'feature'):
        state = summary['workers'][role]
        if state.get('state') != 'completed' or state.get('cleanup_errors'):
            raise ValueError('设备采集或清理状态不正确')
        count, units = 0, 0
        decoder = Decoder(session['devices'][role]['schema'])
        previous, previous_sequence = 0, 0
        previous_values = None
        current_window, window_info, window_files = 0, None, 0
        for entry in index_entries(root / role / 'index.jsonl'):
            path = Path(entry['payload'])
            sequence = int(path.stem)
            if sequence <= previous_sequence:
                raise ValueError('索引含重复文件或顺序错误')
            previous_sequence = sequence
            payload = checked_payload(root, role, entry)
            if role == 'raw':
                samples = int(entry['samples'])
                validate_lengths([samples], SUPPORTED_LENGTHS)
                fs = float(entry['sampling_frequency_hz'])
                if not math.isfinite(fs) or fs <= 0:
                    raise ValueError('采样率无效')
                window = int(entry['window_id'])
                if window != current_window:
                    if window_info is not None and window_files != len(window_info['files']):
                        raise ValueError('窗口索引缺少批次')
                    if window != current_window + 1:
                        raise ValueError('窗口顺序不连续')
                    window_info = json.loads((root / role / f'window-{window:08d}.json').read_text())
                    health = window_info['health']
                    expected_files = window_info['files']
                    if (not expected_files or len(set(expected_files)) != len(expected_files) or
                        any(health[name] != 0 for name in ERROR_STATS) or
                        health['window_id'] != window or
                        health['raw_batch_samples'] != window_info['samples'] or
                        health['raw_batches'] != len(expected_files) or
                        health['raw_packets'] * 64 != len(expected_files) * window_info['samples'] or
                        window_info['ended_monotonic_ns'] <= window_info['started_monotonic_ns']):
                        raise ValueError('窗口验收证据不完整或有采集错误')
                    current_window, window_files = window, 0
                if (samples != window_info['samples'] or
                    fs != float(window_info['health']['raw_sampling_frequency']) or
                    window_files >= len(window_info['files']) or
                    entry['payload'] != window_info['files'][window_files]):
                    raise ValueError('批次与所属窗口的长度、采样率或文件列表不一致')
                if len(payload) != samples * decoder.size:
                    raise ValueError('原始文件长度与真实批次配置不符')
                batch = BatchValidator(decoder, samples)
                batch.previous_timestamp = previous
                result = None
                for offset in range(0, len(payload), decoder.size):
                    result = batch.feed(payload[offset:offset + decoder.size])
                first_stamp, last_stamp = result[1:]
                previous = last_stamp
                count += samples
                window_files += 1
            else:
                rows = csv.DictReader(io.StringIO(payload.decode('utf-8')))
                expected = ['received_realtime_ns', 'same_payload_as_previous'] + [
                    field['name'] for field in decoder.schema['fields']]
                if rows.fieldnames != expected:
                    raise ValueError('特征 CSV 列名或列顺序错误')
                unit_count = 0
                first_stamp = None
                for row in rows:
                    if set(row) != set(expected) or int(row['received_realtime_ns']) <= 0:
                        raise ValueError('特征 CSV 行结构或接收时间错误')
                    stamp = int(row['in_timestamp'])
                    if stamp <= 0 or stamp < previous:
                        raise ValueError('特征时间戳错误')
                    values = []
                    for field in decoder.schema['fields']:
                        value = int(row[field['name']])
                        low = -(1 << (field['bits'] - 1)) if field['signed'] else 0
                        high = (1 << (field['bits'] - (1 if field['signed'] else 0))) - 1
                        if not low <= value <= high:
                            raise ValueError('特征值超出扫描类型范围')
                        if field['name'] != 'in_timestamp':
                            values.append(value)
                    values = tuple(values)
                    if int(row['same_payload_as_previous']) != int(values == previous_values):
                        raise ValueError('特征重复标记与数据不符')
                    previous_values = values
                    if first_stamp is None:
                        first_stamp = stamp
                    previous = stamp
                    unit_count += 1
                if not unit_count or unit_count != entry['records']:
                    raise ValueError('特征记录计数不匹配')
                last_stamp = previous
                count += unit_count
            if (entry['first_iio_timestamp_ns'] != first_stamp or
                entry['last_iio_timestamp_ns'] != last_stamp):
                raise ValueError('元数据时间范围与载荷不符')
            units += 1
        if role == 'raw' and window_info is not None and window_files != len(window_info['files']):
            raise ValueError('最后一个窗口索引缺少批次')
        if count == 0 or count != state['records'] or units != state['units']:
            raise ValueError(f'{role} 的索引计数与会话摘要不匹配')
        counts[role] = count
    return counts


def parser():
    result = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument('--list', action='store_true', help='只读枚举 XYZS 设备，不配置或采集')
    result.add_argument('--verify', type=Path, help='只做已保存会话的离线完整性检查')
    result.add_argument('--output', type=Path, help='数据根目录；每次运行创建独立会话')
    result.add_argument('--raw-serdev', help='原始采集传感器的父实例，例如 serial0-0')
    result.add_argument('--feature-serdev', help='特征采集传感器的父实例，例如 serial1-0')
    result.add_argument('--lengths', nargs='+', type=int, default=SUPPORTED_LENGTHS, help='用户态轮换长度列表；默认 1024 2048 4096 8192 16384 32768')
    result.add_argument('--window-seconds', type=float, default=5, help='原始采集窗口，0.1～60 秒；不是总时长')
    result.add_argument('--seconds', type=float, default=60, help='会话时长，默认 60 秒；0 表示运行到 Ctrl+C')
    result.add_argument('--poll-ms', type=int, default=1000, help='特征轮询周期，100～60000 ms')
    result.add_argument('--feature-chunk-seconds', type=float, default=60, help='特征 CSV 分片时间')
    result.add_argument('--stall-seconds', type=float, default=30, help='任一路数据停止推进的超时')
    result.add_argument('--startup-timeout', type=float, default=60, help='等待传感器上线的超时')
    result.add_argument('--buffer-records', type=int, default=16384, help='原始 IIO buffer 的扫描记录容量')
    result.add_argument('--min-free-mib', type=int, default=512, help='磁盘剩余空间下限')
    result.add_argument('--machine-id', default='unlabeled', help='用户定义的被测机器标识')
    result.add_argument('--raw-sensor-id', default='unlabeled', help='原始角色的传感器自定义编号')
    result.add_argument('--feature-sensor-id', default='unlabeled', help='特征角色的传感器自定义编号')
    result.add_argument('--raw-mount', default='unlabeled', help='原始角色的实际安装位置')
    result.add_argument('--feature-mount', default='unlabeled', help='特征角色的实际安装位置')
    result.add_argument('--condition', default='unlabeled', help='用户确认的工况/故障标签；不自动推断')
    result.add_argument('--notes', default='', help='安装位置、转速、负载等实验说明')
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.verify:
        print(json.dumps(verify(args.verify), ensure_ascii=False))
        return 0
    if args.list:
        print(json.dumps(discover(), ensure_ascii=False, indent=2))
        return 0
    if not args.output or not args.raw_serdev or not args.feature_serdev:
        cli.error('采集需要 --output、--raw-serdev、--feature-serdev')
    for name in ('seconds', 'window_seconds', 'feature_chunk_seconds', 'stall_seconds', 'startup_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != 'seconds' and value == 0):
            cli.error(f'--{name.replace("_", "-")} 取值无效')
    if not 0.1 <= args.window_seconds <= 60 or not 100 <= args.poll_ms <= 60000:
        cli.error('原始窗口须为 0.1～60 秒，特征周期须为 100～60000 ms')
    if args.buffer_records < 1024 or args.min_free_mib < 0:
        cli.error('buffer 至少 1024 条，磁盘余量不能为负')
    validate_lengths(args.lengths, SUPPORTED_LENGTHS)
    pairs = discover()
    devices = {}
    for role, instance in [('raw', args.raw_serdev), ('feature', args.feature_serdev)]:
        if instance not in pairs or not {'raw', 'feature'} <= pairs[instance].keys():
            raise ValueError(f'找不到完整的 XYZS 设备对：{instance}')
        devices[role] = dict(pairs[instance], serdev=instance,
            sensor_id=getattr(args, role + '_sensor_id'), mount=getattr(args, role + '_mount'))
        for kind in ('raw', 'feature'):
            if read(Path(devices[role][kind]) / 'buffer/enable') != '0':
                raise RuntimeError(f'{instance} 有正在使用的 buffer；拒绝接管')
        schema = layout(Path(devices[role][role]))
        names = {field['name'] for field in schema['fields']}
        if 'in_timestamp' not in names or (role == 'raw' and names != set(RAW_NAMES)):
            raise ValueError('IIO 布局不符合当前记录器要求')
        if role == 'feature' and schema['fields'][-1]['name'] != 'in_timestamp':
            raise ValueError('时间戳不是最后一个扫描字段')
        devices[role]['schema'] = schema
    if devices['raw']['parent'] == devices['feature']['parent']:
        raise ValueError('原始和特征必须选择两个不同的物理传感器实例')
    raw_control = Path(devices['raw']['feature'])
    for name in ('sensor_raw_batch_samples', 'sensor_raw_batch_samples_available',
                 'sensor_raw_manual', 'sensor_raw_capture'):
        if not (raw_control / name).exists():
            raise RuntimeError('驱动缺少可变定长接口，请先部署新版模块')
    validate_lengths(args.lengths, [int(x) for x in
                     read(raw_control / 'sensor_raw_batch_samples_available').split()])
    if os.geteuid() != 0:
        raise PermissionError('配置 IIO 需要 root；先用 --list 确认映射，再使用 sudo 采集')
    locks = []
    children = {}
    root = None
    stop = mp.Event()
    old_handlers = {}
    try:
        for instance in sorted((args.raw_serdev, args.feature_serdev)):
            # 不删除锁文件，避免另一个进程持锁时产生不同 inode。
            path = Path(tempfile.gettempdir()) / ('ssf-training-' + instance + '.lock')
            fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            locks.append(fd)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        args.output.mkdir(parents=True, exist_ok=True)
        root = args.output / (dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8])
        root.mkdir()
        sync_directory(args.output)
        for role in devices:
            (root / role).mkdir()
        atomic_json(root / 'session.json', {
            'format_version': 2, 'session_id': root.name, 'started_utc': utc(), 'devices': devices,
            'arguments': {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
            'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'host_kernel': os.uname().release, 'iio_timestamp_clock': 'kernel default; host timestamp, not sensor sample time',
            'driver_srcversion': read(Path('/sys/module/ssf_mems/srcversion'))
                if Path('/sys/module/ssf_mems/srcversion').exists() else 'not exposed',
            'units': {'accel_scale_output': 'm/s^2', 'velocity_scale_output': 'm/s', 'temperature_scale_output': 'millidegree Celsius'},
            'quality_scope': 'structural validation; CRC performed inside driver; no metrology guarantee',
            'raw_is_contiguous_across_batches': False, 'cross_sensor_synchronization': False})
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
        for role, pair in devices.items():
            child = mp.Process(target=worker, args=(role, pair, args, root, stop), name='mems-' + role)
            child.start()
            children[role] = child
        print(f'会话目录：{root}', flush=True)
        started = time.monotonic()
        while not stop.wait(0.2):
            if args.seconds and time.monotonic() - started >= args.seconds:
                stop.set()
            if any(not child.is_alive() for child in children.values()):
                stop.set()
    finally:
        stop.set()
        wait_seconds = 120
        if root and (root / 'raw/capture_config.json').exists():
            settings = json.loads((root / 'raw/capture_config.json').read_text())
            maximum = max(args.lengths)
            wait_seconds = max(wait_seconds, args.window_seconds + maximum /
                float(settings['sensor_sampling_frequency']) + maximum / 64 * 392 * 10 /
                int(settings['sensor_baudrate']) + 30)
        deadline = time.monotonic() + wait_seconds
        for child in children.values():
            child.join(max(0, deadline - time.monotonic()))
        hung = []
        for role, child in children.items():
            if child.is_alive():
                hung.append(role)
                child.kill()
                child.join(5)
        if root:
            workers = {}
            for role in devices:
                path = root / role / 'status.json'
                workers[role] = json.loads(path.read_text()) if path.exists() else {'state': 'missing'}
            passed = len(children) == 2 and not hung and all(
                child.exitcode == 0 and workers[role].get('state') == 'completed'
                for role, child in children.items())
            atomic_json(root / 'summary.json', {'state': 'completed' if passed else 'failed',
                        'ended_utc': utc(), 'workers': workers, 'forced_stop': hung,
                        'manual_cleanup_required': bool(hung)})
        for fd in locks:
            os.close(fd)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    if root and passed:
        try:
            counts = verify(root)
        except Exception as error:
            summary = json.loads((root / 'summary.json').read_text())
            summary.update(state='failed', verification_error=repr(error))
            atomic_json(root / 'summary.json', summary)
            raise
        print('离线校验：', json.dumps(counts, ensure_ascii=False), flush=True)
        return 0
    print(f'会话失败，请检查 {root}/summary.json；不要把失败会话直接用作训练集。', file=sys.stderr)
    return 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f'错误：{error}', file=sys.stderr)
        raise SystemExit(1)
