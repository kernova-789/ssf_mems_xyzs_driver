#!/usr/bin/env python3
"""在定长首包前和上传中卸载；确认重载后仍能以 128000 正常通信。"""
import json
from pathlib import Path
import subprocess
import sys
import time

output = Path(sys.argv[1])
module = output / 'ssf_mems.ko'
result = {'cycles': []}


def find(name):
    """按名字获取 IIO 目录，不依赖本次设备编号。"""
    return next(p for p in Path('/sys/bus/iio/devices').glob('iio:device*')
                if (p / 'name').read_text().strip() == name)


def ready():
    """后台完成匹配后检查普通通信，不修改传感器参数。"""
    feature = find('ssf_mems_xyzs')
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if int((feature / 'sensor_online').read_text()) == 1:
            assert int((feature / 'sensor_baudrate').read_text()) == 128000
            return feature
        time.sleep(0.1)
    raise RuntimeError('sensor did not become online at 128000')


def snapshot(raw):
    """读取无串口副作用的批次统计；多字段不是原子快照。"""
    return {key: int((raw / key).read_text()) for key in
            ('raw_link_mode', 'raw_batch_packets', 'raw_stop_attempts',
             'raw_publishing', 'raw_batches')}


try:
    for label, mode in (('before_first_packet', 1), ('during_upload', 2)):
        feature = ready()
        raw = find('ssf_mems_xyzs_raw')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            state = snapshot(raw)
            count = state['raw_batch_packets']
            if state['raw_link_mode'] == mode and (
                    (mode == 1 and count == 0) or (mode == 2 and 2 <= count <= 10)):
                break
            time.sleep(0.01)
        else:
            raise RuntimeError('did not reach ' + label)
        before = subprocess.check_output(['dmesg'], text=True)
        row = {'label': label, 'before': state}
        result['cycles'].append(row)
        print('unload', label, state, flush=True)
        started = time.monotonic()
        subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=45)
        row['unload_seconds'] = round(time.monotonic() - started, 3)
        assert not feature.exists() and not raw.exists()
        after = subprocess.check_output(['dmesg'], text=True)
        # 日志可能环形滚动，按最后一条有限采集摘要判定，不按字符偏移截取。
        line = next(line for line in reversed(after.splitlines())
                    if 'finite raw capture:' in line)
        assert 'status -19' in line and 'unconfirmed' not in line, line
        row['completion_log'] = line
        (output / (label + '-kernel.log')).write_text(after)
        started = time.monotonic()
        subprocess.run(['insmod', str(module)], check=True, timeout=20)
        ready()
        row['reload_online_seconds'] = round(time.monotonic() - started, 3)
        reloaded = subprocess.check_output(['dmesg'], text=True)
        # 只比较新增的末尾：不应在重载时重复更改波特率和保存 Flash。
        last_online = reloaded.rfind('sensor online, baud rate 128000')
        last_config = reloaded.rfind('acquisition configuration:')
        assert last_online > last_config >= 0
        assert 'baud change:' not in reloaded[last_config:last_online]
        print('passed', row, flush=True)
    result['passed'] = True
except Exception as error:
    result['error'] = repr(error)
    raise
finally:
    (output / 'unload_summary.json').write_text(json.dumps(result, indent=2))
