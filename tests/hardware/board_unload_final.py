#!/usr/bin/env python3
"""验证首包前/收流中卸载，并检查重载后没有遗留私有流。"""
import json
from pathlib import Path
import re
import subprocess
import sys
import time

output = Path(sys.argv[1])
result = {'cycles': []}

def find(name):
    """按名字发现设备，避免依赖 IIO 编号。"""
    return next(p for p in Path('/sys/bus/iio/devices').glob('iio:device*')
                if (p / 'name').read_text().strip() == name)

def snapshot(raw):
    """只读诊断成员；状态边界允许跨越，判断时重复检查。"""
    return {key: int((raw / key).read_text()) for key in
            ('raw_link_mode', 'raw_packets', 'raw_start_wait_ms',
             'raw_stop_attempts', 'raw_publishing')}

def new_log(before, after):
    """按内核时间提取新增日志，避免 dmesg 环形缓冲滚动后按字符数截错。"""
    stamps = re.findall(r'^\[\s*(\d+\.\d+)\]', before, re.MULTILINE)
    assert stamps, 'missing kernel timestamps'
    cutoff = float(stamps[-1])
    lines = []
    for line in after.splitlines():
        stamp = re.match(r'^\[\s*(\d+\.\d+)\]', line)
        if stamp is not None and float(stamp[1]) > cutoff:
            lines.append(line)
    return '\n'.join(lines) + '\n'

def ready():
    """等待普通读回成功；sysfs 本身可能等待总线锁。"""
    feature = find('ssf_mems_xyzs')
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if int((feature / 'sensor_online').read_text()) == 1:
            assert int((feature / 'sensor_baudrate').read_text()) == 9600
            return feature
        time.sleep(0.1)
    raise RuntimeError('sensor did not become online at 9600')

try:
    for label, wanted_mode in (('before_first_packet', 1), ('while_streaming', 2)):
        feature = ready()
        raw = find('ssf_mems_xyzs_raw')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            state = snapshot(raw)
            if state['raw_link_mode'] == wanted_mode and (
                    (wanted_mode == 1 and state['raw_packets'] == 0) or
                    (wanted_mode == 2 and 20 < state['raw_packets'] < 300)):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError('did not reach unload condition: ' + label)
        before = subprocess.check_output(['dmesg'], text=True)
        row = {'label': label, 'before': state}
        result['cycles'].append(row)
        print('unload', label, state, flush=True)
        started = time.monotonic()
        subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=55)
        row['unload_seconds'] = round(time.monotonic() - started, 3)
        row['iio_removed'] = not feature.exists() and not raw.exists()
        assert row['iio_removed']
        unloaded = subprocess.check_output(['dmesg'], text=True)
        unload_log = new_log(before, unloaded)
        (output / (label + '_unload.log')).write_text(unload_log)
        assert 'remains unconfirmed' not in unload_log
        assert 'raw stop unconfirmed' not in unload_log
        assert 'raw capture:' in unload_log and 'status -19' in unload_log
        started = time.monotonic()
        subprocess.run(['insmod', str(output / 'ssf_mems-final.ko')],
                       check=True, timeout=20)
        feature = ready()
        row['reload_online_seconds'] = round(time.monotonic() - started, 3)
        reloaded = subprocess.check_output(['dmesg'], text=True)
        reload_log = new_log(unloaded, reloaded)
        (output / (label + '_reload.log')).write_text(reload_log)
        assert 'leftover private raw stream' not in reload_log
        assert 'temporary raw baud detected' not in reload_log
        assert 'baud change:' not in reload_log
        assert 'sensor online, baud rate 9600' in reload_log
        print('passed', row, flush=True)
    result['pass'] = True
except Exception as error:
    result['error'] = repr(error)
    raise
finally:
    (output / 'final_unload_summary.json').write_text(json.dumps(result, indent=2))
    (output / 'kernel_final.log').write_text(subprocess.check_output(['dmesg'], text=True))
    print(json.dumps(result, indent=2), flush=True)
