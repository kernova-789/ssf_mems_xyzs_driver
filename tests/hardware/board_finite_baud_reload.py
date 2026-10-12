#!/usr/bin/env python3
"""一次实际调速对照：128000→115200，重载后自动回到正式目标 128000。"""
import json
from pathlib import Path
import subprocess
import sys
import time

output = Path(sys.argv[1])
result = {}


def feature_device():
    """按设备名找到特征 IIO 目录，不固定编号。"""
    return next(p for p in Path('/sys/bus/iio/devices').glob('iio:device*')
                if (p / 'name').read_text().strip() == 'ssf_mems_xyzs')


def verify_online(baud):
    """等待自动匹配和采样配置完成，再用驱动接口读回速率。"""
    feature = feature_device()
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if int((feature / 'sensor_online').read_text()) == 1:
            assert int((feature / 'sensor_baudrate').read_text()) == baud
            assert int((feature / 'sensor_sampling_rate_index').read_text()) == 6
            return feature
        time.sleep(0.1)
    raise RuntimeError('online verification timed out')


try:
    feature = verify_online(128000)
    started = time.monotonic()
    # 子进程使阻塞的 sysfs 写也有超时上限；失败绝不重发保存命令。
    subprocess.run([sys.executable, '-c',
                    'import pathlib,sys; pathlib.Path(sys.argv[1]).write_text("115200")',
                    str(feature / 'sensor_baudrate')], check=True, timeout=60)
    result['manual_change_seconds'] = round(time.monotonic() - started, 3)
    assert int((feature / 'sensor_baudrate').read_text()) == 115200
    result['intermediate_baud'] = 115200
    subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=45)
    started = time.monotonic()
    subprocess.run(['insmod', str(output / 'ssf_mems.ko')], check=True, timeout=20)
    verify_online(128000)
    result['automatic_connect_seconds'] = round(time.monotonic() - started, 3)
    log = subprocess.check_output(['dmesg'], text=True)
    log = log[log.rfind('acquisition configuration:'):]
    (output / 'baud-reload-kernel.log').write_text(log)
    assert 'sensor matched: baud 115200, target 128000' in log
    assert log.count('saving/rebooting via 40110=1') == 1
    assert 'baud change verified: sensor 128000, host 128000' in log
    result['final_baud'] = 128000
    result['passed'] = True
    print(json.dumps(result), flush=True)
except Exception as error:
    result['error'] = repr(error)
    raise
finally:
    (output / 'baud_reload_summary.json').write_text(json.dumps(result, indent=2))
