#!/usr/bin/env python3
"""切换独立实验模块，保存每路原始字节及日志；所有实验串口帧由实验模块发送。"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
parser.add_argument('--module', type=Path, required=True)
parser.add_argument('--seconds', type=int, default=60)
parser.add_argument('--rounds', type=int, default=3)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
for path in Path('/sys/bus/iio/devices').glob('iio:device*/buffer/enable'):
    assert path.read_text().strip() == '0', path
result = {'seconds': args.seconds, 'rounds': args.rounds, 'status': {}}
log = (args.output / 'kernel-live.log').open('w')
follow = subprocess.Popen(['dmesg', '--follow-new'], stdout=log, stderr=subprocess.STDOUT)
try:
    (args.output / 'kernel-before.log').write_text(subprocess.check_output(['dmesg'], text=True))
    (args.output / 'uart-before.log').write_text(Path('/proc/tty/driver/serial').read_text())
    subprocess.run(['rmmod', 'ssf_mems'], check=True, timeout=60)
    (args.output / 'iio-unloaded.log').write_text('\n'.join(p.name for p in Path('/sys/bus/iio/devices').iterdir()))
    subprocess.run(['insmod', str(args.module), f'seconds={args.seconds}', f'rounds={args.rounds}',
                    'first_timeout=45'], check=True, timeout=20)
    deadline = time.monotonic() + args.rounds * (45 + args.seconds + 50) + 30
    while time.monotonic() < deadline:
        states = {}
        for instance in ('serial0-0', 'serial1-0'):
            path = Path('/sys/kernel/debug') / ('mems-continuous-' + instance) / 'status'
            states[instance] = path.read_text().strip()
        result['status'] = states
        if all('done=1 ' in state for state in states.values()):
            break
        time.sleep(2)
    else:
        raise TimeoutError('experiment deadline expired; retaining state for diagnosis')
    for instance in ('serial0-0', 'serial1-0'):
        path = Path('/sys/kernel/debug') / ('mems-continuous-' + instance)
        # debugfs inode大小可能为0，必须实际read，不能依赖sendfile或stat大小。
        with (path / 'raw.bin').open('rb') as source, (args.output / (instance + '-raw.bin')).open('wb') as target:
            shutil.copyfileobj(source, target)
    (args.output / 'uart-after.log').write_text(Path('/proc/tty/driver/serial').read_text())
    (args.output / 'kernel.log').write_text(subprocess.check_output(['dmesg'], text=True))
    result['completed'] = True
finally:
    follow.terminate()
    follow.wait(timeout=10)
    log.close()
    (args.output / 'lab_summary.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
# 保存证据后保留实验模块，必须先确认停流状态，再决定恢复正式驱动。
