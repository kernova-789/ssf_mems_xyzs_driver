#!/usr/bin/env python3
"""同时记录两路 serdev 传感器；每路独立保存、校验并保留真实退出码。"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
parser.add_argument('--seconds', type=float, default=300)
parser.add_argument('--module', type=Path, help='可选：在启动记录前加载正式模块')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
scripts = Path(__file__).resolve().parent
results = {'passed': False, 'seconds': args.seconds, 'instances': {}}
children = []
logs = []
started = time.monotonic()
try:
    if args.module:
        subprocess.run(['insmod', str(args.module)], check=True, timeout=20)
    for port, instance in [('RS485-1', 'serial0-0'), ('RS485-2', 'serial1-0')]:
        output = args.output / port
        output.mkdir()
        log = (output / 'console.log').open('w')
        logs.append(log)
        child = subprocess.Popen([sys.executable, str(scripts / 'board_capture.py'),
                                  str(output), str(args.seconds), '--serdev', instance],
                                 stdout=log, stderr=subprocess.STDOUT)
        children.append((port, instance, output, child))
    for port, instance, output, child in children:
        code = child.wait(timeout=max(1, args.seconds + 90 - (time.monotonic() - started)))
        row = {'serdev': instance, 'capture_exit': code}
        results['instances'][port] = row
        for name, command in [
            ('summary', ['check_capture.py', str(output / 'capture_summary.json'),
                         '--seconds', str(args.seconds)]),
            ('binary', ['analyze_finite_iio.py', str(output), '--require-whole-batches'])]:
            checked = subprocess.run([sys.executable, str(scripts / command[0])] + command[1:],
                                     text=True, capture_output=True, timeout=30)
            row[name + '_exit'] = checked.returncode
            (output / (name + '-check.log')).write_text(checked.stdout + checked.stderr)
        row['passed'] = all(row[name] == 0 for name in ('capture_exit', 'summary_exit', 'binary_exit'))
        print(json.dumps({port: row}), flush=True)
    results['passed'] = all(row['passed'] for row in results['instances'].values())
except Exception as error:
    results['error'] = repr(error)
    raise
finally:
    for _, _, _, child in children:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    for log in logs:
        log.close()
    results['elapsed'] = round(time.monotonic() - started, 3)
    (args.output / 'dual_summary.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results), flush=True)
if not results['passed']:
    raise SystemExit(1)
