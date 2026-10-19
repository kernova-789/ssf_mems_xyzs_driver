#!/usr/bin/env python3
"""双路正式驱动调速筛查；脚本仅操作 IIO，不发送串口帧或补发失败请求。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from iio_devices import find_pair


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--rates', nargs='+', type=int,
                        default=[128000, 115200, 230400, 256000, 460800, 500000,
                                 512000, 600000, 750000, 921600, 1000000])
    parser.add_argument('--no-mixed', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    scripts = Path(__file__).resolve().parent
    features = {port: find_pair(instance)['ssf_mems_xyzs'] for port, instance in
                [('RS485-1', 'serial0-0'), ('RS485-2', 'serial1-0')]}
    saved = {p: {k: (d / k).read_text().strip() for k in
                 ('sensor_acquisition_mode', 'sensor_poll_mode', 'sensor_poll_fixed_interval_ms')}
             for p, d in features.items()}
    result = {'saved': saved, 'phases': [], 'restored': {}}
    log = (args.output / 'kernel-live.log').open('w')
    follow = subprocess.Popen(['dmesg', '--follow-new'], stdout=log, stderr=subprocess.STDOUT)

    def bounded_write(path, value):
        started = time.monotonic()
        command = [sys.executable, '-c',
                   'from pathlib import Path; import sys; Path(sys.argv[1]).write_text(sys.argv[2])',
                   str(path), str(value)]
        try:
            proc = subprocess.run(command, capture_output=True, text=True, timeout=90)
            return {'value': str(value), 'exit': proc.returncode,
                    'stderr': proc.stderr, 'elapsed': round(time.monotonic() - started, 3)}
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(f'bounded sysfs write timed out: {path}; no retry') from error

    def set_modes(mode):
        with ThreadPoolExecutor(max_workers=2) as pool:
            rows = list(pool.map(lambda d: bounded_write(d / 'sensor_acquisition_mode', mode), features.values()))
        assert all(row['exit'] == 0 for row in rows), rows

    def configure(item):
        port, baud = item
        device = features[port]
        try:
            before = (device / 'sensor_baudrate').read_text().strip()
        except OSError as error:
            before = {'errno': error.errno, 'error': repr(error)}
        write = {'skipped_same_value': True, 'value': before}
        if before != str(baud):
            write = bounded_write(device / 'sensor_baudrate', baud)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if (device / 'sensor_online').read_text().strip() == '1':
                try:
                    actual = (device / 'sensor_baudrate').read_text().strip()
                except OSError as error:
                    actual = {'errno': error.errno, 'error': repr(error)}
                return port, {'before': before, 'requested': baud, 'write': write,
                              'actual': actual, 'online': True}
            time.sleep(0.2)
        return port, {'before': before, 'requested': baud, 'write': write,
                      'actual': None, 'online': False}

    def phase(label, rates, seconds):
        output = args.output / label
        output.mkdir()
        row = {'label': label, 'rates': rates, 'seconds': seconds,
               'kernel_start': time.clock_gettime(time.CLOCK_BOOTTIME)}
        result['phases'].append(row)
        print(json.dumps({'starting': row}), flush=True)
        set_modes('feature')
        with ThreadPoolExecutor(max_workers=2) as pool:
            row['configuration'] = dict(pool.map(configure, rates.items()))
        (output / 'uart-before.log').write_text(Path('/proc/tty/driver/serial').read_text())
        child = None
        try:
            with (output / 'capture-console.log').open('w') as out:
                child = subprocess.Popen([sys.executable, str(scripts / 'board_dual_capture.py'),
                                          str(output / 'capture'), '--seconds', str(seconds)],
                                         stdout=out, stderr=subprocess.STDOUT)
                time.sleep(1)
                set_modes('alternating')
                row['capture_exit'] = child.wait(timeout=seconds + 100)
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=30)
            set_modes('feature')
        (output / 'uart-after.log').write_text(Path('/proc/tty/driver/serial').read_text())
        row['readbacks'] = {}
        for port, device in features.items():
            values = row['readbacks'][port] = {}
            for key in ('sensor_online', 'sensor_baudrate', 'sensor_sampling_rate_index', 'sensor_firmware_version'):
                try:
                    values[key] = (device / key).read_text().strip()
                except OSError as error:
                    values[key] = {'errno': error.errno, 'error': repr(error)}
        row['kernel_end'] = time.clock_gettime(time.CLOCK_BOOTTIME)
        kernel = subprocess.check_output(['dmesg'], text=True)
        lines = []
        for line in kernel.splitlines():
            stamp = re.match(r'\[\s*([\d.]+)\]', line)
            if stamp and row['kernel_start'] <= float(stamp[1]) <= row['kernel_end']:
                lines.append(line)
        (output / 'kernel.log').write_text('\n'.join(lines) + '\n')
        (output / 'phase.json').write_text(json.dumps(row, indent=2))
        (args.output / 'sweep_summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({'finished': row}), flush=True)

    try:
        for baud in args.rates:
            phase(str(baud), {'RS485-1': baud, 'RS485-2': baud}, args.seconds)
        if not args.no_mixed:
            phase('mixed-high-low', {'RS485-1': 1000000, 'RS485-2': 128000}, 90)
            phase('mixed-low-high', {'RS485-1': 128000, 'RS485-2': 1000000}, 90)
    finally:
        try:
            set_modes('feature')
            with ThreadPoolExecutor(max_workers=2) as pool:
                result['restored_bauds'] = dict(pool.map(configure, [('RS485-1', 128000), ('RS485-2', 128000)]))
            for port, device in features.items():
                for key, value in saved[port].items():
                    assert bounded_write(device / key, value)['exit'] == 0
                result['restored'][port] = {key: (device / key).read_text().strip() for key in saved[port]}
        finally:
            follow.terminate()
            follow.wait(timeout=10)
            log.close()
            (args.output / 'sweep_summary.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
