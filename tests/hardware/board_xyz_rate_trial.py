#!/usr/bin/env python3
"""One bounded XYZ baud/sampling trial through production IIO; preserve failure evidence."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import subprocess
import sys
import time


def io(path, value=None):
    code = ('from pathlib import Path; import sys; p=Path(sys.argv[1]); '
            'print(p.read_text().strip()) if len(sys.argv)==2 else p.write_text(sys.argv[2])')
    cmd = [sys.executable, '-c', code, str(path)]
    if value is not None:
        cmd.append(str(value))
    started = time.monotonic()
    try:
        p = subprocess.run(cmd, text=True, capture_output=True, timeout=75)
        return dict(exit=p.returncode, value=p.stdout.strip(), stderr=p.stderr.strip(),
                    elapsed=round(time.monotonic() - started, 3))
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'bounded IIO operation timed out: {path}')


def checked(path, value=None):
    result = io(path, value)
    if result['exit']:
        raise RuntimeError(f'{path}: {result}')
    return result['value']


def kernel_lines():
    return subprocess.check_output(['dmesg'], text=True).splitlines()


def kernel_since(anchor):
    lines = kernel_lines()
    if not anchor:
        return lines
    matches = [i for i, line in enumerate(lines) if line == anchor[-1]]
    if not matches:
        raise RuntimeError('kernel log anchor was overwritten; evidence incomplete')
    return lines[matches[-1] + 1:]


def uart():
    text = Path('/proc/tty/driver/serial').read_text()
    ports = {}
    for name, number in [('serial0-0', 4), ('serial1-0', 7)]:
        line = next(l for l in text.splitlines() if re.match(rf'\s*{number}:', l))
        row = dict(tx=0, rx=0, fe=0, brk=0, oe=0, pe=0)
        row.update({k: int(v) for k, v in re.findall(r'\b(tx|rx|fe|brk|oe|pe):(\d+)', line)})
        ports[name] = row
    return ports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--baud', type=int, required=True)
    parser.add_argument('--rate', type=int, required=True, choices=range(10))
    parser.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    features = {}
    for d in Path('/sys/bus/iio/devices').glob('iio:device*'):
        if (d / 'name').read_text().strip() == 'ssf_mems_xyz':
            features[d.resolve().parent.name] = d
    assert set(features) == {'serial0-0', 'serial1-0'}, features
    result = dict(baud=args.baud, rate_index=args.rate, seconds=args.seconds,
                  passed=False, configuration={}, ports={})
    result['uart_start'] = uart()
    start = time.clock_gettime(time.CLOCK_BOOTTIME)
    log_anchor = kernel_lines()
    capture_start = None

    def modes():
        with ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(lambda d: checked(d / 'sensor_acquisition_mode', 'feature'), features.values()))

    def configure(item):
        name, d = item
        row = result['configuration'][name] = {}
        try:
            before = io(d / 'sensor_baudrate')
            row['before'] = before
            # Also confirm a same-value request: after a lost reboot reply,
            # actual baud and acquisition's reconnect target may differ.
            # The production setter only reads when the value already matches.
            row['baud_write'] = io(d / 'sensor_baudrate', args.baud)
            assert row['baud_write']['exit'] == 0, row['baud_write']
            deadline = time.monotonic() + 40
            while checked(d / 'sensor_online') != '1':
                if time.monotonic() > deadline:
                    raise TimeoutError('sensor did not become online')
                time.sleep(0.25)
            row['baud'] = checked(d / 'sensor_baudrate')
            assert row['baud'] == str(args.baud), row
            checked(d / 'sensor_sampling_rate_index', args.rate)
            row['rate_index'] = checked(d / 'sensor_sampling_rate_index')
            row['frequency'] = checked(d / 'sensor_sampling_frequency')
            assert row['rate_index'] == str(args.rate), row
            row['passed'] = True
        except Exception as error:
            row.update(passed=False, error=repr(error))
        return row

    try:
        modes()
        with ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(configure, features.items()))
        print(json.dumps(dict(configured=result['configuration'])), flush=True)
        if not all(r['passed'] for r in result['configuration'].values()):
            return 1
        if args.seconds == 0:
            result['passed'] = True
            return 0
        before = uart()
        result['uart_before_capture'] = before
        clocks = Path('/sys/kernel/debug/clk/clk_summary')
        if clocks.exists():
            (args.output / 'uart-clocks.log').write_text('\n'.join(
                line for line in clocks.read_text().splitlines() if re.search(r'uart[47]\b', line)) + '\n')
        capture_start = time.clock_gettime(time.CLOCK_BOOTTIME)
        capture_anchor = kernel_lines()
        with (args.output / 'capture-console.log').open('w') as log:
            child = subprocess.run([sys.executable, str(Path(__file__).with_name('board_xyz_regression.py')),
                                    str(args.output / 'capture'), str(args.seconds)],
                                   stdout=log, stderr=subprocess.STDOUT, timeout=args.seconds + 120)
        result['capture_exit'] = child.returncode
        modes()
        after = uart()
        result['uart_after_capture'] = after
        kernel = kernel_since(capture_anchor)
        (args.output / 'kernel-capture.log').write_text('\n'.join(kernel) + '\n')
        summary_file = args.output / 'capture/summary.json'
        summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
        for name, d in features.items():
            row = result['ports'][name] = {}
            relevant = [l for l in kernel if name + ':' in l]
            row['uart_delta'] = {k: after[name][k] - before[name][k] for k in before[name]}
            row['iio'] = summary.get('instances', {}).get(name, {})
            row['finite_statuses'] = [int(m[1]) for l in relevant
                                      if (m := re.search(r'finite raw capture:.*status (-?\d+)', l))]
            row['errors'] = {key: sum(pattern in l for l in relevant) for key, pattern in (
                ('candidate_crc', 'invalid frame CRC/length'),
                ('acquisition', 'sensor acquisition failed:'),
                ('connection', 'sensor connection failed:'),
                ('lost', 'sensor communication lost'),
                ('unconfirmed', 'finite batch completion unconfirmed'))}
            row['readback'] = {key: io(d / key) for key in
                               ('sensor_online', 'sensor_baudrate', 'sensor_sampling_rate_index')}
            row['strict_pass'] = (summary.get('passed', False) and child.returncode == 0
                and not any(row['errors'].values()) and bool(row['finite_statuses'])
                and all(s == 0 for s in row['finite_statuses'])
                and all(row['uart_delta'][k] == 0 for k in ('fe', 'brk', 'oe', 'pe'))
                and all(v['exit'] == 0 for v in row['readback'].values())
                and row['readback']['sensor_online']['value'] == '1'
                and row['readback']['sensor_baudrate']['value'] == str(args.baud)
                and row['readback']['sensor_sampling_rate_index']['value'] == str(args.rate))
        result['passed'] = all(p['strict_pass'] for p in result['ports'].values())
    except Exception as error:
        result['error'] = repr(error)
    finally:
        try:
            modes()
        except Exception as error:
            result['cleanup_error'] = repr(error)
            result['passed'] = False
        (args.output / 'kernel-all.log').write_text('\n'.join(kernel_since(log_anchor)) + '\n')
        result['uart_end'] = uart()
        result['uart_total_delta'] = {
            name: {k: result['uart_end'][name][k] - row[k] for k in row}
            for name, row in result['uart_start'].items()}
        result['kernel_start'] = start
        result['kernel_end'] = time.clock_gettime(time.CLOCK_BOOTTIME)
        (args.output / 'trial.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
