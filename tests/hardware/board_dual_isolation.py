#!/usr/bin/env python3
"""两路使用相反采集模式和不同主机轮询周期，检查状态与数据是否独立。"""
import argparse
import errno
import json
import os
from pathlib import Path
import select
import time

from board_driver_functional import channels, configure, decode, read
from iio_devices import find_pair

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
parser.add_argument('--seconds', type=float, default=20)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
pairs = {port: find_pair(instance) for port, instance in
         [('RS485-1', 'serial0-0'), ('RS485-2', 'serial1-0')]}
features = {port: pair['ssf_mems_xyzs'] for port, pair in pairs.items()}
keys = ('sensor_acquisition_mode', 'sensor_poll_mode', 'sensor_poll_fixed_interval_ms')
saved = {port: {key: read(device / key) for key in keys} for port, device in features.items()}
result = {'passed': False, 'saved': saved, 'phases': []}


def fresh(device):
    until = time.monotonic() + 40
    while time.monotonic() < until:
        try:
            value = read(device / 'in_accel_x_rms_raw')
            if int(read(device / 'sensor_sample_age_ms')) < 2000:
                return value
        except OSError as error:
            assert error.errno == errno.ENODATA, error
        time.sleep(0.1)
    raise TimeoutError('feature mode failed to refresh')


def capture(label, modes):
    output = args.output / label
    output.mkdir()
    streams = {}
    opened = []
    poller = select.poll()
    try:
        for port, pair in pairs.items():
            for name, device in pair.items():
                is_raw = name.endswith('_raw')
                fields, size = configure(device, [field['index'] for field in channels(device)])
                fd = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
                opened.append((device, fd))
                (device / 'buffer/enable').write_text('1')
                poller.register(fd, select.POLLIN)
                streams[fd] = dict(port=port, raw=is_raw, fields=fields, size=size, data=bytearray())
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            for fd, event in poller.poll(50):
                assert not event & (select.POLLERR | select.POLLHUP | select.POLLNVAL), event
                stream = streams[fd]
                try:
                    blob = os.read(fd, stream['size'] * 4096)
                except BlockingIOError:
                    continue
                assert blob, 'unexpected EOF'
                stream['data'].extend(blob)
    finally:
        for device, fd in opened:
            (device / 'buffer/enable').write_text('0')
            os.close(fd)
    rows = {}
    for stream in streams.values():
        port = stream['port']
        kind = 'raw' if stream['raw'] else 'feature'
        blob = stream['data']
        (output / (port + '-' + kind + '.bin')).write_bytes(blob)
        row = decode(blob, stream['fields'], stream['size'], stream['raw'])
        rows[port + '-' + kind] = row
        if modes[port] == kind:
            assert row['records'] >= (1024 if kind == 'raw' else 5), (port, kind, row)
        else:
            assert row['records'] == 0, ('inactive data stream produced records', port, kind, row)
    return rows


try:
    assert all(read(device / 'buffer/enable') == '0' for pair in pairs.values() for device in pair.values())
    for port, period in [('RS485-1', 700), ('RS485-2', 1300)]:
        (features[port] / 'sensor_poll_mode').write_text('fixed')
        (features[port] / 'sensor_poll_fixed_interval_ms').write_text(str(period))
    result['periods'] = {port: read(device / 'sensor_poll_fixed_interval_ms') for port, device in features.items()}
    assert result['periods'] == {'RS485-1': '700', 'RS485-2': '1300'}
    for label, modes in [('first', {'RS485-1': 'feature', 'RS485-2': 'raw'}),
                          ('swapped', {'RS485-1': 'raw', 'RS485-2': 'feature'})]:
        for port, mode in modes.items():
            (features[port] / 'sensor_acquisition_mode').write_text(mode)
        for port, mode in modes.items():
            assert read(features[port] / 'sensor_acquisition_mode') == mode
            if mode == 'feature':
                fresh(features[port])
            else:
                try:
                    read(features[port] / 'in_accel_x_rms_raw')
                except OSError as error:
                    assert error.errno == errno.ENODATA, error
                else:
                    raise AssertionError('raw mode exposed a feature cache')
        row = {'name': label, 'modes': modes, 'streams': capture(label, modes)}
        result['phases'].append(row)
        print(json.dumps(row), flush=True)
    result['passed'] = True
except Exception as error:
    result['error'] = repr(error)
    raise
finally:
    for port, device in features.items():
        for key in ('sensor_poll_fixed_interval_ms', 'sensor_poll_mode', 'sensor_acquisition_mode'):
            (device / key).write_text(saved[port][key])
    result['restored'] = {port: {key: read(device / key) for key in keys} for port, device in features.items()}
    (args.output / 'isolation_summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
if not result['passed']:
    raise SystemExit(1)
