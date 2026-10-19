#!/usr/bin/env python3
"""Exercise XYZ throughput rejection on one port while the other collects normally."""
import json
import os
from pathlib import Path
import select
import struct
import sys
import time

root = Path('/sys/bus/iio/devices')
pairs = {}
for d in root.glob('iio:device*'):
    name = (d / 'name').read_text().strip()
    if name in ('ssf_mems_xyz', 'ssf_mems_xyz_raw'):
        pairs.setdefault(d.resolve().parent.name, {})[name] = d
assert set(pairs) == {'serial0-0', 'serial1-0'}
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
result = {'passed': False, 'instances': {}}
streams = {}; poll = select.poll()
try:
    for port, pair in pairs.items():
        f = pair['ssf_mems_xyz']
        deadline = time.monotonic() + 45
        while (f / 'sensor_online').read_text().strip() != '1':
            if time.monotonic() >= deadline:
                raise TimeoutError('sensor did not become online')
            time.sleep(0.2)
        (f / 'sensor_acquisition_mode').write_text('feature')
        (f / 'sensor_sampling_rate_index').write_text('6' if port == 'serial0-0' else '3')
        result['instances'][port] = dict(raw_records=0, feature_records=0, sequence_errors=0)
        for name, d in pair.items():
            assert (d / 'buffer/enable').read_text().strip() == '0'
            for e in d.glob('scan_elements/*_en'): e.write_text('1')
            (d / 'buffer/length').write_text('8192')
            fd = os.open('/dev/' + d.name, os.O_RDONLY | os.O_NONBLOCK)
            streams[fd] = dict(device=d, port=port, raw=name.endswith('_raw'), pending=b'')
            poll.register(fd, select.POLLIN)
            (d / 'buffer/enable').write_text('1')
    for pair in pairs.values():
        (pair['ssf_mems_xyz'] / 'sensor_acquisition_mode').write_text('alternating')
    until = time.monotonic() + 25
    while time.monotonic() < until:
        for fd, ev in poll.poll(200):
            assert not ev & (select.POLLERR | select.POLLHUP | select.POLLNVAL)
            s = streams[fd]; row = result['instances'][s['port']]
            b = s['pending'] + os.read(fd, 24 * 8192)
            size = 24 if s['raw'] else 48
            end = len(b) // size * size; s['pending'] = b[end:]
            for offset in range(0, end, size):
                if s['raw']:
                    seq, idx = struct.unpack_from('<IH', b, offset + 8)
                    if (seq, idx) != divmod(row['raw_records'] % 1024, 64):
                        row['sequence_errors'] += 1
                    row['raw_records'] += 1
                else: row['feature_records'] += 1
    a, b = result['instances']['serial0-0'], result['instances']['serial1-0']
    result['passed'] = (a['raw_records'] == 0 and a['feature_records'] > 0 and
                        b['raw_records'] >= 1024 and b['feature_records'] > 0 and
                        b['sequence_errors'] == 0)
finally:
    (out / 'summary.json').write_text(json.dumps(result, indent=2))
    for fd, s in streams.items():
        (s['device'] / 'buffer/enable').write_text('0'); os.close(fd)
    for pair in pairs.values():
        f = pair['ssf_mems_xyz']
        (f / 'sensor_acquisition_mode').write_text('feature')
        (f / 'sensor_sampling_rate_index').write_text('3')
    (out / 'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
if not result['passed']: raise SystemExit(1)
