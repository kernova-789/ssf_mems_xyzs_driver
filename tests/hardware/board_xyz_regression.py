#!/usr/bin/env python3
"""Simultaneous XYZ capture via production IIO; verify aligned scans and whole batches."""
import argparse
import json
import os
from pathlib import Path
import re
import select
import struct
import time


def layout(device):
    fields = []
    for p in device.glob('scan_elements/*_index'):
        name = p.name[:-6]
        kind = (p.parent / (name + '_type')).read_text().strip()
        m = re.fullmatch(r'(le|be):([su])(\d+)/(\d+)>>(\d+)', kind)
        if m is None or int(m[5]):
            raise ValueError(kind)
        fields.append(dict(name=name, index=int(p.read_text()), width=int(m[4]) // 8))
    offset = 0
    alignment = 1
    for f in sorted(fields, key=lambda f: f['index']):
        alignment = max(alignment, f['width'])
        offset = (offset + f['width'] - 1) // f['width'] * f['width']
        f['offset'] = offset
        offset += f['width']
    return fields, (offset + alignment - 1) // alignment * alignment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('seconds', type=float)
    parser.add_argument('--rate0', type=int, choices=range(10))
    parser.add_argument('--rate1', type=int, choices=range(10))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    pairs = {}
    for d in Path('/sys/bus/iio/devices').glob('iio:device*'):
        name = (d / 'name').read_text().strip()
        if name in ('ssf_mems_xyz', 'ssf_mems_xyz_raw'):
            pairs.setdefault(d.resolve().parent.name, {})[name] = d
    assert len(pairs) == 2 and all(len(p) == 2 for p in pairs.values()), pairs
    streams = {}
    poll = select.poll()
    results = dict(passed=False, requested_seconds=args.seconds, instances={})
    started = time.monotonic()
    try:
        for port, pair in pairs.items():
            feature = pair['ssf_mems_xyz']
            (feature / 'sensor_acquisition_mode').write_text('feature')
            selected = args.rate0 if port == 'serial0-0' else args.rate1
            if selected is not None:
                (feature / 'sensor_sampling_rate_index').write_text(str(selected))
            row = dict(raw_records=0, feature_records=0, complete_batches=0,
                       errors=0, expected=0, last_stamp=0)
            results['instances'][port] = row
            for name, d in pair.items():
                assert (d / 'buffer/enable').read_text().strip() == '0'
                fields, size = layout(d)
                assert size == (24 if name.endswith('_raw') else 48), (fields, size)
                for p in d.glob('scan_elements/*_en'):
                    p.write_text('1')
                (d / 'buffer/length').write_text('8192')
                fd = os.open('/dev/' + d.name, os.O_RDONLY | os.O_NONBLOCK)
                raw = name.endswith('_raw')
                out = (args.output / (port + ('-raw.bin' if raw else '-features.bin'))).open('wb')
                streams[fd] = dict(device=d, port=port, raw=raw, size=size,
                                   pending=b'', output=out, done=False)
                poll.register(fd, select.POLLIN)
                (d / 'buffer/enable').write_text('1')
        for pair in pairs.values():
            (pair['ssf_mems_xyz'] / 'sensor_acquisition_mode').write_text('alternating')
        started = time.monotonic()
        while not all(s['done'] for s in streams.values() if s['raw']):
            elapsed = time.monotonic() - started
            if elapsed > args.seconds + 35:
                raise TimeoutError('capture did not finish a complete batch')
            for fd, ev in poll.poll(200):
                assert not ev & (select.POLLERR | select.POLLHUP | select.POLLNVAL), ev
                s = streams[fd]
                data = os.read(fd, s['size'] * 8192)
                assert data
                s['output'].write(data)
                b = s['pending'] + data
                end = len(b) // s['size'] * s['size']
                s['pending'] = b[end:]
                row = results['instances'][s['port']]
                for offset in range(0, end, s['size']):
                    if s['raw']:
                        x, y, z, seq, idx, stamp = struct.unpack_from('<hhh2xIH2xq', b, offset)
                        if (seq, idx) != divmod(row['expected'], 64) or stamp < row['last_stamp']:
                            row['errors'] += 1
                        row['expected'] = (row['expected'] + 1) % 1024
                        row['last_stamp'] = stamp
                        row['raw_records'] += 1
                        if seq == 15 and idx == 63:
                            row['complete_batches'] += 1
                    else:
                        stamp = struct.unpack_from('<q', b, offset + 40)[0]
                        if stamp <= 0:
                            row['errors'] += 1
                        row['feature_records'] += 1
                if s['raw'] and elapsed >= args.seconds and row['expected'] == 0:
                    (s['device'] / 'buffer/enable').write_text('0')
                    poll.unregister(fd)
                    s['done'] = True
        for port, pair in pairs.items():
            row = results['instances'][port]
            row['raw_status'] = {p.name: p.read_text().strip()
                                 for p in pair['ssf_mems_xyz_raw'].glob('raw_*') if p.is_file()}
        results['passed'] = all(r['raw_records'] >= 1024 and r['feature_records'] > 0
                                and r['errors'] == 0 and r['expected'] == 0
                                for r in results['instances'].values())
    finally:
        for fd, s in streams.items():
            (s['device'] / 'buffer/enable').write_text('0')
            os.close(fd)
            s['output'].close()
        results['elapsed_seconds'] = round(time.monotonic() - started, 3)
        (args.output / 'summary.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2), flush=True)
    return 0 if results['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
