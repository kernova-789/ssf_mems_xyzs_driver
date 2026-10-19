#!/usr/bin/env python3
"""Independently validate saved XYZ scans and summarize rate trials offline."""
import json
from pathlib import Path
import struct
import sys

root = Path(sys.argv[1])
rows = []
for file in sorted(root.glob('*/trial.json')):
    trial = json.loads(file.read_text())
    row = dict(name=file.parent.name, baud=trial['baud'], rate=trial['rate_index'],
               passed=trial['passed'], configuration=trial['configuration'], streams={})
    for raw in sorted((file.parent / 'capture').glob('*-raw.bin')):
        port = raw.name.removesuffix('-raw.bin')
        payload = raw.read_bytes()
        errors = []
        if len(payload) % 24:
            errors.append('incomplete scan')
        previous = 0
        for i in range(len(payload) // 24):
            x, y, z, sequence, index, stamp = struct.unpack_from('<hhh2xIH2xq', payload, i * 24)
            if (sequence, index) != divmod(i % 1024, 64):
                errors.append(f'bad sequence at {i}')
            if stamp <= 0 or stamp < previous:
                errors.append(f'bad timestamp at {i}')
            if payload[i * 24 + 6:i * 24 + 8] != bytes(2) or payload[i * 24 + 14:i * 24 + 16] != bytes(2):
                errors.append(f'nonzero padding at {i}')
            previous = stamp
        records = len(payload) // 24
        if records % 1024:
            errors.append('incomplete batch')
        feature = raw.with_name(port + '-features.bin').read_bytes()
        feature_errors = int(len(feature) % 48 != 0)
        for i in range(len(feature) // 48):
            feature_errors += int(struct.unpack_from('<q', feature, i * 48 + 40)[0] <= 0)
        row['streams'][port] = dict(raw_records=records, complete_batches=records // 1024,
                                    raw_validation_errors=len(errors), error_examples=errors[:10],
                                    feature_records=len(feature) // 48,
                                    feature_validation_errors=feature_errors)
    rows.append(row)
output = root / 'offline-analysis.json'
output.write_text(json.dumps(rows, indent=2))
print(json.dumps(rows, indent=2))
if any(s['raw_validation_errors'] or s['feature_validation_errors']
       for r in rows for s in r['streams'].values()):
    raise SystemExit(1)
