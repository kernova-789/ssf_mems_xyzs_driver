#!/usr/bin/env python3
"""Verify ordinary length writes independently of the driver's finite batch length."""
import errno
import json
from pathlib import Path
import sys
import time

output = Path(sys.argv[1])
results = {}
for d in Path('/sys/bus/iio/devices').glob('iio:device*'):
    if (d / 'name').read_text().strip() != 'ssf_mems_xyz':
        continue
    (d / 'sensor_acquisition_mode').write_text('feature')
    deadline = time.monotonic() + 60
    while (d / 'sensor_online').read_text().strip() != '1':
        if time.monotonic() > deadline:
            raise TimeoutError(str(d))
        time.sleep(0.2)
    p = d / 'sensor_sampling_length_index'
    p.write_text('5')
    assert p.read_text().strip() == '5'
    p.write_text('2')
    assert p.read_text().strip() == '2'
    p.write_text('2')
    try:
        p.write_text('6')
    except OSError as error:
        assert error.errno == errno.EINVAL
    else:
        raise AssertionError('invalid length accepted')
    assert p.read_text().strip() == '2'
    results[d.resolve().parent.name] = dict(valid_writes=[5, 2], same_value=2,
                                           invalid_value=6, invalid_errno=errno.EINVAL,
                                           final_length_index=2)
assert len(results) == 2
output.write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
