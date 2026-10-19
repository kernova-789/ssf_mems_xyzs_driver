#!/usr/bin/env python3
"""Restore both XYZ units to manual defaults, after confirming communication."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time

from board_xyz_rate_trial import checked

output = Path(sys.argv[1])
features = {d.resolve().parent.name: d
            for d in Path('/sys/bus/iio/devices').glob('iio:device*')
            if (d / 'name').read_text().strip() == 'ssf_mems_xyz'}
assert set(features) == {'serial0-0', 'serial1-0'}
results = {}


def restore(item):
    name, device = item
    row = results[name] = {}
    try:
        checked(device / 'sensor_acquisition_mode', 'feature')
        assert checked(device / 'sensor_online') == '1', 'recover communication before restoring'
        checked(device / 'sensor_sampling_rate_index', 9)
        assert checked(device / 'sensor_sampling_rate_index') == '9'
        checked(device / 'sensor_baudrate', 9600)
        deadline = time.monotonic() + 60
        while checked(device / 'sensor_online') != '1':
            if time.monotonic() > deadline:
                raise TimeoutError('default baud did not become online')
            time.sleep(0.2)
        assert checked(device / 'sensor_baudrate') == '9600'
        # Last, because baud changes save/reboot the unit.
        checked(device / 'sensor_sampling_length_index', 2)
        keys = ('sensor_online', 'sensor_baudrate', 'sensor_sampling_rate_index',
                'sensor_sampling_frequency', 'sensor_sampling_length_index',
                'sensor_acquisition_mode', 'buffer/enable')
        row.update({k: checked(device / k) for k in keys})
        assert [row[k] for k in keys] == ['1', '9600', '9', '26667', '2', 'feature', '0'], row
        raw = next(d for d in Path('/sys/bus/iio/devices').glob('iio:device*')
                   if d.resolve().parent.name == name and (d / 'name').read_text().strip() == 'ssf_mems_xyz_raw')
        row['raw'] = {k: checked(raw / k) for k in ('buffer/enable', 'raw_active', 'raw_link_mode')}
        assert all(v == '0' for v in row['raw'].values()), row
        row['passed'] = True
    except Exception as error:
        row.update(passed=False, error=repr(error))


with ThreadPoolExecutor(max_workers=2) as pool:
    list(pool.map(restore, features.items()))
output.write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
raise SystemExit(0 if all(r['passed'] for r in results.values()) else 1)
