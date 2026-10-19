#!/usr/bin/env python3
"""不读取IIO字符设备、不发送配置读请求，仅被动观察正式驱动自主采集。"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from iio_devices import find_pair

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
parser.add_argument('--seconds', type=int, default=90)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
pairs = {port: find_pair(instance) for port, instance in
         [('RS485-1', 'serial0-0'), ('RS485-2', 'serial1-0')]}
for pair in pairs.values():
    for device in pair.values():
        assert (device / 'buffer/enable').read_text().strip() == '0'
    feature = pair['ssf_mems_xyzs']
    (feature / 'sensor_acquisition_mode').write_text('alternating')


def snapshot():
    result = {'boottime': time.clock_gettime(time.CLOCK_BOOTTIME), 'instances': {}}
    for port, pair in pairs.items():
        row = result['instances'][port] = {}
        for role, device in pair.items():
            keys = ['buffer/enable']
            if role.endswith('_raw'):
                keys += ['raw_packets', 'raw_crc_errors', 'raw_sequence_gaps', 'raw_buffer_drops']
            else:
                keys += ['sensor_online', 'sensor_acquisition_mode', 'sensor_sample_age_ms', 'in_accel_x_rms_raw']
            values = row[role] = {}
            for key in keys:
                try:
                    values[key] = (device / key).read_text().strip()
                except OSError as error:
                    values[key] = {'errno': error.errno}
    return result


result = {'before': snapshot(), 'seconds': args.seconds, 'snapshots': []}
for name, command in [('processes-before.log', ['ps', '-eo', 'pid,stat,args']),
                      ('uart-before.log', ['cat', '/proc/tty/driver/serial']),
                      ('kernel-before.log', ['dmesg'])]:
    (args.output / name).write_text(subprocess.check_output(command, text=True))
deadline = time.monotonic() + args.seconds
while time.monotonic() < deadline:
    time.sleep(min(5, max(0, deadline - time.monotonic())))
    result['snapshots'].append(snapshot())
result['after'] = snapshot()
for name, command in [('processes-after.log', ['ps', '-eo', 'pid,stat,args']),
                      ('uart-after.log', ['cat', '/proc/tty/driver/serial']),
                      ('kernel-after.log', ['dmesg'])]:
    (args.output / name).write_text(subprocess.check_output(command, text=True))
(args.output / 'autonomous_summary.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
