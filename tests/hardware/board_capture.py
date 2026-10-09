#!/usr/bin/env python3
"""Bounded IIO hardware test; disable owned buffers even when capture fails."""
import errno
import json
import os
from pathlib import Path
import re
import select
import struct
import sys
import time

output = Path(sys.argv[1])
duration = float(sys.argv[2])
devices = {}
for path in Path('/sys/bus/iio/devices').glob('iio:device*'):
    name = (path / 'name').read_text().strip()
    if name in ('ssf_mems_xyzs', 'ssf_mems_xyzs_raw'):
        devices[name] = path
assert len(devices) == 2, devices

def write(path, value):
    """写入本次测试负责的 IIO 配置属性。"""
    path.write_text(str(value))

def layout(device):
    """按扫描索引和存储位宽计算当前全通道记录布局。"""
    result = []
    for path in (device / 'scan_elements').glob('*_index'):
        name = path.name[:-6]
        kind = path.with_name(name + '_type').read_text().strip()
        match = re.fullmatch(r'(le|be):([su])(\d+)/(\d+)>>(\d+)', kind)
        assert match is not None, kind
        result.append((int(path.read_text()), name, kind, int(match[4]) // 8))
    result.sort()
    offset = 0
    fields = []
    for index, name, kind, width in result:
        offset = (offset + width - 1) // width * width
        fields.append(dict(index=index, name=name, type=kind, offset=offset))
        offset += width
    return fields, offset

stats = {'raw_records': 0, 'feature_records': 0, 'raw_sessions': 0,
         'sequence_breaks': 0, 'bad_indices': 0, 'timestamp_regressions': 0,
         'raw_min': [32767] * 3, 'raw_max': [-32768] * 3,
         'raw_first': [], 'feature_first': [], 'phase_changes': [],
         'raw_status': [], 'feature_cache_valid': 0, 'feature_cache_enodata': 0}
opened = []
streams = {}
last_raw = None
raw_format = struct.Struct('<hhh2xIH2xq')
poller = select.poll()
start = time.monotonic()
next_status = start
last_active = None
try:
    for name, device in devices.items():
        assert (device / 'buffer/enable').read_text().strip() == '0', device
        fields, size = layout(device)
        stats[name] = dict(path=str(device), fields=fields, scan_bytes=size)
        assert size == (24 if name.endswith('_raw') else 72), (name, fields, size)
        for path in (device / 'scan_elements').glob('*_en'):
            write(path, 1)
        write(device / 'buffer/length', 8192 if name.endswith('_raw') else 64)
        fd = os.open('/dev/' + device.name, os.O_RDONLY | os.O_NONBLOCK)
        opened.append((device, fd))
        write(device / 'buffer/enable', 1)
        poller.register(fd, select.POLLIN)
        stream = (output / ('raw.bin' if name.endswith('_raw') else 'features.bin')).open('wb')
        streams[fd] = dict(name=name, size=size, output=stream, pending=b'')
    print('capture started', json.dumps({k: v['scan_bytes'] for k, v in stats.items()
                                         if isinstance(v, dict)}, ensure_ascii=False), flush=True)
    while time.monotonic() - start < duration:
        for fd, events in poller.poll(100):
            assert events & (select.POLLERR | select.POLLHUP | select.POLLNVAL) == 0, events
            stream = streams[fd]
            try:
                data = os.read(fd, stream['size'] * 4096)
            except BlockingIOError:
                continue
            assert data, 'unexpected IIO EOF'
            stream['output'].write(data)
            data = stream['pending'] + data
            complete = len(data) // stream['size'] * stream['size']
            stream['pending'] = data[complete:]
            if stream['name'].endswith('_raw'):
                for x, y, z, sequence, index, timestamp in raw_format.iter_unpack(data[:complete]):
                    row = (x, y, z, sequence, index, timestamp)
                    stats['raw_records'] += 1
                    if len(stats['raw_first']) < 8:
                        stats['raw_first'].append(row)
                    if index >= 64:
                        stats['bad_indices'] += 1
                    for axis, value in enumerate((x, y, z)):
                        stats['raw_min'][axis] = min(stats['raw_min'][axis], value)
                        stats['raw_max'][axis] = max(stats['raw_max'][axis], value)
                    if last_raw is None:
                        stats['raw_sessions'] += 1
                    else:
                        old_seq, old_index, old_timestamp = last_raw
                        if timestamp < old_timestamp:
                            stats['timestamp_regressions'] += 1
                        expected = ((old_seq + (old_index == 63)) & 0xffffffff,
                                    (old_index + 1) % 64)
                        if (sequence, index) != expected:
                            if sequence == 0 and index == 0 and timestamp - old_timestamp > 1000000000:
                                stats['raw_sessions'] += 1
                            else:
                                stats['sequence_breaks'] += 1
                    last_raw = sequence, index, timestamp
            else:
                for offset in range(0, complete, stream['size']):
                    row = data[offset:offset + stream['size']]
                    stats['feature_records'] += 1
                    if len(stats['feature_first']) < 8:
                        stats['feature_first'].append(dict(
                            xyz_rms=struct.unpack_from('<HHH', row, 26),
                            temperature=struct.unpack_from('<h', row, 12)[0],
                            timestamp=struct.unpack_from('<q', row, 64)[0]))
        now = time.monotonic()
        if now >= next_status:
            raw = devices['ssf_mems_xyzs_raw']
            values = {name: int((raw / name).read_text()) for name in
                      ('raw_active', 'raw_packets', 'raw_sequence_gaps',
                       'raw_crc_errors', 'raw_buffer_drops', 'raw_drain_packets',
                       'raw_drain_sequence_gaps', 'raw_drain_crc_errors',
                       'raw_start_wait_ms', 'raw_stop_attempts',
                       'raw_link_mode', 'raw_publishing')}
            values['elapsed'] = round(now - start, 3)
            stats['raw_status'].append(values)
            if (values['raw_active'], values['raw_link_mode'], values['raw_publishing']) != last_active:
                stats['phase_changes'].append(values)
                print('phase', json.dumps(values), flush=True)
                last_active = (values['raw_active'], values['raw_link_mode'], values['raw_publishing'])
            try:
                (devices['ssf_mems_xyzs'] / 'in_accel_x_rms_raw').read_text()
                stats['feature_cache_valid'] += 1
            except OSError as error:
                if error.errno != errno.ENODATA:
                    raise
                stats['feature_cache_enodata'] += 1
            next_status = now + 0.25
finally:
    for device, fd in opened:
        os.close(fd)
        write(device / 'buffer/enable', 0)
    for stream in streams.values():
        stream['output'].close()
        assert len(stream['pending']) == 0, stream['pending']
    stats['elapsed'] = round(time.monotonic() - start, 3)
    (output / 'capture_summary.json').write_text(json.dumps(stats, indent=2))

print(json.dumps({k: v for k, v in stats.items()
                  if k not in devices and k not in ('raw_status', 'phase_changes',
                                                     'raw_first', 'feature_first')}, indent=2), flush=True)
