#!/usr/bin/env python3
"""校验独立连续回报实验的线上私有包；不把它解释为正式驱动IIO结果。"""
import argparse
import json
from pathlib import Path
import re


def crc_table():
    table = []
    for value in range(256):
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
        table.append(value)
    return table


TABLE = crc_table()


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc = (crc >> 8) ^ TABLE[(crc ^ byte) & 255]
    return crc


def packets(data):
    partial = bytearray()
    result = dict(packets=0, bad_candidates=0, sequence_gaps=0, ignored=0,
                  first_sequence=None, last_sequence=None)
    expected = 0
    for byte in data:
        if not partial and byte != 0x15:
            result['ignored'] += 1
            continue
        partial.append(byte)
        if len(partial) < 392:
            continue
        if partial[391] == 0x17 and crc16(partial[:391]) == 0:
            sequence = int.from_bytes(partial[1:5], 'big')
            if sequence != expected:
                result['sequence_gaps'] += 1
            expected = (sequence + 1) & 0xffffffff
            if result['first_sequence'] is None:
                result['first_sequence'] = sequence
            result['last_sequence'] = sequence
            result['packets'] += 1
            partial.clear()
        else:
            result['bad_candidates'] += 1
            next_header = partial.find(b'\x15', 1)
            partial = partial[next_header:] if next_header != -1 else bytearray()
    result['tail_bytes'] = len(partial)
    return result


def analyze(root):
    lines = (root / 'kernel.log').read_text().splitlines()
    result = {}
    for instance in ('serial0-0', 'serial1-0'):
        blob = (root / (instance + '-raw.bin')).read_bytes()
        rows = []
        row = None
        for line in lines:
            if instance + ':' not in line:
                continue
            values = {key: int(value) for key, value in re.findall(r'(\w+)=([0-9]+)', line)}
            if 'CL START ' in line:
                row = {'start': values}
                rows.append(row)
            elif row is not None and 'CL FIRST ' in line:
                row['first'] = values
            elif row is not None and 'CL STREAM ' in line:
                row['stream'] = values
            elif row is not None and 'CL FINISH ' in line:
                row['finish'] = values
                begin, end = row['start']['offset'], values['offset']
                assert 0 <= begin <= end <= len(blob), (instance, begin, end, len(blob))
                row['binary'] = packets(blob[begin:end])
                stream_end = row['stream']['offset']
                assert begin <= stream_end <= end
                row['steady_binary'] = packets(blob[begin:stream_end])
                # 测量截止可能在包中间，尾字节单列；最终停流文件仍严格检查残包。
                row['steady_passed'] = bool(values['full'] and values['stream_clean']
                                            and not values['blob_lost'] and row['steady_binary']['packets']
                                            and not any(row['steady_binary'][key] for key in
                                                        ('bad_candidates', 'sequence_gaps', 'ignored')))
                row['passed'] = bool(values['full'] and values['restored'] and values['stream_clean']
                                     and not values['blob_lost'] and row['binary']['packets']
                                     and not any(row['binary'][key] for key in
                                                 ('bad_candidates', 'sequence_gaps', 'ignored', 'tail_bytes')))
        result[instance] = {'bytes': len(blob), 'rounds': rows,
                            'passed': bool(rows) and all(row.get('passed', False) for row in rows)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = analyze(args.root)
    blob = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(blob)
    else:
        print(blob)


if __name__ == '__main__':
    main()
