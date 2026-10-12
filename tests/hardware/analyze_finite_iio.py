#!/usr/bin/env python3
"""离线逐记录核对目标 ARM64 的全通道 IIO 文件；不接触硬件。"""
import argparse
import json
import math
from pathlib import Path
import struct


def analyze_raw(blob):
    """按 24 字节布局检查每批 0..15/0..63、填充、时间戳和完整批数。"""
    if len(blob) % 24 != 0:
        raise ValueError('raw.bin has a partial IIO record')
    previous = None
    errors = 0
    complete = 0
    records = 0
    for offset in range(0, len(blob), 24):
        _, _, _, sequence, index, stamp = struct.unpack_from('<hhh2xIH2xq', blob, offset)
        pair = (sequence, index)
        expected = (0, 0) if previous is None else (
            (previous[0] + (previous[1] == 63)) % 16, (previous[1] + 1) % 64)
        if (pair != expected or sequence >= 16 or index >= 64 or stamp <= 0 or
                (previous is not None and stamp < previous[2]) or
                blob[offset + 6:offset + 8] != b'\0\0' or
                blob[offset + 14:offset + 16] != b'\0\0'):
            errors += 1
        if pair == (15, 63):
            complete += 1
        records += 1
        previous = (sequence, index, stamp)
    return dict(records=records, complete_batches=complete, errors=errors,
                tail_samples=records % 1024,
                last_position=None if previous is None else list(previous[:2]))


def analyze_progress(raw_blob, feature_blob, elapsed, limit=30.0):
    """用主机时间戳检查整轮覆盖和两路最长空档，防止晚期停采被掩盖。"""
    if not math.isfinite(elapsed) or elapsed <= 0 or not math.isfinite(limit) or limit <= 0:
        raise ValueError('capture interval and progress limit must be finite positive numbers')
    stamps = {
        'raw': [struct.unpack_from('<q', raw_blob, offset + 16)[0]
                for offset in range(0, len(raw_blob), 24)],
        'feature': [struct.unpack_from('<q', feature_blob, offset + 64)[0]
                    for offset in range(0, len(feature_blob), 72)],
    }
    if any(not values for values in stamps.values()):
        return dict(passed=False, failures=['one stream has no timestamps'])
    first = min(values[0] for values in stamps.values())
    last = max(values[-1] for values in stamps.values())
    coverage = (last - first) / 1e9
    failures = []
    if coverage < elapsed - limit:
        failures.append('timestamps do not cover the planned capture interval')
    gaps = {}
    for name, values in stamps.items():
        boundaries = [first] + values + [last]
        gaps[name] = max((right - left) / 1e9
                         for left, right in zip(boundaries, boundaries[1:]))
        if gaps[name] >= limit:
            failures.append(f'{name} stream stopped advancing for at least {limit:g}s')
    return dict(passed=not failures, coverage_seconds=round(coverage, 3),
                max_gap_seconds=gaps, failures=failures)


def main():
    """交叉核对二进制记录和在线摘要，返回可供测试脚本使用的退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--require-whole-batches', action='store_true')
    parser.add_argument('--progress-limit', type=float, default=30.0,
                        help='默认 5 秒交替配置下允许的最大无数据空档（默认 30 秒）')
    args = parser.parse_args()
    if not math.isfinite(args.progress_limit) or args.progress_limit <= 0:
        parser.error('--progress-limit must be a finite positive number')
    summary = json.loads((args.capture / 'capture_summary.json').read_text())
    raw_blob = (args.capture / 'raw.bin').read_bytes()
    raw = analyze_raw(raw_blob)
    feature = (args.capture / 'features.bin').read_bytes()
    failures = []
    if raw['errors'] != 0 or raw['records'] != summary['raw_records']:
        failures.append('raw records/sequence/timestamps/padding mismatch')
    if raw['records'] == 0:
        failures.append('no raw data')
    if args.require_whole_batches and raw['tail_samples'] != 0:
        failures.append('capture ends inside a finite batch')
    if len(feature) % 72 != 0 or len(feature) // 72 != summary['feature_records']:
        failures.append('feature record count/size mismatch')
    else:
        previous = 0
        for offset in range(0, len(feature), 72):
            stamp = struct.unpack_from('<q', feature, offset + 64)[0]
            if feature[offset + 57:offset + 64] != bytes(7):
                failures.append('feature scan padding is not zero')
                break
            if stamp <= 0 or stamp < previous:
                failures.append('feature timestamp regression')
                break
            previous = stamp
    progress = None
    if len(feature) % 72 == 0:
        progress = analyze_progress(raw_blob, feature, float(summary['elapsed']),
                                    args.progress_limit)
        failures.extend(progress['failures'])
    print(json.dumps(dict(passed=len(failures) == 0, raw=raw, progress=progress,
                          feature_records=len(feature) // 72, failures=failures), indent=2))
    return 0 if len(failures) == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
