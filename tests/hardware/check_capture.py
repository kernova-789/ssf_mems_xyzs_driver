#!/usr/bin/env python3
"""检查双 IIO 采集记录；保存过正确数据并不代表整轮长测通过。"""
import argparse
import json
import math
from pathlib import Path


def capture_failures(summary, expected_seconds, recovery_limit=15.0,
                     progress_limit=30.0):
    """返回失败原因；分别检查时长、有效数据和持续恢复失败，不混入排空统计。"""
    failures = []
    elapsed = float(summary['elapsed'])
    if not math.isfinite(elapsed) or elapsed < expected_seconds - 0.5:
        failures.append(f'采集未完成：实际 {elapsed:.3f}s，要求 {expected_seconds:.3f}s')
    for name in ('raw_records', 'feature_records'):
        if summary[name] <= 0:
            failures.append(f'{name} 没有有效记录')
    for name in ('sequence_breaks', 'bad_indices', 'timestamp_regressions'):
        if summary[name] != 0:
            failures.append(f'{name}={summary[name]}')
    statuses = summary['raw_status']
    if len(statuses) == 0:
        failures.append('缺少链路状态记录')
    recovering_since = None
    recovery_failed = False
    previous = -1.0
    for status in statuses:
        now = float(status['elapsed'])
        if not math.isfinite(now) or now < 0 or now < previous or now > elapsed:
            failures.append('链路状态时间无效或倒退')
            break
        previous = now
        if status['raw_link_mode'] == 3:
            if recovering_since is None:
                recovering_since = now
            if now - recovering_since >= recovery_limit:
                recovery_failed = True
        else:
            recovering_since = None
    if recovering_since is not None and elapsed - recovering_since >= recovery_limit:
        recovery_failed = True
    if recovery_failed:
        failures.append(f'恢复未确认状态持续至少 {recovery_limit:g}s')
    # 新记录附有逐时累计数；旧归档没有这些字段时不伪造持续性结论。
    for name in ('raw_records', 'feature_records'):
        if statuses and all(name in status for status in statuses):
            count = 0
            advanced_at = 0.0
            stalled = False
            for status in statuses:
                now = float(status['elapsed'])
                value = int(status[name])
                if value < count or value > int(summary[name]):
                    failures.append(f'{name} 累计数无效或倒退')
                    break
                if now - advanced_at >= progress_limit:
                    stalled = True
                if value > count:
                    advanced_at = now
                count = value
            if elapsed - advanced_at >= progress_limit:
                stalled = True
            if stalled:
                failures.append(f'{name} 连续至少 {progress_limit:g}s 未推进')
    counters = ['raw_sequence_gaps', 'raw_crc_errors', 'raw_buffer_drops']
    if summary.get('finite_batch_samples') is not None:
        counters += ['raw_drain_sequence_gaps', 'raw_drain_crc_errors',
                     'raw_ignored_bytes']
    for name in counters:
        peak = max((status[name] for status in statuses), default=0)
        if peak != 0:
            failures.append(f'有效采集阶段 {name} 最大值为 {peak}')
    return failures


def main():
    """读取采集摘要并以退出码交付判定；不操作串口、IIO 或传感器。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--seconds', type=float, required=True,
                        help='本轮计划的完整采集秒数，不能填提前结束后的实际时长')
    parser.add_argument('--recovery-limit', type=float, default=15.0,
                        help='本轮测试允许连续处于恢复未确认状态的秒数（默认 15）')
    parser.add_argument('--progress-limit', type=float, default=30.0,
                        help='新格式记录允许任一数据流不推进的秒数（默认 30）')
    args = parser.parse_args()
    if (not math.isfinite(args.seconds) or args.seconds <= 0 or
            not math.isfinite(args.recovery_limit) or args.recovery_limit <= 0 or
            not math.isfinite(args.progress_limit) or args.progress_limit <= 0):
        parser.error('时间参数必须是有限正数')
    try:
        failures = capture_failures(json.loads(args.summary.read_text()),
                                    args.seconds, args.recovery_limit,
                                    args.progress_limit)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'FAIL: 摘要无法校验：{error}\n')
    print(json.dumps({'passed': len(failures) == 0, 'failures': failures},
                     ensure_ascii=False, indent=2))
    return 0 if len(failures) == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
