#!/usr/bin/env python3
"""汇总调速实测：分别保留IIO完整性、驱动通信告警和UART错误，不混为零错误。"""
import argparse
import json
from pathlib import Path
import re


def uart_counts(text, number):
    line = next((line for line in text.splitlines() if re.match(rf'\s*{number}:', line)), '')
    if not line:
        raise ValueError(f'UART {number} missing from serial snapshot')
    # /proc省略值为零的错误字段；首次出现fe不能漏算。
    result = dict(fe=0, brk=0, oe=0, pe=0)
    result.update({key: int(value) for key, value in re.findall(r'\b(tx|rx|fe|brk|oe|pe):([0-9]+)', line)})
    return result


def analyze(root):
    result = []
    for path in sorted(root.glob('*/phase.json')):
        phase = json.loads(path.read_text())
        lines = (path.parent / 'kernel.log').read_text().splitlines()
        rows = {}
        for port, instance, number in [('RS485-1', 'serial0-0', 4), ('RS485-2', 'serial1-0', 7)]:
            relevant = [line for line in lines if instance + ':' in line]
            row = rows[port] = {'requested_baud': phase['rates'][port],
                                'configuration': phase['configuration'][port],
                                'readbacks': phase['readbacks'][port]}
            for name, pattern in [('candidate_crc_logs', 'invalid frame CRC/length'),
                                  ('acquisition_error_logs', 'sensor acquisition failed:'),
                                  ('connection_error_logs', 'sensor connection failed:'),
                                  ('communication_lost_logs', 'sensor communication lost'),
                                  ('finite_unconfirmed_logs', 'finite batch completion unconfirmed')]:
                row[name] = sum(pattern in line for line in relevant)
            row['finite_statuses'] = [int(match[1]) for line in relevant
                                      if (match := re.search(r'finite raw capture:.*status (-?\d+)', line))]
            capture = path.parent / 'capture' / port
            summary_path = capture / 'capture_summary.json'
            if summary_path.exists():
                summary = json.loads(summary_path.read_text())
                row['capture'] = {key: summary[key] for key in
                                  ('elapsed', 'raw_records', 'feature_records', 'sequence_breaks',
                                   'bad_indices', 'timestamp_regressions')}
                row['counter_peaks'] = {key: max((s[key] for s in summary['raw_status']), default=0)
                                        for key in ('raw_crc_errors', 'raw_sequence_gaps', 'raw_buffer_drops',
                                                    'raw_drain_crc_errors', 'raw_drain_sequence_gaps',
                                                    'raw_ignored_bytes')}
                for kind in ('summary', 'binary'):
                    check = capture / (kind + '-check.log')
                    row[kind + '_check'] = json.loads(check.read_text()) if check.exists() else None
            before = uart_counts((path.parent / 'uart-before.log').read_text(), number)
            after = uart_counts((path.parent / 'uart-after.log').read_text(), number)
            row['uart_before'] = before
            row['uart_after'] = after
            row['uart_delta'] = {key: after[key] - before[key] for key in before.keys() & after.keys()}
            row['strict_pass'] = (
                row['configuration']['actual'] == str(row['requested_baud']) and
                row['readbacks']['sensor_baudrate'] == str(row['requested_baud']) and
                all(not isinstance(value, dict) for value in row['readbacks'].values()) and
                (row.get('summary_check') or {}).get('passed', False) and
                (row.get('binary_check') or {}).get('passed', False) and
                not any(row[key] for key in ('candidate_crc_logs', 'acquisition_error_logs',
                                            'connection_error_logs', 'communication_lost_logs',
                                            'finite_unconfirmed_logs')) and
                all(status == 0 for status in row['finite_statuses']) and
                all(row['uart_delta'].get(key, 0) == 0 for key in ('fe', 'brk', 'oe', 'pe')))
        result.append({'label': phase['label'], 'seconds': phase['seconds'], 'ports': rows})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = {str(root): analyze(root) for root in args.roots}
    blob = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(blob)
    else:
        print(blob)


if __name__ == '__main__':
    main()
