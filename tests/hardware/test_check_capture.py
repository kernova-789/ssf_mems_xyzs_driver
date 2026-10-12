#!/usr/bin/env python3
"""不依赖开发板的长测判定回归测试。"""
import unittest

from check_capture import capture_failures


def sample_summary():
    """构造一份包含短暂正常停流的有效摘要。"""
    statuses = []
    for elapsed, mode in ((0, 0), (5, 1), (8, 2), (13, 3), (14, 0), (20, 0)):
        statuses.append(dict(elapsed=elapsed, raw_link_mode=mode,
                             raw_sequence_gaps=0, raw_crc_errors=0,
                             raw_buffer_drops=0))
    return dict(elapsed=20.01, raw_records=26752, feature_records=8,
                sequence_breaks=0, bad_indices=0, timestamp_regressions=0,
                raw_status=statuses)


class CaptureValidationTest(unittest.TestCase):
    """覆盖曾经能保存正常段、但整轮长测已经失败的情况。"""

    def test_normal_stop_is_allowed(self):
        """正常的短暂停流不能被当成故障。"""
        self.assertEqual(capture_failures(sample_summary(), 20), [])

    def test_aborted_capture_is_not_a_pass(self):
        """计划 600 秒但只跑了 20 秒，即使数据完整也失败。"""
        self.assertTrue(capture_failures(sample_summary(), 600))

    def test_no_data_is_not_a_pass(self):
        """两个设备都必须实际交付数据。"""
        for name in ('raw_records', 'feature_records'):
            summary = sample_summary()
            summary[name] = 0
            self.assertTrue(capture_failures(summary, 20))

    def test_stuck_link_is_not_hidden_by_good_samples(self):
        """已有正常段也不能掩盖后来长期卡在恢复状态。"""
        summary = sample_summary()
        summary['elapsed'] = 60
        summary['raw_status'][4]['raw_link_mode'] = 3
        summary['raw_status'][5]['raw_link_mode'] = 3
        self.assertTrue(capture_failures(summary, 60))

    def test_primary_errors_fail_but_drain_errors_are_separate(self):
        """排空错误单独分析，不伪装成有效段错误；有效段异常必须失败。"""
        summary = sample_summary()
        summary['raw_status'][3]['raw_drain_crc_errors'] = 1
        self.assertEqual(capture_failures(summary, 20), [])
        for name in ('raw_crc_errors', 'raw_sequence_gaps', 'raw_buffer_drops'):
            summary = sample_summary()
            summary['raw_status'][2][name] = 1
            self.assertTrue(capture_failures(summary, 20))

    def test_invalid_status_time_fails(self):
        """时间损坏或缺少状态时不能给出通过判定。"""
        for value in (-1, 100, float('nan')):
            summary = sample_summary()
            summary['raw_status'][2]['elapsed'] = value
            self.assertTrue(capture_failures(summary, 20))
        summary['raw_status'] = []
        self.assertTrue(capture_failures(summary, 20))

    def test_finite_drain_and_stop_errors_fail(self):
        """定长模式不应发送停止命令，排空错误也不能隐藏。"""
        keys = ('raw_drain_sequence_gaps', 'raw_drain_crc_errors',
                'raw_ignored_bytes', 'raw_stop_attempts')
        for key in keys:
            summary = sample_summary()
            summary['finite_batch_samples'] = 1024
            for row in summary['raw_status']:
                row.update({name: 0 for name in keys})
            self.assertEqual(capture_failures(summary, 20), [])
            summary['raw_status'][2][key] = 1
            self.assertTrue(capture_failures(summary, 20))

    def test_each_stream_must_keep_progressing(self):
        """曾交付正常数据、随后无声停采也必须失败，不能只看链路模式。"""
        summary = sample_summary()
        summary['elapsed'] = 80
        summary['raw_status'] = []
        for elapsed in range(0, 81, 5):
            row = dict(elapsed=elapsed, raw_link_mode=0,
                       raw_sequence_gaps=0, raw_crc_errors=0,
                       raw_buffer_drops=0, raw_records=elapsed * 100,
                       feature_records=elapsed)
            summary['raw_status'].append(row)
        summary['raw_records'] = 8000
        summary['feature_records'] = 80
        self.assertEqual(capture_failures(summary, 80), [])
        for name in ('raw_records', 'feature_records'):
            with self.subTest(name=name):
                for row in summary['raw_status']:
                    row[name] = min(row['elapsed'], 20)
                errors = capture_failures(summary, 80)
                self.assertTrue(any(name in error and '未推进' in error
                                    for error in errors))
                for row in summary['raw_status']:
                    row[name] = row['elapsed'] * (100 if name == 'raw_records' else 1)

    def test_progress_counter_cannot_regress(self):
        """损坏的累计计数不能被当成恢复采集。"""
        summary = sample_summary()
        for index, row in enumerate(summary['raw_status']):
            row['raw_records'] = index
        summary['raw_status'][3]['raw_records'] = 0
        self.assertTrue(any('倒退' in error
                            for error in capture_failures(summary, 20)))


if __name__ == '__main__':
    unittest.main()
