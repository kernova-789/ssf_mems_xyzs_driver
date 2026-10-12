#!/usr/bin/env python3
"""定长 IIO 离线检查器的纯内存回归测试。"""
import struct
import unittest

from analyze_finite_iio import analyze_progress, analyze_raw


def batch():
    """构造两个完整批次，时间戳用主机时间而非等间隔采样时间。"""
    return b''.join(struct.pack('<hhh2xIH2xq', -1, 2, -3, n // 64 % 16,
                                n % 64, 1000 + n) for n in range(2048))


class FiniteBinaryTest(unittest.TestCase):
    """合法重置、缺点和损坏记录必须能明确区分。"""

    def test_two_complete_batches(self):
        """15/63→0/0 是合法批次边界，不依赖一秒时间间隔。"""
        result = analyze_raw(batch())
        self.assertEqual((result['complete_batches'], result['errors']), (2, 0))
        self.assertEqual(result['tail_samples'], 0)

    def test_drop_and_partial_record(self):
        """删除完整扫描记录也必须报错；半条扫描记录直接拒绝。"""
        data = batch()
        self.assertGreater(analyze_raw(data[:240] + data[264:])['errors'], 0)
        with self.assertRaises(ValueError):
            analyze_raw(data[:-1])

    def test_timestamp_and_padding(self):
        """序号正确不能掩盖时间倒退或非零填充。"""
        data = bytearray(batch())
        struct.pack_into('<q', data, 24 + 16, 1)
        self.assertGreater(analyze_raw(data)['errors'], 0)
        data = bytearray(batch())
        data[6] = 1
        self.assertGreater(analyze_raw(data)['errors'], 0)

    def test_late_stall_and_missing_coverage(self):
        """两路曾有数据不够，整轮的尾部停采也必须检测。"""
        def raw_at(seconds):
            return b''.join(struct.pack('<hhh2xIH2xq', 1, 2, 3, 0, 0,
                                        int(second * 1e9)) for second in seconds)

        def feature_at(seconds):
            return b''.join(bytes(64) + struct.pack('<q', int(second * 1e9))
                            for second in seconds)

        raw = raw_at(range(10, 76, 5))
        features = feature_at(range(5, 81, 5))
        self.assertTrue(analyze_progress(raw, features, 80)['passed'])
        self.assertFalse(analyze_progress(raw_at([10, 15]), features, 80)['passed'])
        self.assertFalse(analyze_progress(raw, features, 150)['passed'])
        self.assertFalse(analyze_progress(raw, b'', 80)['passed'])
        for limit in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                analyze_progress(raw, features, 80, limit)


if __name__ == '__main__':
    unittest.main()
