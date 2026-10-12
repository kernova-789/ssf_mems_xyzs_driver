#!/usr/bin/env python3
"""验证实机测试脚本的布局计算与数据判定，不访问开发板。"""
from pathlib import Path
import struct
import tempfile
import unittest

from board_driver_functional import configure, decode


class FunctionalHarnessTest(unittest.TestCase):
    """防止测试程序自身的对齐错误被误认为驱动故障。"""

    def setUp(self):
        """构造最小原始 IIO sysfs 夹具。"""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.device = Path(self.temp.name)
        (self.device / 'buffer').mkdir()
        (self.device / 'buffer/enable').write_text('0')
        (self.device / 'scan_elements').mkdir()
        for index, (name, kind) in enumerate((
                ('in_accel_x', 'le:s16/16>>0'), ('in_accel_y', 'le:s16/16>>0'),
                ('in_accel_z', 'le:s16/16>>0'), ('in_count0_packet_sequence', 'le:u32/32>>0'),
                ('in_count1_sample_index', 'le:u16/16>>0'), ('in_timestamp', 'le:s64/64>>0'))):
            base = self.device / 'scan_elements' / name
            base.with_name(name + '_index').write_text(str(index))
            base.with_name(name + '_type').write_text(kind)
            base.with_name(name + '_en').write_text('0')

    def test_kernel_scan_alignment(self):
        """无时间戳扫描也要按最大字段宽度向上对齐。"""
        for selected, expected in (([0], 2), ([0, 3, 4], 12),
                                   ([0, 1, 2, 3, 4], 16),
                                   ([1, 3, 4, 5], 24)):
            with self.subTest(selected=selected):
                fields, size = configure(self.device, selected)
                self.assertEqual(size, expected)
                self.assertEqual([field['index'] for field in fields], selected)

    def test_valid_batch_boundary_and_padding(self):
        """起点可以在批次中间，但后续必须连续，填充必须为零。"""
        fields, size = configure(self.device, [0, 1, 2, 3, 4, 5])
        data = struct.pack('<hhh2xIH2xq', -1, 2, 3, 15, 63, 100)
        data += struct.pack('<hhh2xIH2xq', 4, 5, 6, 0, 0, 200)
        self.assertEqual(decode(data, fields, size, True)['records'], 2)
        damaged = bytearray(data)
        damaged[6] = 1
        with self.assertRaises(AssertionError):
            decode(damaged, fields, size, True)

    def test_missing_point_and_reversed_clock(self):
        """序号缺点和时间倒退都不允许成为通过结果。"""
        fields, size = configure(self.device, [0, 1, 2, 3, 4, 5])
        for index, timestamp in ((2, 200), (1, 50)):
            data = struct.pack('<hhh2xIH2xq', 1, 2, 3, 0, 0, 100)
            data += struct.pack('<hhh2xIH2xq', 1, 2, 3, 0, index, timestamp)
            with self.assertRaises(AssertionError):
                decode(data, fields, size, True)

    def test_active_buffer_cannot_be_reconfigured(self):
        """脚本不得主动改动正在由其他消费者使用的 buffer。"""
        (self.device / 'buffer/enable').write_text('1')
        with self.assertRaises(AssertionError):
            configure(self.device, [0])


if __name__ == '__main__':
    unittest.main()
