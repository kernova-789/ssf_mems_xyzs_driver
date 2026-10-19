import unittest

from analyze_continuous_lab import crc16, packets


def packet(sequence):
    body = b'\x15' + sequence.to_bytes(4, 'big') + bytes(384)
    crc = crc16(body)
    return body + crc.to_bytes(2, 'little') + b'\x17'


class ContinuousParserTests(unittest.TestCase):
    def test_valid_and_truncated(self):
        data = packet(0) + packet(1) + packet(2)[:80]
        result = packets(data)
        self.assertEqual(result['packets'], 2)
        self.assertEqual(result['tail_bytes'], 80)
        self.assertEqual(result['sequence_gaps'], 0)
        self.assertEqual(result['bad_candidates'], 0)

    def test_corruption_resynchronizes(self):
        corrupt = bytearray(packet(0))
        corrupt[20] ^= 1
        result = packets(bytes(corrupt) + packet(1))
        self.assertGreater(result['bad_candidates'], 0)
        self.assertEqual(result['packets'], 1)
        self.assertEqual(result['sequence_gaps'], 1)

    def test_gap_and_ignored_bytes(self):
        result = packets(b'noise' + packet(0) + packet(2))
        self.assertEqual(result['ignored'], 5)
        self.assertEqual(result['sequence_gaps'], 1)

    def test_known_modbus_crc(self):
        self.assertEqual(crc16(bytes.fromhex('010300000001840a')), 0)


if __name__ == '__main__':
    unittest.main()
