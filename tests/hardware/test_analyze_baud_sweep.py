import unittest

from analyze_baud_sweep import uart_counts


class UartCountsTests(unittest.TestCase):
    def test_omitted_errors_are_zero(self):
        self.assertEqual(uart_counts('4: uart:16550A tx:12 rx:34 RTS', 4),
                         dict(tx=12, rx=34, fe=0, brk=0, oe=0, pe=0))

    def test_error_fields_and_instance(self):
        text = '4: uart:16550A tx:1 rx:2 fe:3\n7: uart:16550A tx:5 rx:6 fe:7 oe:8'
        self.assertEqual(uart_counts(text, 7), dict(tx=5, rx=6, fe=7, brk=0, oe=8, pe=0))

    def test_missing_port_is_not_zero_errors(self):
        with self.assertRaises(ValueError):
            uart_counts('4: uart:16550A tx:1 rx:2', 7)
