"""验证同型号双传感器的 IIO 配对与歧义拒绝，不访问硬件。"""
from pathlib import Path
import tempfile
import unittest

from iio_devices import find_pair


class IioDiscoveryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'sysfs'
        self.root.mkdir()

    def add(self, instance, number, name):
        path = self.base / 'devices' / instance / f'iio:device{number}'
        path.mkdir(parents=True)
        (path / 'name').write_text(name)
        (self.root / path.name).symlink_to(path, target_is_directory=True)

    def test_single_instance_compatible(self):
        self.add('serial0-0', 1, 'ssf_mems_xyzs')
        self.add('serial0-0', 2, 'ssf_mems_xyzs_raw')
        self.assertEqual(set(find_pair(root=self.root)), {'ssf_mems_xyzs', 'ssf_mems_xyzs_raw'})

    def test_two_instances_require_selection(self):
        for instance, start in [('serial0-0', 1), ('serial1-0', 3)]:
            self.add(instance, start, 'ssf_mems_xyzs')
            self.add(instance, start + 1, 'ssf_mems_xyzs_raw')
        with self.assertRaisesRegex(RuntimeError, 'use --serdev'):
            find_pair(root=self.root)
        for instance in ('serial0-0', 'serial1-0'):
            pair = find_pair(instance, self.root)
            self.assertTrue(all(path.resolve().parent.name == instance for path in pair.values()))

    def test_incomplete_pairs_cannot_be_combined(self):
        self.add('serial0-0', 1, 'ssf_mems_xyzs')
        self.add('serial1-0', 2, 'ssf_mems_xyzs_raw')
        with self.assertRaisesRegex(RuntimeError, 'incomplete'):
            find_pair('serial0-0', self.root)

    def test_missing_instance(self):
        with self.assertRaisesRegex(RuntimeError, 'missing'):
            find_pair('serial9-0', self.root)


if __name__ == '__main__':
    unittest.main()
