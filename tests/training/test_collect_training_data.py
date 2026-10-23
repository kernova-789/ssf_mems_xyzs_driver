#!/usr/bin/env python3
"""训练记录器离线测试；合成数据不代表实机已通过，运行方式见 README.md。"""
import json
import os
from pathlib import Path
import signal
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import collect_training_data as capture


def raw_schema():
    specs = [('in_accel_x', 0, 2, True), ('in_accel_y', 2, 2, True),
             ('in_accel_z', 4, 2, True), ('in_count0_packet_sequence', 8, 4, False),
             ('in_count1_sample_index', 12, 2, False), ('in_timestamp', 16, 8, True)]
    return {'scan_bytes': 24, 'fields': [dict(name=name, offset=offset, width=width,
             bits=width * 8, signed=signed, shift=0, byteorder='little')
             for name, offset, width, signed in specs]}


def raw_batch(start=1000, samples=1024):
    return b''.join(struct.pack('<hhh2xIH2xq', -123, 45, -67, n // 64, n % 64,
                                start + n) for n in range(samples))


def feature_schema():
    return {'scan_bytes': 16, 'fields': [
        dict(name='in_temp', offset=0, width=2, bits=16, signed=True, shift=0, byteorder='little'),
        dict(name='in_timestamp', offset=8, width=8, bits=64, signed=True, shift=0, byteorder='little')]}


class BatchTests(unittest.TestCase):
    def feed(self, blob, validator=None):
        validator = validator or capture.BatchValidator(capture.Decoder(raw_schema()), 1024)
        results = []
        for offset in range(0, len(blob), 24):
            result = validator.feed(blob[offset:offset + 24])
            if result is not None:
                results.append(result)
        return validator, results

    def test_only_whole_batches_and_signed_values(self):
        validator, results = self.feed(raw_batch()[:24 * 1023])
        self.assertEqual(results, [])
        self.assertEqual(validator.count, 1023)
        result = validator.feed(raw_batch()[-24:])
        self.assertEqual(len(result[0]), 24576)
        self.assertEqual(capture.Decoder(raw_schema()).decode(result[0][:24])['in_accel_x'], -123)
        self.assertEqual(validator.count, 0)

    def test_two_batches_reset_packet_number(self):
        _, results = self.feed(raw_batch() + raw_batch(10000))
        self.assertEqual(len(results), 2)

    def test_drop_duplicate_and_wrong_start_rejected(self):
        blob = raw_batch()
        for damaged in (blob[24:], blob[:240] + blob[264:], blob[:240] + blob[216:]):
            with self.subTest(size=len(damaged)), self.assertRaises(ValueError):
                self.feed(damaged)

    def test_padding_and_timestamp_rejected(self):
        for offset, value in [(6, 1), (14, 1)]:
            blob = bytearray(raw_batch())
            blob[offset] = value
            with self.assertRaises(ValueError):
                self.feed(blob)
        blob = bytearray(raw_batch())
        struct.pack_into('<q', blob, 40, 1)
        with self.assertRaises(ValueError):
            self.feed(blob)
        with self.assertRaises(ValueError):
            self.feed(raw_batch() + raw_batch(1))

    def test_real_length_limit_is_not_bypassed(self):
        capture.validate_lengths([1024, 2048, 32768], capture.SUPPORTED_LENGTHS)
        for lengths, actual in [([1024, 2048], [1024]), ([65536], capture.SUPPORTED_LENGTHS), ([0], [1024])]:
            with self.assertRaises(ValueError):
                capture.validate_lengths(lengths, actual)

    def test_unsupported_length_never_discovers_or_writes_devices(self):
        with patch.object(capture, 'discover') as discover, self.assertRaises(ValueError):
            capture.main(['--output', '/unused', '--raw-serdev', 'a', '--feature-serdev', 'b',
                          '--lengths', '1024', '1000'])
        discover.assert_not_called()


class LayoutTests(unittest.TestCase):
    def test_dynamic_layout_endian_shift_and_tail_alignment(self):
        with tempfile.TemporaryDirectory() as temporary:
            device = Path(temporary)
            scan = device / 'scan_elements'
            scan.mkdir()
            for name, index, kind in [('in_timestamp', 5, 'be:s64/64>>0'),
                                      ('in_temp', 0, 'be:s12/16>>4')]:
                (scan / (name + '_index')).write_text(str(index))
                (scan / (name + '_type')).write_text(kind)
            schema = capture.layout(device)
            self.assertEqual(schema['scan_bytes'], 16)
            row = bytes.fromhex('fff0') + bytes(6) + (100).to_bytes(8, 'big')
            values = capture.Decoder(schema).decode(row)
            self.assertEqual(values, {'in_temp': -1, 'in_timestamp': 100})

    def test_two_identical_names_are_paired_by_parent(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bus = root / 'iio'
            bus.mkdir()
            for index, instance, name in [(0, 'serial0-0', 'ssf_mems_xyzs'),
                                           (1, 'serial1-0', 'ssf_mems_xyzs'),
                                           (2, 'serial0-0', 'ssf_mems_xyzs_raw'),
                                           (3, 'serial1-0', 'ssf_mems_xyzs_raw')]:
                target = root / instance / f'iio:device{index}'
                target.mkdir(parents=True)
                (target / 'name').write_text(name)
                (bus / target.name).symlink_to(target)
            pairs = capture.discover(bus)
            self.assertEqual(set(pairs), {'serial0-0', 'serial1-0'})
            self.assertEqual(Path(pairs['serial0-0']['raw']).name, 'iio:device2')


class DatasetTests(unittest.TestCase):
    def populate(self, root):
        raw_layout, feature_layout = raw_schema(), feature_schema()
        for field in raw_layout['fields']:
            if field['name'].startswith('in_accel_'):
                field.update(scale='0.004788403', unit_after_scale='m/s^2')
        feature_layout['fields'][0].update(scale='10', unit_after_scale='millidegree Celsius')
        capture.atomic_json(root / 'session.json', {'format_version': 2, 'session_id': 'unit-test',
            'arguments': {'machine_id': 'machine-test', 'condition': 'unlabeled', 'notes': ''},
            'devices': {
            'raw': {'schema': raw_layout, 'sensor_id': 'sensor-A', 'mount': 'bearing-A'},
            'feature': {'schema': feature_layout, 'sensor_id': 'sensor-B', 'mount': 'bearing-B'}}})
        raw = capture.Dataset(root, 'raw', 0)
        features = capture.Dataset(root, 'feature', 0)
        entry = raw.commit(raw_batch(), 'bin', {'samples': 1024, 'window_id': 1,
            'sampling_frequency_hz': '5333.4', 'first_iio_timestamp_ns': 1000,
            'last_iio_timestamp_ns': 2023})
        features.commit(b'received_realtime_ns,same_payload_as_previous,in_temp,in_timestamp\n'
                        b'100000,0,-100,1000\n', 'csv', {'records': 1,
                        'first_iio_timestamp_ns': 1000, 'last_iio_timestamp_ns': 1000})
        health = {name: 0 for name in capture.ERROR_STATS}
        health.update(window_id=1, raw_batch_samples=1024, raw_batches=1,
                      raw_packets=16, raw_sampling_frequency='5333.4')
        capture.atomic_json(root / 'raw/window-00000001.json', {'samples': 1024,
            'started_monotonic_ns': 1, 'ended_monotonic_ns': 2,
            'files': [entry['payload']], 'health': health})
        capture.atomic_json(root / 'summary.json', {'state': 'completed', 'workers': {
            'raw': {'records': 1024, 'units': 1, 'state': 'completed', 'cleanup_errors': []},
            'feature': {'records': 1, 'units': 1, 'state': 'completed', 'cleanup_errors': []}}})
        return raw, features

    def test_commit_and_verify_and_quarantine(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            raw, _ = self.populate(root)
            raw.reject(b'bad tail', 'partial')
            self.assertEqual(capture.verify(root), {'raw': 1024, 'feature': 1})
            self.assertEqual(len(list(root.rglob('*.partial'))), 0)

    def test_disk_full_never_commits_index(self):
        with tempfile.TemporaryDirectory() as temporary:
            dataset = capture.Dataset(temporary, 'raw', 1)
            usage = type('Usage', (), {'free': 0})()
            with patch.object(capture.shutil, 'disk_usage', return_value=usage), self.assertRaises(OSError):
                dataset.commit(b'abc', 'bin', {})
            self.assertFalse((Path(temporary) / 'raw/index.jsonl').exists())

    def test_tamper_and_failed_session_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.populate(root)
            path = next((root / 'raw').rglob('*.bin'))
            path.write_bytes(b'bad')
            with self.assertRaises(ValueError):
                capture.verify(root)
            capture.atomic_json(root / 'summary.json', {'state': 'failed'})
            with self.assertRaises(ValueError):
                capture.verify(root)

    def test_duplicate_index_and_uncommitted_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.populate(root)
            (root / 'raw/orphan.bin.partial').write_bytes(b'not indexed')
            self.assertEqual(capture.verify(root)['raw'], 1024)
            index = root / 'raw/index.jsonl'
            index.write_text(index.read_text() * 2)
            with self.assertRaises(ValueError):
                capture.verify(root)


    def test_window_proof_is_required_and_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.populate(root)
            proof = root / 'raw/window-00000001.json'
            value = json.loads(proof.read_text())
            value['health']['raw_crc_errors'] = 1
            capture.atomic_json(proof, value)
            with self.assertRaises(ValueError):
                capture.verify(root)
            proof.unlink()
            with self.assertRaises(OSError):
                capture.verify(root)

    def test_feature_semantics_beyond_checksum(self):
        import hashlib
        for replacement in ('100000,0,40000,1000', '100000,1,-100,1000', '100000,0,-100,999'):
            with self.subTest(replacement=replacement), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.populate(root)
                index = root / 'feature/index.jsonl'
                entry = next(capture.index_entries(index))
                path = root / entry['payload']
                payload = path.read_text().replace('100000,0,-100,1000', replacement).encode()
                path.write_bytes(payload)
                entry.update(bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())
                capture.atomic_json(path.with_suffix('.json'), entry)
                index.write_text(json.dumps(entry) + '\n')
                with self.assertRaises(ValueError):
                    capture.verify(root)

    def test_reader_units_identity_and_catalog(self):
        from training_dataset import TrainingDataset
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'session'
            root.mkdir()
            self.populate(root)
            dataset = TrainingDataset(root)
            batch = next(dataset.raw_batches(physical=True))
            self.assertEqual(batch['sensor_id'], 'sensor-A')
            self.assertEqual(batch['mount'], 'bearing-A')
            self.assertEqual(batch['xyz_counts'][0], [-123, 45, -67])
            self.assertAlmostEqual(batch['xyz_m_s2'][0][0], -123 * 0.004788403)
            self.assertEqual(len(batch['sample_time_seconds']), 1024)
            feature = next(dataset.feature_rows(physical=True))
            self.assertEqual(feature['sensor_id'], 'sensor-B')
            self.assertEqual(feature['physical']['in_temp'], -1000)
            destination = Path(temporary) / 'catalog.csv'
            dataset.catalog(destination)
            import csv
            with destination.open() as stream:
                entries = list(csv.DictReader(stream))
            self.assertEqual([row['role'] for row in entries], ['raw', 'feature'])
            self.assertEqual(entries[0]['samples'], '1024')
            with self.assertRaises(FileExistsError):
                dataset.catalog(destination)
            with self.assertRaises(ValueError):
                dataset.catalog(root / 'catalog.csv')


class WorkerTests(unittest.TestCase):
    def test_user_schedule_and_failed_window_never_publishes_index(self):
        for failure in ('none', 'driver_error', 'wrong_count', 'crc'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                feature, raw = root / 'feature-device', root / 'raw-device'
                feature.mkdir(); raw.mkdir()
                (feature / 'sensor_acquisition_mode').write_text('raw')
                (feature / 'sensor_raw_batch_samples').write_text('1024')
                dataset = capture.Dataset(root, 'raw', 0)
                pair = {'feature': str(feature), 'raw': str(raw)}
                args = capture.parser().parse_args([])
                args.lengths = [1024, 2048]
                args.min_free_mib = 0
                stop = threading.Event()
                status = {'records': 0, 'units': 0}
                settings = {'sensor_sampling_frequency': '5333.4', 'sensor_baudrate': '128000'}
                reader, writer = os.pipe()
                os.set_blocking(reader, False)
                requested = []
                real_write = capture.write
                def fake_write(path, value):
                    if path.name != 'sensor_raw_capture':
                        return real_write(path, value)
                    samples = int(capture.read(feature / 'sensor_raw_batch_samples'))
                    requested.append(samples)
                    payload = raw_batch(100000 * len(requested), samples)
                    for offset in range(0, len(payload), 997):
                        os.write(writer, payload[offset:offset + 997])
                    for name in capture.ERROR_STATS:
                        (raw / name).write_text('1' if failure == 'crc' and name == 'raw_crc_errors' else '0')
                    (raw / 'raw_batch_samples').write_text(str(samples))
                    (raw / 'raw_packets').write_text(str(samples // 64))
                    (raw / 'raw_batches').write_text('2' if failure == 'wrong_count' else '1')
                    (raw / 'raw_sampling_frequency').write_text('5333.4')
                    if failure != 'none' or len(requested) == 2:
                        stop.set()
                    if failure == 'driver_error':
                        raise OSError('模拟窗口收尾失败')
                try:
                    with patch.object(capture, 'write', side_effect=fake_write):
                        if failure == 'none':
                            capture.capture_raw_windows(pair, args, root, stop, reader,
                                capture.Decoder(raw_schema()), settings, status, dataset)
                        else:
                            with self.assertRaises((ValueError, OSError)):
                                capture.capture_raw_windows(pair, args, root, stop, reader,
                                    capture.Decoder(raw_schema()), settings, status, dataset)
                finally:
                    os.close(reader); os.close(writer)
                index = root / 'raw/index.jsonl'
                if failure == 'none':
                    self.assertEqual(requested, [1024, 2048])
                    entries = list(capture.index_entries(index))
                    self.assertEqual([e['samples'] for e in entries], [1024, 2048])
                    self.assertEqual(status['records'], 3072)
                    self.assertEqual([e['bytes'] for e in entries], [24576, 49152])
                else:
                    self.assertFalse(index.exists())

    def test_feature_snapshot_csv_and_failure_quarantine(self):
        for regress in (False, True):
            with self.subTest(regress=regress), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                feature = root / 'iio:device0'
                raw = root / 'iio:device1'
                (feature / 'buffer').mkdir(parents=True)
                (feature / 'scan_elements').mkdir()
                raw.mkdir()
                for name, value in {'sensor_acquisition_mode': 'alternating',
                    'sensor_raw_duration_ms': '5000', 'sensor_poll_mode': 'adaptive',
                    'sensor_poll_fixed_interval_ms': '2000', 'sensor_online': '1',
                    'sensor_sampling_frequency': '5333.4', 'sensor_sampling_rate_index': '6',
                    'sensor_firmware_version': '5100', 'sensor_baudrate': '128000',
                    'sensor_raw_batch_samples': '1024', 'sensor_raw_manual': '0',
                    'sensor_raw_batch_samples_available': '1024 2048 4096 8192 16384 32768',
                    'sensor_raw_capture': '0'}.items():
                    (feature / name).write_text(value)
                (raw / 'raw_batch_samples').write_text('1024')
                for name, value in [('enable', '0'), ('length', '16'), ('watermark', '1')]:
                    (feature / 'buffer' / name).write_text(value)
                for name in ('in_temp', 'in_timestamp'):
                    (feature / 'scan_elements' / (name + '_en')).write_text('0')
                session = root / 'session'
                session.mkdir()
                args = capture.parser().parse_args([])
                args.min_free_mib = 0
                args.lengths = [1024]
                pair = {'feature': str(feature), 'raw': str(raw), 'schema': feature_schema()}
                read_fd, write_fd = os.pipe()
                os.set_blocking(read_fd, False)
                stop = threading.Event()
                def send():
                    time.sleep(0.05)
                    os.write(write_fd, struct.pack('<h6xq', -123, 1000) +
                             struct.pack('<h6xq', -123, 900 if regress else 1100))
                    # 给工作循环时间读取完整快照，然后模拟正常停止。
                    time.sleep(0.1)
                    stop.set()
                thread = threading.Thread(target=send)
                real_open = os.open
                def fake_open(path, flags, *args):
                    return read_fd if str(path) == '/dev/iio:device0' else real_open(path, flags, *args)
                handlers = [signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)]
                try:
                    thread.start()
                    with patch.object(capture.os, 'open', side_effect=fake_open):
                        if regress:
                            with self.assertRaises(SystemExit):
                                capture.worker('feature', pair, args, session, stop)
                        else:
                            capture.worker('feature', pair, args, session, stop)
                finally:
                    stop.set()
                    thread.join(2)
                    os.close(write_fd)
                    for sig, handler in zip((signal.SIGINT, signal.SIGTERM), handlers):
                        signal.signal(sig, handler)
                self.assertEqual(capture.read(feature / 'sensor_acquisition_mode'), 'alternating')
                self.assertEqual(capture.read(feature / 'buffer/enable'), '0')
                files = list((session / 'feature').rglob('*.csv'))
                if regress:
                    self.assertEqual(files, [])
                    self.assertTrue((session / 'rejected').exists())
                else:
                    import csv
                    with files[0].open() as stream:
                        rows = list(csv.DictReader(stream))
                    self.assertEqual([row['in_temp'] for row in rows], ['-123', '-123'])
                    self.assertEqual([row['same_payload_as_previous'] for row in rows], ['0', '1'])

    def test_raw_capture_and_failure_restore_owned_configuration(self):
        # 用管道模拟 IIO read/poll，其他 sysfs 操作只写临时普通文件。
        for corrupt in (False, True):
            with self.subTest(corrupt=corrupt), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                feature = root / 'iio:device0'
                raw = root / 'iio:device1'
                feature.mkdir()
                (raw / 'buffer').mkdir(parents=True)
                (raw / 'scan_elements').mkdir()
                values = {'sensor_acquisition_mode': 'alternating', 'sensor_raw_duration_ms': '5000',
                          'sensor_poll_mode': 'adaptive', 'sensor_poll_fixed_interval_ms': '2000',
                          'sensor_online': '1', 'sensor_sampling_frequency': '5333.4',
                          'sensor_sampling_rate_index': '6', 'sensor_firmware_version': '5100', 'sensor_baudrate': '128000',
                    'sensor_raw_batch_samples': '1024', 'sensor_raw_manual': '0',
                    'sensor_raw_batch_samples_available': '1024 2048 4096 8192 16384 32768',
                    'sensor_raw_capture': '0'}
                for name, value in values.items():
                    (feature / name).write_text(value)
                for name in capture.ERROR_STATS:
                    (raw / name).write_text('0')
                (raw / 'raw_sampling_frequency').write_text('5333.4')
                (raw / 'raw_batch_samples').write_text('1024')
                for name, value in [('enable', '0'), ('length', '16'), ('watermark', '1')]:
                    (raw / 'buffer' / name).write_text(value)
                for name in capture.RAW_NAMES:
                    (raw / 'scan_elements' / (name + '_en')).write_text('0')
                session = root / 'session'
                session.mkdir()
                args = capture.parser().parse_args([])
                args.min_free_mib = 0
                args.lengths = [1024]
                pair = {'feature': str(feature), 'raw': str(raw), 'schema': raw_schema()}
                read_fd, write_fd = os.pipe()
                os.set_blocking(read_fd, False)
                stop = threading.Event()
                payload = bytearray(raw_batch())
                if corrupt:
                    payload[6] = 1
                real_open = os.open
                real_write = capture.write
                def fake_open(path, flags, *args):
                    if str(path) == '/dev/iio:device1':
                        return read_fd
                    return real_open(path, flags, *args)
                def fake_write(path, value):
                    if path.name != 'sensor_raw_capture':
                        return real_write(path, value)
                    for offset in range(0, len(payload), 997):
                        if stop.is_set():
                            return
                        os.write(write_fd, payload[offset:offset + 997])
                    (raw / 'raw_batch_samples').write_text('1024')
                    (raw / 'raw_packets').write_text('16')
                    (raw / 'raw_batches').write_text('1')
                    stop.set()
                old_int = signal.getsignal(signal.SIGINT)
                old_term = signal.getsignal(signal.SIGTERM)
                try:
                    with patch.object(capture.os, 'open', side_effect=fake_open), \
                         patch.object(capture, 'write', side_effect=fake_write):
                        if corrupt:
                            with self.assertRaises(SystemExit):
                                capture.worker('raw', pair, args, session, stop)
                        else:
                            capture.worker('raw', pair, args, session, stop)
                finally:
                    stop.set()
                    os.close(write_fd)
                    signal.signal(signal.SIGINT, old_int)
                    signal.signal(signal.SIGTERM, old_term)
                for name in ('sensor_acquisition_mode', 'sensor_raw_duration_ms',
                             'sensor_poll_mode', 'sensor_poll_fixed_interval_ms'):
                    self.assertEqual(capture.read(feature / name), values[name])
                self.assertEqual(capture.read(raw / 'buffer/enable'), '0')
                self.assertEqual(capture.read(raw / 'buffer/length'), '16')
                self.assertTrue(all(capture.read(path) == '0' for path in (raw / 'scan_elements').glob('*_en')))
                status = json.loads((session / 'raw/status.json').read_text())
                self.assertEqual(status['state'], 'failed' if corrupt else 'completed')
                self.assertEqual(status['records'], 0 if corrupt else 1024)


if __name__ == '__main__':
    unittest.main()
