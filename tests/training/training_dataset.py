#!/usr/bin/env python3
"""已完成训练会话的只读入口；生成 CSV 目录表，不接触 IIO 或修改原始数据。

python3 tests/training/training_dataset.py /data/mems/<session> --catalog /tmp/catalog.csv
也可在 Python 中导入 TrainingDataset，逐批读取 XYZ 或逐条读取特征。
"""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path

from collect_training_data import Decoder, checked_payload, index_entries, verify


class TrainingDataset:
    """构造时执行整轮验收；迭代时再校验载荷，内存仅保留一个分片。"""
    def __init__(self, session):
        self.root = Path(session)
        self.counts = verify(self.root)
        self.metadata = json.loads((self.root / 'session.json').read_text())
        self.session_id = self.metadata['session_id']
        self.arguments = self.metadata['arguments']

    def _identity(self, role):
        device = self.metadata['devices'][role]
        return dict(session_id=self.session_id, role=role,
                    machine_id=self.arguments['machine_id'], condition=self.arguments['condition'],
                    sensor_id=device.get('sensor_id', 'unlabeled'), mount=device.get('mount', 'unlabeled'),
                    serdev=device.get('serdev', 'unknown'), notes=self.arguments.get('notes', ''))

    def _entries(self, role):
        return index_entries(self.root / role / 'index.jsonl')

    def raw_batches(self, physical=False):
        """返回独立批次；xyz_counts 为 N×3，physical=True 时另返回 xyz_m_s2。

        sample_time_seconds 是批内相对时间，不连接跨批空档，也不用于跨传感器对时。
        """
        schema = self.metadata['devices']['raw']['schema']
        decoder = Decoder(schema)
        axes = ['in_accel_x', 'in_accel_y', 'in_accel_z']
        scales = {}
        if physical:
            for axis in axes:
                field = next(field for field in schema['fields'] if field['name'] == axis)
                value = float(field['scale'])
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('原始加速度比例无效')
                scales[axis] = value
        for entry in self._entries('raw'):
            payload = checked_payload(self.root, 'raw', entry)
            rows = [decoder.decode(payload[offset:offset + decoder.size])
                    for offset in range(0, len(payload), decoder.size)]
            fs = float(entry['sampling_frequency_hz'])
            result = dict(self._identity('raw'), unit_id=f"{self.session_id}:raw:{Path(entry['payload']).stem}",
                          metadata=entry, xyz_counts=[[row[axis] for axis in axes] for row in rows],
                          sample_time_seconds=[i / fs for i in range(len(rows))],
                          host_iio_timestamp_ns=[row['in_timestamp'] for row in rows])
            if physical:
                result['xyz_m_s2'] = [[row[axis] * scales[axis] for axis in axes] for row in rows]
            yield result

    def feature_rows(self, physical=False):
        """原始整数和可选物理值分别返回；温度物理值单位是毫摄氏度。"""
        fields = self.metadata['devices']['feature']['schema']['fields']
        for entry in self._entries('feature'):
            payload = checked_payload(self.root, 'feature', entry)
            for index, row in enumerate(csv.DictReader(io.StringIO(payload.decode('utf-8')))):
                values = {field['name']: int(row[field['name']]) for field in fields}
                result = dict(self._identity('feature'),
                    unit_id=f"{self.session_id}:feature:{Path(entry['payload']).stem}:{index}",
                    received_realtime_ns=int(row['received_realtime_ns']),
                    same_payload_as_previous=bool(int(row['same_payload_as_previous'])), raw=values)
                if physical:
                    converted, units = {}, {}
                    for field in fields:
                        if 'scale' in field:
                            scale = float(field['scale'])
                            if not math.isfinite(scale) or scale <= 0:
                                raise ValueError('特征比例无效')
                            converted[field['name']] = values[field['name']] * scale
                            units[field['name']] = field['unit_after_scale']
                    result.update(physical=converted, physical_units=units)
                yield result

    def catalog(self, destination):
        """每个文件一行的派生目录表，含标签、来源、长度、时间范围及校验值。"""
        columns = ['unit_id', 'session_id', 'role', 'machine_id', 'condition', 'sensor_id',
                   'mount', 'serdev', 'notes', 'payload', 'records', 'samples', 'window_id',
                   'sampling_frequency_hz', 'first_iio_timestamp_ns', 'last_iio_timestamp_ns',
                   'bytes', 'sha256']
        destination = Path(destination)
        # 生成物须在会话外，以保持输入会话只读；拒绝覆盖任何已有文件。
        if self.root.resolve() == destination.resolve() or self.root.resolve() in destination.resolve().parents:
            raise ValueError('目录表请输出到会话目录之外')
        with destination.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for role in ('raw', 'feature'):
                for entry in self._entries(role):
                    row = self._identity(role)
                    row.update({key: entry[key] for key in columns if key in entry})
                    row['unit_id'] = f"{self.session_id}:{role}:{Path(entry['payload']).stem}"
                    row['records'] = entry['samples'] if role == 'raw' else entry['records']
                    writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--catalog', type=Path, help='输出会话之外的新 CSV 目录表，不覆盖文件')
    args = parser.parse_args()
    dataset = TrainingDataset(args.session)
    if args.catalog:
        dataset.catalog(args.catalog)
    print(json.dumps({'session_id': dataset.session_id, 'verified_records': dataset.counts}, ensure_ascii=False))


if __name__ == '__main__':
    main()
