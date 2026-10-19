"""按 serdev 实例选择一对 IIO 设备，避免同型号多设备被名称覆盖。"""
from pathlib import Path


def find_pair(serdev=None, root=Path('/sys/bus/iio/devices')):
    names = {'ssf_mems_xyzs', 'ssf_mems_xyzs_raw'}
    instances = {}
    for path in root.glob('iio:device*'):
        name = (path / 'name').read_text().strip()
        if name not in names:
            continue
        parent = path.resolve().parent.name
        pair = instances.setdefault(parent, {})
        if name in pair:
            raise RuntimeError(f'duplicate IIO role {name} under {parent}')
        pair[name] = path
    if serdev is None:
        if len(instances) != 1:
            raise RuntimeError(f'expected one sensor instance, found {sorted(instances)}; use --serdev')
        serdev = next(iter(instances))
    if serdev not in instances or set(instances[serdev]) != names:
        raise RuntimeError(f'incomplete or missing sensor instance {serdev}')
    return instances[serdev]
