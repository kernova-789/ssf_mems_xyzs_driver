#!/usr/bin/env python3
"""Check unsupported sample lengths are rejected without changing the prior setting."""
import errno,json
from pathlib import Path
from iio_devices import find_pair
rows=[]
for serdev in ['serial0-0','serial1-0']:
    d=find_pair(serdev)['ssf_mems_xyzs'];p=d/'sensor_raw_batch_samples';before=p.read_text().strip()
    for value in ['0','1','512','1025','65536','-1','abc']:
        try:p.write_text(value)
        except OSError as e:
            assert e.errno==errno.EINVAL,(serdev,value,e)
            assert p.read_text().strip()==before
            rows.append({'serdev':serdev,'input':value,'errno':e.errno,'unchanged':True})
        else:raise AssertionError((serdev,value,'unsupported length accepted'))
print(json.dumps({'passed':True,'checks':rows},indent=2))
