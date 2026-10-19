#!/usr/bin/env python3
"""Run explicit XYZ screening points sequentially; never retry failed captures."""
import json
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
script = Path(__file__).with_name('board_xyz_rate_trial.py')
points = [(6, 460800, 6), (7, 512000, 6), (8, 600000, 6),
          (9, 750000, 7), (10, 921600, 7), (11, 1000000, 8)]
rows = []
for number, baud, rate in points:
    output = root / f'{number:02d}-{baud}-r{rate}'
    if output.exists():
        raise FileExistsError(output)
    print(f'START {output.name}', flush=True)
    with (root / (output.name + '-console.log')).open('w') as log:
        child = subprocess.run([sys.executable, str(script), str(output),
                                '--baud', str(baud), '--rate', str(rate), '--seconds', '30'],
                               stdout=log, stderr=subprocess.STDOUT)
    trial = json.loads((output / 'trial.json').read_text())
    row = dict(name=output.name, exit=child.returncode, passed=trial['passed'],
               configuration=trial['configuration'])
    rows.append(row)
    (root / 'matrix.json').write_text(json.dumps(rows, indent=2))
    print(json.dumps(row), flush=True)
