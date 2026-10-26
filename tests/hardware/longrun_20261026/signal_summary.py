#!/usr/bin/env python3
"""Describe preserved feature readings using the saved scan schema and scales."""
import collections,json,statistics
from pathlib import Path
from collect_training_data import Decoder
b=Path('/data/ssf-tests/20261026-dual-45min');schemas=json.loads((b/'evidence/iio-layouts-and-scales.json').read_text());out={}
for role,serdev in [('driver','serial0-0'),('replay-capture','serial1-0')]:
 schema=schemas[serdev]['feature'];decode=Decoder(schema);fields={r['name']:r for r in schema['fields']};data=(b/'longrun'/role/'features.bin').read_bytes();rows=[decode.decode(data[i:i+decode.size]) for i in range(0,len(data),decode.size)]
 result={'feature_rows':len(rows),'measurements':{}}
 for name,unit,factor in [('in_temp','degree Celsius',.001),('in_accel_x_rms','m/s^2',1),('in_accel_y_rms','m/s^2',1),('in_accel_z_rms','m/s^2',1),('in_count0_sound_rms_db','device-defined dB',1)]:
  scale=float(fields[name]['scale'])*factor;v=[r[name]*scale for r in rows]
  result['measurements'][name]={'unit':unit,'scale_applied':scale,'minimum':min(v),'maximum':max(v),'mean':statistics.mean(v)}
 result['startup_flags_counts']=dict(collections.Counter(r['in_count6_startup_flags'] for r in rows))
 summary=json.loads((b/'longrun'/role/'capture_summary.json').read_text());result['raw_adc_min']=summary['raw_min'];result['raw_adc_max']=summary['raw_max']
 out[serdev]=result
(b/'evidence/signal-summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
