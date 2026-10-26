#!/usr/bin/env python3
"""Board-only acceptance adapter: retain IIO bytes, feed bounded real-data TSV chunks to Replay."""
import argparse, hashlib, json, struct, subprocess, time
from pathlib import Path
p=argparse.ArgumentParser(); p.add_argument('capture',type=Path); p.add_argument('output',type=Path); p.add_argument('executable',type=Path); p.add_argument('--capture-pid',type=int,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
columns='id sensor_id timestamp_us data_type unit kind payload quality duration_us sample_rate_hz encoding source batch_id acquisition_plan input_origin'.split()
from iio_devices import find_pair
pair=find_pair('serial1-0')
clocks={(device/'current_timestamp_clock').read_text().strip('\x00\n ') for device in pair.values()}
assert len(clocks)==1,clocks
clock=clocks.pop()
assert clock in ('realtime','monotonic'),clock
offset=0 if clock=='realtime' else time.time_ns()-time.monotonic_ns()
meta={'version':1,'sensor_id':'rs485-2-xyzs','identity_scope':'physical sensor on UART7 during this session; not hardware serial number','source':'serial1-0','utc_minus_monotonic_ns':offset,'iio_timestamp_clock':clock,'timestamp_basis':'IIO host receive timestamp; realtime used directly, monotonic converted by fixed measured offset; not sensor sampling clock','raw_encoding':'IIO all channels little endian 24 bytes per scan; 1024 scans per block','feature_encoding':'IIO all channels little endian 72 bytes per feature; units preserved as counts','sample_rate_hz':5333.4,'quality_scope':'driver transport validation; no calibration or diagnosis','input_origin':'historical_real','session':str(a.capture)}
(a.output/'mapping.json').write_text(json.dumps(meta,indent=2))
pos={'raw':0,'features':0}; counts={'raw':0,'features':0}; chunk=0; total=0
ledger=(a.output/'expected.jsonl').open('w'); logs=(a.output/'imports.jsonl').open('w')
def alive():
    try:
        Path('/proc/'+str(a.capture_pid)+'/stat').read_text(); return True
    except FileNotFoundError:return False

def flush():
    global chunk,total
    rows=[]
    for kind,size,step in [('raw',24,1024),('features',72,1)]:
        f=a.capture/(kind+'.bin')
        if not f.exists():continue
        with f.open('rb') as src:
            src.seek(pos[kind]); data=src.read(1024*1024)
        n=len(data)//(size*step)*(size*step)
        for start in range(0,n,size*step):
            blob=data[start:start+size*step]; stamp=struct.unpack_from('<q',blob,16 if kind=='raw' else 64)[0]
            utc=(stamp+offset)//1000
            assert abs(utc-time.time_ns()//1000)<300_000_000,('IIO timestamp outside current session',utc)
            ident=f'{a.capture.name}-{kind}-{counts[kind]:09d}'
            duration=round(1024/5333.4*1e6) if kind=='raw' else 1
            row=[ident,'rs485-2-xyzs',str(utc),'vibration_iio_block' if kind=='raw' else 'feature_iio_scan','count','hex',blob.hex(),'0',str(duration),'5333.4' if kind=='raw' else '',f'iio-le-{size}-v1','serial1-0',ident,'board-20261026-mapping-v1','historical_real']
            rows.append(row); ledger.write(json.dumps({'id':ident,'kind':kind,'timestamp_us':utc,'duration_us':duration,'sha256':hashlib.sha256(blob).hexdigest(),'bytes':len(blob)})+'\n');counts[kind]+=1
        pos[kind]+=n
    if not rows:return
    file=a.output/f'input-{chunk:06d}.tsv'; file.write_text('edgesense-offline-tsv\t1\n'+'\t'.join(columns)+'\n'+''.join('\t'.join(r)+'\n' for r in rows))
    start=time.monotonic(); result=subprocess.run([str(a.executable),'import',str(file),str(a.output/'history')],text=True,capture_output=True,timeout=60)
    logs.write(json.dumps({'chunk':chunk,'seconds':time.monotonic()-start,'code':result.returncode,'stdout':result.stdout,'stderr':result.stderr})+'\n');logs.flush();ledger.flush()
    if result.returncode:raise RuntimeError(result.stderr)
    value=json.loads(result.stdout);assert value['inserted']==len(rows) and value['duplicates']==0,value
    total+=len(rows);chunk+=1
    (a.output/'status.json').write_text(json.dumps({'records':total,'counts':counts,'bytes_consumed':pos,'chunks':chunk,'utc':time.time()}))
while alive():
    flush();time.sleep(5)
flush();ledger.close();logs.close()
for kind,size in [('raw',24),('features',72)]:
    assert pos[kind]==(a.capture/(kind+'.bin')).stat().st_size,(kind,pos)
assert all(counts.values()),counts
(a.output/'import-complete.json').write_text(json.dumps({'records':total,'counts':counts,'chunks':chunk,'bytes_consumed':pos}))
