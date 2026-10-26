#!/usr/bin/env python3
"""Independent ledger/bytes comparison of real-board Replay query and playback outputs."""
import argparse,hashlib,json,subprocess,sqlite3
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('executable',type=Path);a=p.parse_args()
expected={r['id']:r for r in map(json.loads,(a.output/'expected.jsonl').read_text().splitlines())}
assert expected
meta=json.loads((a.output/'mapping.json').read_text())
assert meta['iio_timestamp_clock'] in ('realtime','monotonic')
start=min(r['timestamp_us'] for r in expected.values());end=max(r['timestamp_us']+r['duration_us'] for r in expected.values())
results={}
for mode,terminal in [('query','query_end'),('replay','replay_end')]:
    target=a.output/(mode+'.ndjson')
    with target.open('w') as f:
        result=subprocess.run([str(a.executable),mode,str(a.output/'history'),str(start),str(end),'37'],stdout=f,stderr=subprocess.PIPE,text=True,timeout=300)
    assert result.returncode==0,result.stderr
    seen=set();last=None
    with target.open() as f:
        for line in f:
            r=json.loads(line);last=r
            if r['kind']!='record':continue
            assert r['id'] not in seen
            seen.add(r['id']);e=expected[r['id']]
            blob=bytes.fromhex(r['block_hex'])
            assert hashlib.sha256(blob).hexdigest()==e['sha256'] and len(blob)==e['bytes']
            assert r['timestamp_us']==e['timestamp_us'] and r['duration_us']==e['duration_us']
            assert r['sensor_id']=='rs485-2-xyzs' and r['source']=='serial1-0'
            raw=e['kind']=='raw'
            assert r['data_type']==('vibration_iio_block' if raw else 'feature_iio_scan')
            assert r['sample_rate_hz']==(5333.4 if raw else None)
            assert r['encoding']==('iio-le-24-v1' if raw else 'iio-le-72-v1')
            assert r['batch_id']==r['id'] and r['version']==1 and r['value'] is None
            assert r['data_sufficient'] is True
            assert r['metadata']['input_contract']=='edgesense-offline-tsv/1'
            assert r['unit']=='count' and r['quality']==0 and r['assessment']=='unassessed' and r['evidence_count']==0
            assert r['metadata']['input_origin']=='historical_real'
            assert r['metadata']['acquisition_plan']=='board-20261026-mapping-v1'
    assert last['kind']==terminal and last['records']==len(expected) and seen==expected.keys(),last
    results[mode]=last
file=sorted(a.output.glob('input-*.tsv'))[-1]
r=subprocess.run([str(a.executable),'import',str(file),str(a.output/'history')],capture_output=True,text=True,timeout=60)
assert r.returncode==0,r.stderr
v=json.loads(r.stdout);assert v['inserted']==0 and v['duplicates']==v['input_records'];results['duplicate_reimport']=v
with sqlite3.connect(str(a.output/'history/replay.db')) as db:
    results['integrity_check']=db.execute('PRAGMA integrity_check').fetchall();assert results['integrity_check']==[('ok',)]
results['expected_records']=len(expected);results['utc_range_us']=[start,end];results['passed']=True
(a.output/'verification.json').write_text(json.dumps(results,indent=2));print(json.dumps(results))
