#!/usr/bin/env python3
"""Verify saved sound files, import actual IIO sound blocks, and compare real Replay output."""
import argparse,hashlib,json,sqlite3,struct,subprocess,wave
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);p.add_argument('executable',type=Path);a=p.parse_args();a.output.mkdir(exist_ok=False)
columns='id sensor_id timestamp_us data_type unit kind payload quality duration_us sample_rate_hz encoding source batch_id acquisition_plan input_origin'.split();expected={};imports=[];counts={}
for n,port in [(1,'serial0-0'),(2,'serial1-0')]:
 root=a.capture/f'rs485-{n}';summary=json.loads((root/'summary.json').read_text());assert summary['passed'] and summary['elapsed_seconds']>=300
 schema=json.loads((root/'schema.json').read_text());assert schema['scan_bytes']==16 and schema['timestamp_clock'].strip('\x00\n ')=='realtime'
 count=0
 for meta in summary['batches']:
  stem=f"batch-{meta['batch']:04d}";blob=(root/(stem+'.iio.bin')).read_bytes();pcm=(root/(stem+'.pcm-s32le.bin')).read_bytes();assert len(blob)==65536 and len(pcm)==16384
  assert hashlib.sha256(blob).hexdigest()==meta['scan_sha256'] and hashlib.sha256(pcm).hexdigest()==meta['pcm_sha256']
  rows=list(struct.iter_unpack('<iHHq',blob));last=0
  for i,(value,seq,index,stamp) in enumerate(rows):assert seq==i//64 and index==i%64 and stamp>0 and stamp>=last;last=stamp
  assert struct.pack('<4096i',*[r[0] for r in rows])==pcm
  with wave.open(str(root/(stem+'.wav')),'rb') as wav:
   assert (wav.getnchannels(),wav.getsampwidth(),wav.getframerate(),wav.getnframes())==(1,4,16000,4096)
   assert wav.readframes(4096)==pcm
  assert rows[0][3]==meta['first_iio_timestamp_ns'] and rows[-1][3]==meta['last_iio_timestamp_ns']
  assert all(int(meta['status'][k])==0 for k in ['crc_errors','sequence_gaps','ignored_bytes','buffer_drops','error'])
  ident=f'{a.capture.name}-{port}-{stem}';stamp=rows[0][3]//1000
  row=[ident,port+'-xyzs',str(stamp),'raw_sound_iio','count','hex',blob.hex(),'0','256000','16000','iio-le-16-v1',port,ident,'sound-five-minute-v1','historical_real']
  file=a.output/(port+'-'+stem+'.tsv');file.write_text('edgesense-offline-tsv\t1\n'+'\t'.join(columns)+'\n'+'\t'.join(row)+'\n')
  expected[ident]={'sensor_id':row[1],'timestamp_us':stamp,'sha256':meta['scan_sha256'],'bytes':65536,'source':port,'source_file':str(root/(stem+'.iio.bin'))}
  result=subprocess.run([str(a.executable),'import',str(file),str(a.output/'history')],text=True,capture_output=True,timeout=60);assert result.returncode==0,result.stderr
  ack=json.loads(result.stdout);assert ack['inserted']==1 and ack['duplicates']==0;imports.append(ack);count+=1
 counts[port]=count
(a.output/'expected.json').write_text(json.dumps(expected,indent=2));(a.output/'imports.json').write_text(json.dumps(imports,indent=2))
start=min(r['timestamp_us'] for r in expected.values());end=max(r['timestamp_us']+256000 for r in expected.values());results={}
for mode,terminal in [('query','query_end'),('replay','replay_end')]:
 path=a.output/(mode+'.ndjson')
 with path.open('w') as f:r=subprocess.run([str(a.executable),mode,str(a.output/'history'),str(start),str(end),'37'],stdout=f,stderr=subprocess.PIPE,text=True,timeout=300)
 assert r.returncode==0,r.stderr
 seen=set();last=None
 for line in path.open():
  v=json.loads(line);last=v
  if v['kind']!='record':continue
  assert v['id'] not in seen;seen.add(v['id']);e=expected[v['id']];payload=bytes.fromhex(v['block_hex'])
  assert len(payload)==e['bytes'] and hashlib.sha256(payload).hexdigest()==e['sha256']
  for k in ['sensor_id','timestamp_us','source']:assert v[k]==e[k]
  assert v['data_type']=='raw_sound_iio' and v['unit']=='count' and v['duration_us']==256000 and v['sample_rate_hz']==16000
  assert v['encoding']=='iio-le-16-v1' and v['batch_id']==v['id'] and v['quality']==0 and v['value'] is None and v['version']==1
  assert v['assessment']=='unassessed' and v['data_sufficient'] and v['evidence_count']==0
  assert v['metadata']=={'acquisition_plan':'sound-five-minute-v1','input_contract':'edgesense-offline-tsv/1','input_origin':'historical_real'}
 assert last['kind']==terminal and last['records']==len(expected) and seen==expected.keys(),last;results[mode]=last
last_input=sorted(a.output.glob('*.tsv'))[-1]
r=subprocess.run([str(a.executable),'import',str(last_input),str(a.output/'history')],capture_output=True,text=True,timeout=60);assert r.returncode==0,r.stderr
results['duplicate_reimport']=json.loads(r.stdout);assert results['duplicate_reimport']['inserted']==0 and results['duplicate_reimport']['duplicates']==1
with sqlite3.connect(str(a.output/'history/replay.db')) as db:
 results['integrity_check']=db.execute('PRAGMA integrity_check').fetchall();assert results['integrity_check']==[('ok',)]
results.update(passed=True,records=len(expected),batches_by_port=counts,utc_range_us=[start,end],raw_files_pcm_wav_verified=True)
(a.output/'verification.json').write_text(json.dumps(results,indent=2));print(json.dumps(results))
