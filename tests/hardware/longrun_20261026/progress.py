#!/usr/bin/env python3
"""Read-only compact status for the board acceptance run."""
import json,re,time
from pathlib import Path
base=Path('/data/ssf-tests/20261026-dual-45min');run=base/'longrun'
r={'local_time':time.strftime('%Y-%m-%d %H:%M:%S'),'files':{}}
for role in ['driver','replay-capture']:
 d=run/role
 if d.exists():
  r['files'][role]={p.name:p.stat().st_size for p in d.glob('*.bin')}
  log=d/'console.log'
  if log.exists():
   lines=log.read_text().splitlines();phases=[s for s in lines if s.startswith('phase ')]
   if phases:r[role]=json.loads(phases[-1][6:])
p=run/'replay/status.json'
if p.exists():r['replay']=json.loads(p.read_text())
p=run/'result.json'
if p.exists():r['result']=json.loads(p.read_text())
text=Path('/proc/tty/driver/serial').read_text()
compact={'time':r['local_time'],'captures':{}}
for role in ['driver','replay-capture']:
 if role in r:
  v=r[role];compact['captures'][role]={k:v[k] for k in ['elapsed','raw_records','feature_records','raw_crc_errors','raw_sequence_gaps','raw_buffer_drops']}
compact['replay']=r.get('replay',{})
compact['uart_errors']={line.split(':',1)[0]:dict(re.findall(r'(fe|brk|oe|pe):(\d+)',line)) for line in text.splitlines() if re.match(r'^\d+:',line)}
if 'result' in r:compact['result']=r['result']
print(json.dumps(compact,ensure_ascii=False))
