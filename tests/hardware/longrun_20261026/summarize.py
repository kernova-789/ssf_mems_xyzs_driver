#!/usr/bin/env python3
"""Summarize actual evidence; do not turn a script pass into a clean kernel-log claim."""
import argparse,collections,json,re,statistics
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('base',type=Path);a=p.parse_args();base=a.base;out={}
def uart(text):
 return {int(line.split(':',1)[0]):{k:int(v) for k,v in re.findall(r'(tx|rx|fe|brk|oe|pe):(\d+)',line)} for line in text.splitlines() if re.match(r'^\d+:',line)}
for name in ['smoke','smoke-corrected','longrun']:
 d=base/name
 if not (d/'result.json').exists():continue
 r=json.loads((d/'result.json').read_text());r['captures']={}
 for role in ['driver','replay-capture']:
  s=json.loads((d/role/'capture_summary.json').read_text());statuses=s['raw_status']
  r['captures'][role]={k:s[k] for k in ['elapsed','raw_records','feature_records','raw_sessions','sequence_breaks','bad_indices','timestamp_regressions']}
  r['captures'][role]['raw_nominal_signal_seconds']=s['raw_records']/5333.4
  r['captures'][role]['maximum_observed_errors']={k:max(v[k] for v in statuses) for k in ['raw_crc_errors','raw_sequence_gaps','raw_buffer_drops','raw_drain_crc_errors','raw_drain_sequence_gaps','raw_ignored_bytes']}
 monitor=list(map(json.loads,(d/'monitor.jsonl').read_text().splitlines()))
 before=uart(monitor[0]['/proc/tty/driver/serial']);after=uart(monitor[-1]['/proc/tty/driver/serial'])
 r['uart_delta']={str(port):{k:after[port].get(k,0)-before[port].get(k,0) for k in ['tx','rx','fe','brk','oe','pe']} for port in before}
 r['process_peak_rss_kib']={}
 for row in monitor:
  for proc,v in row['processes'].items():
   match=re.search(r'^VmRSS:\s+(\d+)',v['status'],re.M)
   if match:r['process_peak_rss_kib'][proc]=max(r['process_peak_rss_kib'].get(proc,0),int(match[1]))
 cpu=[]
 for x,y in zip(monitor,monitor[1:]):
  b=list(map(int,x['/proc/stat'].splitlines()[0].split()[1:9]));e=list(map(int,y['/proc/stat'].splitlines()[0].split()[1:9]));delta=[v-u for u,v in zip(b,e)];total=sum(delta)
  if total:cpu.append(100*(total-delta[3]-delta[4])/total)
 r['system_cpu_percent']={'average':statistics.mean(cpu),'maximum':max(cpu)} if cpu else {}
 r['minimum_disk_free_bytes']=min(v['disk_free'] for v in monitor)
 lines=(d/'kernel-after.log').read_text().splitlines();before_lines=(d/'kernel-before.log').read_text().splitlines();cut=before_lines[-1] if before_lines else None
 if cut in lines:lines=lines[lines.index(cut)+1:]
 else:r['kernel_delta_warning']='baseline line unavailable in ring buffer'
 errors=[v for v in lines if re.search(r'failed|unconfirmed|CRC/length|communication lost|overrun|I/O error|EXT4-fs error|nvme.*(error|timeout)',v,re.I)]
 r['kernel_new_errors']=errors
 r['data_checks_passed']=r['passed']
 r['communication_stability_passed']=not errors and all(v[k]==0 for v in r['uart_delta'].values() for k in ['fe','brk','oe','pe'])
 r['engineering_acceptance_passed']=r['data_checks_passed'] and r['communication_stability_passed']
 r['acceptance_scope']='data checks cover persisted input; stability additionally requires no new UART errors or kernel acquisition/recovery errors'
 (d/'kernel-delta.log').write_text('\n'.join(lines)+'\n')
 if (d/'replay/verification.json').exists():r['replay_verification']=json.loads((d/'replay/verification.json').read_text())
 imports=[json.loads(v)['seconds'] for v in (d/'replay/imports.jsonl').read_text().splitlines()]
 if imports:r['import_seconds']={'count':len(imports),'mean':statistics.mean(imports),'p95':sorted(imports)[min(len(imports)-1,int(.95*len(imports)))],'maximum':max(imports)}
 out[name]=r
out['length_sweeps']={}
for port in [1,2]:
 parent=base/f'length-sweep-rs485-{port}'
 for session in parent.iterdir():
  if not session.is_dir():continue
  summary=json.loads((session/'summary.json').read_text());lengths=collections.defaultdict(lambda:{'windows':0,'batches':0,'samples':0,'maximum_window_seconds':0,'errors':0})
  for path in (session/'raw').glob('window-*.json'):
   v=json.loads(path.read_text());length=v['samples'];r=lengths[length];h=v['health'];r['windows']+=1;r['batches']+=h['raw_batches'];r['samples']+=length*h['raw_batches'];r['maximum_window_seconds']=max(r['maximum_window_seconds'],(v['ended_monotonic_ns']-v['started_monotonic_ns'])/1e9)
   r['errors']+=sum(h[k] for k in ['raw_crc_errors','raw_sequence_gaps','raw_buffer_drops','raw_drain_crc_errors','raw_drain_sequence_gaps','raw_ignored_bytes'])
  out['length_sweeps'][str(port)]={'session':str(session),'summary':summary,'lengths':dict(lengths)}
(base/'evidence/summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
