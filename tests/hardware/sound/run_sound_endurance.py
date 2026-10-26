#!/usr/bin/env python3
"""Five-minute dual-sensor sound gate; reject any new UART or kernel communication errors."""
import json,os,re,subprocess,sys,time
from pathlib import Path
base=Path(__file__).resolve().parent;out=Path(sys.argv[1]);seconds=int(sys.argv[2]) if len(sys.argv)>2 else 300
assert subprocess.check_output(['findmnt','-rn','-T',str(out.parent),'-o','SOURCE'],text=True).strip()=='/dev/nvme0n1p1'
out.mkdir(exist_ok=False)
children=[];logs=[];result={'passed':False,'planned_seconds':seconds,'started_utc':time.time()}
def uart():
 return Path('/proc/tty/driver/serial').read_text()
def parse(text):
 return {line.split(':')[0]:{k:int(v) for k,v in re.findall(r'(fe|brk|oe|pe):(\d+)',line)} for line in text.splitlines() if re.match(r'^\d+:',line)}
try:
 before=subprocess.check_output(['dmesg'],text=True);(out/'kernel-before.log').write_text(before)
 first=uart();(out/'uart-before.txt').write_text(first)
 for n,s in [(1,'serial0-0'),(2,'serial1-0')]:
  log=(out/f'rs485-{n}.log').open('w');logs.append(log)
  proc=subprocess.Popen([sys.executable,str(base/'board_sound_capture.py'),str(out/f'rs485-{n}'),'--serdev',s,'--seconds',str(seconds)],stdout=log,stderr=subprocess.STDOUT);children.append(proc)
 deadline=time.monotonic()+seconds+90
 with (out/'monitor.jsonl').open('w') as f:
  while any(p.poll() is None for p in children):
   if any(p.poll() not in (None,0) for p in children):raise RuntimeError('sound capture child failed')
   if time.monotonic()>deadline:raise TimeoutError('sound gate exceeded deadline')
   f.write(json.dumps({'utc':time.time(),'uart':uart(),'free_bytes':os.statvfs(out).f_bavail*os.statvfs(out).f_frsize})+'\n');f.flush();time.sleep(2)
 result['exit_codes']=[p.returncode for p in children];assert result['exit_codes']==[0,0]
 result['captures']={str(n):json.loads((out/f'rs485-{n}/summary.json').read_text()) for n in [1,2]}
 assert all(r['passed'] and r['elapsed_seconds']>=seconds for r in result['captures'].values())
 last=uart();(out/'uart-after.txt').write_text(last);a,b=parse(first),parse(last)
 result['uart_error_delta']={port:{k:b[port].get(k,0)-a[port].get(k,0) for k in ['fe','brk','oe','pe']} for port in a}
 after=subprocess.check_output(['dmesg'],text=True);(out/'kernel-after.log').write_text(after)
 lines=after.splitlines();assert before.splitlines()[-1] in lines
 delta=lines[lines.index(before.splitlines()[-1])+1:];(out/'kernel-delta.log').write_text('\n'.join(delta)+'\n')
 result['kernel_errors']=[v for v in delta if re.search(r'failed|unconfirmed|CRC/length|communication lost|overrun|I/O error|EXT4-fs error|nvme.*(error|timeout)',v,re.I)]
 result['passed']=not result['kernel_errors'] and all(v==0 for row in result['uart_error_delta'].values() for v in row.values())
except BaseException as e:result['error']=repr(e)
finally:
 for p in children:
  if p.poll() is None:p.terminate()
 for p in children:
  if p.poll() is None:
   try:p.wait(timeout=45)
   except subprocess.TimeoutExpired:p.kill();p.wait()
 for f in logs:f.close()
 result['ended_utc']=time.time();(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='captures'}),flush=True)
sys.exit(0 if result['passed'] else 1)
