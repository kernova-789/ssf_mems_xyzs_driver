#!/usr/bin/env python3
"""Run two independent consumers and bounded real-data Replay import on the RK3588."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seconds',type=int,default=2700);a=p.parse_args()
base=Path(__file__).resolve().parent;sys.path.insert(0,str(base));from iio_devices import find_pair
out=a.output
source=subprocess.check_output(['findmnt','-rn','-T',str(out.parent),'-o','SOURCE'],text=True).strip()
assert source=='/dev/nvme0n1p1',('output must use the confirmed external data disk',source)
out.mkdir(parents=True,exist_ok=False)
exe=base/'replay-build/replay_offline_example'
pairs={s:find_pair(s) for s in ('serial0-0','serial1-0')}
children=[];logs=[];result={'seconds':a.seconds,'start_utc':time.time(),'passed':False,'instances':{}}
def snapshot():
    data={'utc':time.time(),'monotonic':time.monotonic(),'disk_free':os.statvfs(out).f_bavail*os.statvfs(out).f_frsize,'processes':{}}
    for name,proc in children:
        try:data['processes'][name]={'stat':Path(f'/proc/{proc.pid}/stat').read_text(),'status':Path(f'/proc/{proc.pid}/status').read_text()}
        except FileNotFoundError:pass
    for path in ['/proc/stat','/proc/meminfo','/proc/tty/driver/serial','/proc/diskstats']:
        try:data[path]=Path(path).read_text()
        except OSError as e:data[path]=str(e)
    for s,pair in pairs.items():
        raw=pair['ssf_mems_xyzs_raw'];data[s]={}
        for attr in ['raw_active','raw_crc_errors','raw_sequence_gaps','raw_buffer_drops','raw_drain_crc_errors','raw_drain_sequence_gaps','raw_batches','raw_batch_samples','raw_packets']:
            data[s][attr]=(raw/attr).read_text().strip()
    return data
try:
    subprocess.run(['dmesg'],stdout=(out/'kernel-before.log').open('w'),check=True)
    original={}
    for s,pair in pairs.items():
        assert all((d/'buffer/enable').read_text().strip()=='0' for d in pair.values())
        feature=pair['ssf_mems_xyzs'];original[s]={}
        for key in ['sensor_acquisition_mode','sensor_raw_duration_ms','sensor_feature_duration_ms','sensor_raw_batch_samples','sensor_raw_manual']:
            original[s][key]=(feature/key).read_text().strip()
        (feature/'sensor_acquisition_mode').write_text('feature')
        (feature/'sensor_raw_batch_samples').write_text('1024');(feature/'sensor_raw_manual').write_text('0')
        (feature/'sensor_raw_duration_ms').write_text('5000');(feature/'sensor_feature_duration_ms').write_text('5000')
    (out/'original-config.json').write_text(json.dumps(original,indent=2))
    for label,s in [('driver','serial0-0'),('replay-capture','serial1-0')]:
        dest=out/label;dest.mkdir();log=(dest/'console.log').open('w');logs.append(log)
        proc=subprocess.Popen([sys.executable,str(base/'board_capture.py'),str(dest),str(a.seconds),'--serdev',s],stdout=log,stderr=subprocess.STDOUT)
        children.append((label,proc))
    log=(out/'replay-import-console.log').open('w');logs.append(log)
    importer=subprocess.Popen([sys.executable,str(base/'replay_live_import.py'),str(out/'replay-capture'),str(out/'replay'),str(exe),'--capture-pid',str(children[1][1].pid)],stdout=log,stderr=subprocess.STDOUT)
    children.append(('importer',importer));time.sleep(1)
    for pair in pairs.values():(pair['ssf_mems_xyzs']/'sensor_acquisition_mode').write_text('alternating')
    deadline=time.monotonic()+a.seconds+90
    with (out/'monitor.jsonl').open('w') as monitor:
        while any(proc.poll() is None for _,proc in children):
            for name,proc in children:
                if proc.poll() not in (None,0):raise RuntimeError(f'{name} exited {proc.returncode}')
            monitor.write(json.dumps(snapshot())+'\n');monitor.flush()
            if time.monotonic()>deadline:raise TimeoutError('capture/import exceeded planned deadline')
            time.sleep(5)
    for label,proc in children:result[label+'_exit']=proc.returncode
    for label,s in [('driver','serial0-0'),('replay-capture','serial1-0')]:
        dest=out/label;row={}
        for name,command in [('summary',['check_capture.py',str(dest/'capture_summary.json'),'--seconds',str(a.seconds)]),('binary',['analyze_finite_iio.py',str(dest),'--require-whole-batches'])]:
            with (dest/(name+'-check.log')).open('w') as f:
                check=subprocess.run([sys.executable,str(base/command[0])]+command[1:],stdout=f,stderr=subprocess.STDOUT,timeout=120)
            row[name+'_exit']=check.returncode
        result['instances'][s]=row
    with (out/'replay-verify.log').open('w') as f:
        result['replay_verify_exit']=subprocess.run([sys.executable,str(base/'verify_replay.py'),str(out/'replay'),str(exe)],stdout=f,stderr=subprocess.STDOUT,timeout=900).returncode
    result['passed']=all(v==0 for row in result['instances'].values() for v in row.values()) and result['replay_verify_exit']==0
except BaseException as e:
    result['error']=repr(e)
finally:
    for name,proc in children:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=45)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
    for f in logs:f.close()
    subprocess.run(['dmesg'],stdout=(out/'kernel-after.log').open('w'))
    for s,pair in pairs.items():
        try:
            # Leave both sensors available for feature reads, and all owned buffers disabled.
            (pair['ssf_mems_xyzs']/'sensor_acquisition_mode').write_text('feature')
        except OSError as e:result.setdefault('cleanup_errors',[]).append(str(e))
    result['end_utc']=time.time();result['elapsed']=result['end_utc']-result['start_utc']
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
sys.exit(0 if result['passed'] else 1)
