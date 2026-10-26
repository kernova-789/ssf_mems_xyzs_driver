#!/usr/bin/env python3
"""Capture validated 4096-point XYZS sound batches using public IIO interfaces."""
import argparse,concurrent.futures,hashlib,json,math,os,select,struct,sys,time,wave
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from collect_training_data import Decoder,layout,snapshot_buffer,write,read,atomic_json
p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);p.add_argument('--serdev',required=True);p.add_argument('--captures',type=int,default=3);p.add_argument('--seconds',type=float,default=0,help='Repeat whole batches for at least this many seconds; overrides captures');a=p.parse_args()
if a.captures<=0 or not math.isfinite(a.seconds) or a.seconds<0:p.error('captures must be positive and seconds finite and nonnegative')
found={}
for d in Path('/sys/bus/iio/devices').glob('iio:device*'):
 if d.resolve().parent.name==a.serdev:found[read(d/'name')]=d
feature=found['ssf_mems_xyzs'];sound=found['ssf_mems_xyzs_sound']
if not (feature/'sensor_sound_capture').exists():raise RuntimeError('sound trigger not available')
assert all(read(d/'buffer/enable')=='0' for d in found.values())
a.output.mkdir(parents=True,exist_ok=False)
saved=snapshot_buffer(sound);mode=read(feature/'sensor_acquisition_mode');fd=None
result={'passed':False,'serdev':a.serdev,'batches':[],'saved_mode':mode,'saved_buffer':saved}
try:
 write(feature/'sensor_acquisition_mode','feature')
 schema=layout(sound);decode=Decoder(schema);frequency=int(read(sound/'sampling_frequency'))
 assert schema['scan_bytes']==16 and frequency==16000
 atomic_json(a.output/'schema.json',schema)
 for f in (sound/'scan_elements').glob('*_en'):write(f,1)
 write(sound/'buffer/length',8192)
 fd=os.open('/dev/'+sound.name,os.O_RDONLY|os.O_NONBLOCK)
 write(sound/'buffer/enable',1)
 poll=select.poll();poll.register(fd,select.POLLIN)
 with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
  run_started=time.monotonic()
  for batch in range(100000000 if a.seconds else a.captures):
   if batch and a.seconds and time.monotonic()-run_started>=a.seconds:break
   started=time.monotonic();begin=time.time_ns();future=pool.submit(write,feature/'sensor_sound_capture',1)
   data=bytearray();deadline=started+45
   while True:
    if time.monotonic()>deadline:raise TimeoutError('sound command did not finish within 45 seconds')
    events=poll.poll(100)
    for _,event in events:
     if event&(select.POLLERR|select.POLLHUP|select.POLLNVAL):raise RuntimeError('IIO read event error')
     try:part=os.read(fd,decode.size*8192)
     except BlockingIOError:continue
     if not part:raise RuntimeError('IIO EOF')
     data.extend(part)
    if future.done() and not events:break
   future.result()
   assert len(data)==4096*decode.size,len(data)
   pcm=[];timestamps=[]
   for i in range(4096):
    row=decode.decode(data[i*decode.size:(i+1)*decode.size])
    assert row['in_count1_packet_sequence']==i//64 and row['in_count2_sample_index']==i%64
    assert row['in_timestamp']>0 and (not timestamps or row['in_timestamp']>=timestamps[-1])
    pcm.append(row['in_count0_sound']);timestamps.append(row['in_timestamp'])
   status=dict(part.split('=') for part in read(sound/'sound_capture_status').split())
   assert int(status['packets'])==64 and int(status['error'])==0,status
   assert all(int(status[k])==0 for k in ['sequence_gaps','crc_errors','ignored_bytes','buffer_drops']),status
   name=f'batch-{batch+1:04d}';scan=a.output/(name+'.iio.bin');scan.write_bytes(data)
   blob=struct.pack('<4096i',*pcm);(a.output/(name+'.pcm-s32le.bin')).write_bytes(blob)
   with wave.open(str(a.output/(name+'.wav')),'wb') as wav:
    wav.setnchannels(1);wav.setsampwidth(4);wav.setframerate(frequency);wav.writeframes(blob)
   row={'batch':batch+1,'samples':4096,'packets':64,'sampling_frequency':frequency,'nominal_signal_seconds':4096/frequency,'seconds':time.monotonic()-started,'started_utc_ns':begin,'first_iio_timestamp_ns':timestamps[0],'last_iio_timestamp_ns':timestamps[-1],'pcm_min':min(pcm),'pcm_max':max(pcm),'unique_values':len(set(pcm)),'scan_sha256':hashlib.sha256(data).hexdigest(),'pcm_sha256':hashlib.sha256(blob).hexdigest(),'status':status}
   atomic_json(a.output/(name+'.json'),row);result['batches'].append(row)
   atomic_json(a.output/'status.json',{'batches':batch+1,'samples':(batch+1)*4096,'elapsed_seconds':time.monotonic()-run_started,'last_batch':row})
   print(json.dumps(row),flush=True)
 result['elapsed_seconds']=time.monotonic()-run_started
 result['planned_seconds']=a.seconds
 assert not a.seconds or result['elapsed_seconds']>=a.seconds
 result['passed']=True
except BaseException as e:
 result['error']=repr(e)
 raise
finally:
 if fd is not None:
  write(sound/'buffer/enable',0);os.close(fd)
 for name,value in saved['scan'].items():write(sound/'scan_elements'/name,value)
 write(sound/'buffer/length',saved['length'])
 if 'watermark' in saved:write(sound/'buffer/watermark',saved['watermark'])
 write(feature/'sensor_acquisition_mode',mode)
 result['finished_utc_ns']=time.time_ns();atomic_json(a.output/'summary.json',result)
