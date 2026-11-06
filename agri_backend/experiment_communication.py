"""Passive task communication measurements; no ROS publishers or network probes."""
import csv
import json
import math
import struct
from pathlib import Path
from time import monotonic_ns, time_ns

from .missions import atomic_json


class CommunicationMiddleware:
    """Bind observations to the session present at request START; never log bodies/queries."""
    PATHS = frozenset(('/api/robot/status', '/api/devices', '/api/workspace', '/api/camera/stream-info'))

    def __init__(self, app, session_getter):
        self.app, self.session_getter = app, session_getter

    async def __call__(self, scope, receive, send):
        if scope['type'] != 'http' or scope.get('method') != 'GET' or scope.get('path') not in self.PATHS:
            return await self.app(scope, receive, send)
        try:
            session = self.session_getter()
        except Exception:
            session = None
        if session is None:
            return await self.app(scope, receive, send)
        started, unix = monotonic_ns(), time_ns()
        status, complete = 0, False

        async def observed_send(message):
            nonlocal status, complete
            await send(message)
            if message['type'] == 'http.response.start':
                status = message['status']
            elif message['type'] == 'http.response.body' and not message.get('more_body', False):
                complete = True
        try:
            return await self.app(scope, receive, observed_send)
        finally:
            # A diagnostic failure must never alter API/control behavior.
            try:
                session.put('http', scope['path'], [unix, started, 'GET', scope['path'], status,
                    (monotonic_ns()-started)/1e6,
                    'ok' if complete and 200 <= status < 400 else 'http_error' if complete else 'incomplete'])
            except Exception:
                pass


def read_csv(path):
    if not path.exists():
        return []
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def numeric(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def distribution(values, threshold=None):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    def percentile(p):
        index = (len(values)-1)*p
        lo = int(index)
        return values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*(index-lo)
    result = {'samples': len(values), 'mean': sum(values)/len(values) if values else None,
              **{f'p{int(p*100)}': percentile(p) if values else None for p in (.5,.95,.99)},
              'max': max(values) if values else None}
    if threshold is not None:
        count = sum(v > threshold for v in values)
        result.update(threshold=threshold, over_threshold=count,
                      over_threshold_percent=100*count/len(values) if values else None)
    return result


def raw_sequence(rows):
    sessions = {}
    for row in rows:
        sessions.setdefault(row['session'], []).append(int(row['sequence']))
    results = {}
    for name, numbers in sessions.items():
        unique = set(numbers)
        expected = max(unique)-min(unique)+1
        missing = expected-len(unique)
        results[name] = dict(received=len(numbers), unique=len(unique), expected_between_first_last=expected,
                             missing=missing, missing_percent=100*missing/expected, duplicates=len(numbers)-len(unique))
    return results


def navi_continuity(directory, chunks):
    """Decode only NAVI frames. Other packet types/no-return bytes are not checksum failures."""
    frames, bad, repeated, invalid_time, estimated_missing, long_intervals = 0, 0, 0, 0, 0, 0
    intervals, buffers, previous, last_chunk = [], {}, {}, None
    raw_path=directory/'raw/mgx23.bin'
    if not raw_path.exists():
        return {'available':False,'reason':'没有原始字节文件'}
    with raw_path.open('rb') as binary, (directory/'communication/navi_frames.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['session','source_ros_ns','device_hhmmss','device_seconds_of_day','checksum_ok','accuracy_m','interval_s'])
        for chunk in chunks:
            session,sequence=chunk['session'],int(chunk['sequence'])
            buffer=buffers.setdefault(session,bytearray())
            if last_chunk is not None and (last_chunk[0]!=session or last_chunk[1]+1!=sequence):
                buffer.clear()  # Never splice a partial frame across an observed delivery gap.
            last_chunk=session,sequence
            binary.seek(int(chunk['offset']));buffer.extend(binary.read(int(chunk['size'])))
            while True:
                start=buffer.find(b'fmin')
                if start<0:
                    del buffer[:-3]
                    break
                del buffer[:start]
                if len(buffer)<100:break
                if buffer[98:100]!=b'ed':
                    del buffer[0]
                    continue
                frame=bytes(buffer[:100]);del buffer[:100]
                frames+=1
                checksum=(sum(frame[:96])&0xffff)==struct.unpack_from('<H',frame,96)[0]
                hhmmss=struct.unpack_from('<d',frame,4)[0]
                accuracy=struct.unpack_from('<f',frame,60)[0]
                seconds=None
                if math.isfinite(hhmmss) and hhmmss>=0:
                    hour,minute,second=int(hhmmss//10000),int(hhmmss//100)%100,hhmmss%100
                    if hour<24 and minute<60 and second<60:seconds=hour*3600+minute*60+second
                delta=None
                if not checksum:bad+=1
                elif seconds is None:invalid_time+=1
                else:
                    if session in previous:
                        delta=(seconds-previous[session]+43200)%86400-43200
                        if delta>0:
                            intervals.append(delta*1000)
                            # Estimate source cadence only, not IP loss; large clock jumps are unclassified.
                            if delta<10:estimated_missing+=max(0,round(delta/.1)-1)
                            else:long_intervals+=1
                        elif delta==0:repeated+=1
                        else:invalid_time+=1
                    previous[session]=seconds
                writer.writerow([session,chunk['source_ros_ns'],hhmmss if math.isfinite(hhmmss) else '',
                                 seconds,checksum,accuracy if math.isfinite(accuracy) else '',delta])
    return {'available':bool(frames),'reason':None if frames else '没有完整 NAVI 帧，不能判断源样本缺失',
            'frames':frames,'checksum_failures':bad,'repeated_device_times':repeated,
            'invalid_or_backward_device_times':invalid_time,'device_interval_ms':distribution(intervals),
            'estimated_missing_at_10hz':estimated_missing if intervals and not long_intervals and not invalid_time else None,
            'unclassified_long_intervals':long_intervals,'assumed_period_ms':100,
            'meaning':'接收到的 NAVI 设备时间按10Hz估算的缺口；可能来自设备停发、链路或记录缺失，不能定位原因或作为IP丢包率'}


def analyze_communication(directory, capture):
    directory = Path(directory)
    folder = directory/'communication'
    # Only new runs opt in by creating metadata at capture start. Never invent legacy observations.
    if not (folder/'metadata.json').exists():
        return {'available': False, 'reason': '本轮未启用通信采集，未测'}
    timing = read_csv(folder/'ros_timing.csv')
    http = read_csv(folder/'http_server.csv')
    chunks = read_csv(directory/'raw/chunks.csv')
    metadata = json.loads((folder/'metadata.json').read_text())
    topics = {}
    for topic in sorted({r['topic'] for r in timing}):
        rows = [r for r in timing if r['topic'] == topic]
        stamps = [numeric(r['received_monotonic_ns']) for r in rows]
        stamps = [s for s in stamps if s is not None]
        intervals = [(b-a)/1e6 for a,b in zip(stamps,stamps[1:]) if b > a]
        ages = [numeric(r['source_age_ms']) for r in rows]
        topics[topic] = {'received': len(rows),
            'observed_hz': (len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None,
            'interarrival_ms': distribution(intervals),
            'source_to_recorder_ms': distribution([a for a in ages if a is not None and a >= 0], metadata['latency_reference_ms']),
            'unstamped_samples': sum(a is None for a in ages), 'negative_age_samples': sum(a is not None and a < 0 for a in ages),
            'non_increasing_receipts': sum(b<=a for a,b in zip(stamps,stamps[1:]))}
    # Compare exact shared Header stamps, with 1s margins inside the position observation window.
    position = [r for r in timing if r['topic']=='/mgx23/position' and numeric(r['source_stamp_ns']) is not None]
    valid_receipts = [numeric(r['received_monotonic_ns']) for r in position]
    valid_receipts = [r for r in valid_receipts if r is not None]
    references = set()
    if valid_receipts:
        lo,hi=min(valid_receipts)+1e9,max(valid_receipts)-1e9
        references={r['source_stamp_ns'] for r in position if numeric(r['received_monotonic_ns']) is not None and lo<=float(r['received_monotonic_ns'])<=hi}
    matches={}
    for topic in ('/mgx23/attitude_stamped','/mgx23/velocity_stamped','/odom'):
        stamps={r['source_stamp_ns'] for r in timing if r['topic']==topic}
        missing=len(references-stamps)
        matches[topic]={'position_reference_samples':len(references),'unmatched':missing,
                       'unmatched_percent':100*missing/len(references) if references else None}
    endpoints={}
    for path in sorted({r['path'] for r in http}):
        rows=[r for r in http if r['path']==path]
        errors=sum(r['outcome']!='ok' for r in rows)
        endpoints[path]={'requests':len(rows),'failed_or_incomplete':errors,'failure_percent':100*errors/len(rows),
                         'server_duration_ms':distribution([numeric(r['duration_ms']) for r in rows], metadata['http_reference_ms'])}
    summary={'schema_version':1,'available':True,'topics':topics,'matched_source_stamps':matches,
             'raw_sequence_by_session':raw_sequence(chunks), 'http_server':endpoints,
             'navi':navi_continuity(directory,chunks),
             'recorder_queue_drops_by_topic':capture.get('queue_drops_by_topic',{}),'recorder_errors':capture.get('errors',[]),
             'unmeasured':{'ip_packet_loss':'未测：未在网络两端抓包，原始块序号不代表 IP 包序号',
                           'browser_round_trip':'未测：后端请求耗时不含浏览器、代理和网络完整往返',
                           'physical_actuation_latency':'未测：无消息标识/独立时钟，未进行专门响应实验'},
             'notes':metadata['notes']+['100ms is a diagnostic reference, not a safety policy or acceptance guarantee.',
                  'Queue drops and raw sequence gaps may describe the same missing data; do not add their rates.',
                  'Absent idle command traffic is not 100% loss; no exact delivery loss rate without matching sender/receiver identities.',
                  'LaserScan Header is normally the first ray; scan acquisition time is included in its age.']}
    atomic_json(folder/'summary.json',summary)
    with (folder/'summary.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['kind','name','samples','mean_ms','p95_ms','p99_ms','max_ms','over_threshold_percent'])
        for kind,entries,field in (('ros',topics,'source_to_recorder_ms'),('http_server',endpoints,'server_duration_ms')):
            for name,values in entries.items():
                item=values[field]
                writer.writerow([kind,name]+[item[k] for k in ('samples','mean','p95','p99','max','over_threshold_percent')])
    return summary
