"""Bounded, read-only live runtime logs and observed Jetson/MCU UART frames."""
import json
import logging
import math
import re
from collections import deque
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Event, Lock, Thread
from time import monotonic, time_ns
from uuid import uuid4
from zoneinfo import ZoneInfo

from fastapi import APIRouter, Query
from agri_base.serial_observer import decode_frame

ROOT=Path(__file__).resolve().parents[1]
LEVELS={10:'DEBUG',20:'INFO',30:'WARN',40:'ERROR',50:'FATAL'}
ANSI=re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
SECRET=re.compile(r'''(?i)(["']?(?:password|passwd|(?:access_|refresh_)?token|authorization|api[_-]?key|secret)["']?\s*[=:]\s*)(?:"[^"]*"|'[^']*'|(?:Bearer\s+)?[^\s,;]+)''')


def clean(value,limit=4000):
    return SECRET.sub(r'\1[REDACTED]',ANSI.sub('',str(value)))[:limit]


class DiagnosticArchive(RotatingFileHandler):
    def handleError(self,record):
        # Never print archive write failures into a service log that this same
        # collector follows: that would produce a recursive error stream.
        raise OSError('诊断归档写入失败，实时内存缓冲仍可查询')


class RuntimeDiagnostics:
    def __init__(self,root=ROOT,capacity=2000):
        self.root=Path(root);self.lock=Lock();self.session=uuid4().hex
        self.started_ns=time_ns();self.logs=deque(maxlen=capacity);self.serial=deque(maxlen=capacity)
        self.seq={'logs':0,'serial':0};self.last_status={};self.status_time=-math.inf
        self.states={};self.serial_identity=None;self.last_sequence=0;self.observed_gaps=0
        self.last_frame={'TX':None,'RX':None};self.bad_frames=0;self.malformed_batches=0
        self.queue=Queue(maxsize=4096);self.stop_event=Event();self.thread=None;self.handler=None
        self.archive_dropped=0;self.archive_error='';self.archive_dir=None;self.tails={}

    def _append(self,kind,value):
        with self.lock:
            self.seq[kind]+=1;item={**value,'id':self.seq[kind]}
            (self.logs if kind=='logs' else self.serial).append(item)
        if self.thread is not None:
            try:self.queue.put_nowait((kind,item))
            except Full:self.archive_dropped+=1
        return item

    def log(self,level,source,message,**details):
        return self._append('logs',{'unix_ns':time_ns(),'level':level,'source':clean(source,120),
            'message':clean(message),**details})

    def ros_log(self,msg):
        level=LEVELS.get(int(msg.level),'INFO')
        self.log(level,msg.name,msg.msg,channel='rosout',
            ros_stamp_ns=msg.stamp.sec*10**9+msg.stamp.nanosec)

    def state(self,source,text):
        try:
            data=json.loads(text)
            if not isinstance(data,dict):return
            key=tuple(str(data.get(k,'')) for k in ('state','phase','reason'))
            if self.states.get(source)==key:return
            self.states[source]=key
            state,phase,reason=key
            if not state and not reason:return
            level='ERROR' if state in ('FAULT','FAILED') else 'WARN' if state=='CANCELED' else 'INFO'
            self.log(level,source,f'{state} {phase} {reason}'.strip(),channel='task_status')
        except (ValueError,TypeError):pass

    def safety_event(self,text):
        try:
            data=json.loads(text)
            if isinstance(data,dict):self.log('WARN','safety',f'{data.get("event", "事件")} {data.get("reason", "")}',channel='safety_event')
        except (ValueError,TypeError):pass

    def serial_status(self,text):
        try:
            value=json.loads(text)
            if not isinstance(value,dict) or value.get('source')!='yahboom_rosmaster_uart':return
            session=value.get('session')
            if not isinstance(session,str) or not re.fullmatch('[0-9a-f]{32}',session):return
            if type(value.get('unix_ns')) is not int or value['unix_ns']<=0:return
            if not isinstance(value.get('port',''),str) or not isinstance(value.get('error',''),str):return
            for name in ('tx_frames','rx_frames','observer_dropped','discarded_bytes'):
                if type(value.get(name)) is not int or value[name]<0:return
            for name in ('tx_age_seconds','rx_age_seconds'):
                if value.get(name) is not None and (type(value[name]) not in (int,float) or not math.isfinite(value[name]) or value[name]<0):return
            delay=(time_ns()-value['unix_ns'])/1e9
            keys=('session','port','source','tx_frames','rx_frames','observer_dropped','discarded_bytes',
                  'tx_age_seconds','rx_age_seconds','error')
            with self.lock:
                self.last_status={k:value[k] for k in keys if k in value}
                self.status_time=monotonic()-max(0.,delay) if delay>=-1. else -math.inf
        except (ValueError,TypeError):pass

    def serial_batch(self,text):
        try:
            if len(text)>100000:raise ValueError()
            data=json.loads(text);frames=data.get('frames')
            if data.get('schema_version')!=1 or not isinstance(frames,list) or len(frames)>64:raise ValueError()
            for frame in frames:
                session=frame['session'];sequence=frame['sequence'];stamp=frame['unix_ns'];raw=frame['frame']
                if (not isinstance(session,str) or not re.fullmatch('[0-9a-f]{32}',session)
                        or type(sequence) is not int or sequence<=0 or type(stamp) is not int
                        or stamp<=0 or not isinstance(raw,str) or len(raw)>800):raise ValueError()
                decoded=decode_frame(frame['direction'],bytes.fromhex(raw))
                with self.lock:
                    if session!=self.serial_identity:
                        self.serial_identity=session;self.last_sequence=0;self.last_frame={'TX':None,'RX':None}
                    if sequence<=self.last_sequence:continue
                    if self.last_sequence:self.observed_gaps+=max(0,sequence-self.last_sequence-1)
                    self.last_sequence=sequence
                    delay=(time_ns()-stamp)/1e9
                    if decoded['checksum_valid']:
                        self.last_frame[decoded['direction']]=monotonic()-max(0.,delay) if delay>=-1. else None
                    if not decoded['checksum_valid']:self.bad_frames+=1
                self._append('serial',{**decoded,'unix_ns':stamp,'driver_session':session,'sequence':sequence,
                    'source':'yahboom_rosmaster_uart'})
        except (ValueError,TypeError,KeyError,AttributeError):self.malformed_batches+=1

    def snapshot(self,kind,after=0,session=None,limit=200):
        now=monotonic()
        with self.lock:
            values=list(self.logs if kind=='logs' else self.serial);last=self.seq[kind]
            reset=bool(session and session!=self.session)
            if reset:after=0
            if after==0 or after>last:
                selected=values[-limit:];skipped=0
            else:
                pending=[v for v in values if v['id']>after]
                selected=pending[-limit:]
                skipped=max(0,(selected[0]['id']-after-1) if selected else 0)
            result={'session':self.session,'started_unix_ns':self.started_ns,'entries':selected,
                'cursor':last,'reset':reset,'skipped':skipped,'capacity':self.logs.maxlen,
                'archive_directory':str(self.archive_dir) if self.archive_dir else None,
                'archive_dropped':self.archive_dropped,'archive_error':self.archive_error}
            if kind=='serial':
                age=now-self.status_time;status=dict(self.last_status)
                tx=now-self.last_frame['TX'] if self.last_frame['TX'] is not None else None
                rx=now-self.last_frame['RX'] if self.last_frame['RX'] is not None else None
                result['status']={**status,'observer_online':age<1.,'status_age_seconds':age if math.isfinite(age) else None,
                    'observed_tx_age_seconds':tx,'observed_rx_age_seconds':rx,
                    'rx_fresh':age<1. and status.get('session')==self.serial_identity and rx is not None and rx<1.,
                    'sequence_gaps':self.observed_gaps,'bad_frames':self.bad_frames,'malformed_batches':self.malformed_batches}
            return result

    def start(self):
        if self.thread is not None:return
        self.archive_dir=self.root/'.runtime/diagnostics'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        self.stop_event.clear();self.thread=Thread(target=self._record,name='runtime-diagnostics',daemon=True)
        self.thread.start();self.handler=DiagnosticHandler(self)
        logging.getLogger().addHandler(self.handler)
        self.log('INFO','backend','运行诊断会话开始；日志与串口观察不参与运动控制')

    def close(self):
        if self.handler is not None:logging.getLogger().removeHandler(self.handler);self.handler=None
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=3)
        self.thread=None

    def _record(self):
        writers={}
        try:
            self.archive_dir.mkdir(parents=True,exist_ok=True)
            for kind in ('logs','serial'):
                writers[kind]=DiagnosticArchive(self.archive_dir/(kind+'.jsonl'),maxBytes=5*1024*1024,
                    backupCount=2,encoding='utf-8',delay=True)
                writers[kind].setFormatter(logging.Formatter('%(message)s'))
            next_tail=0.
            while not self.stop_event.is_set() or not self.queue.empty():
                if not self.stop_event.is_set() and monotonic()>=next_tail:
                    self._console_logs();next_tail=monotonic()+.5
                try:kind,item=self.queue.get(timeout=.1)
                except Empty:continue
                record=logging.LogRecord('diagnostic_archive',logging.INFO,'',0,json.dumps(item,ensure_ascii=False),(),None)
                writers[kind].emit(record)
        except Exception as exc:self.archive_error=clean(exc,240)
        finally:
            for writer in writers.values():writer.close()

    def _console_logs(self):
        # Only the current project manager's fixed service logs; no caller paths.
        try:
            state_path=self.root/'.runtime/project/state.json'
            if state_path.stat().st_size>32768:return
            state=json.loads(state_path.read_text());folder=Path(state.get('logs','')).resolve()
            active=False
            for entry in state.get('services',{}).values():
                try:
                    proc=Path('/proc')/str(int(entry['pid']))
                    fields=(proc/'stat').read_text().split(') ',1)[1].split()
                    if fields[0]!='Z' and fields[19]==entry.get('start_ticks') and entry.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip():active=True
                except (OSError,ValueError,KeyError,TypeError):continue
            if not active:return
            if not folder.is_relative_to((self.root/'.runtime/project/runs').resolve()):return
            for name in ('core','backend','frontend'):
                path=folder/(name+'.log')
                if path.is_symlink() or not path.exists():continue
                st=path.stat();key=str(path);old=self.tails.get(key)
                offset,rest,inode=old if old else (max(0,st.st_size-65536),b'',st.st_ino)
                if inode!=st.st_ino or st.st_size<offset:offset,rest=0,b''
                with path.open('rb') as stream:
                    stream.seek(offset);chunk=stream.read(65536);new_offset=stream.tell()
                if not old and offset:
                    chunk=chunk.partition(b'\n')[2]  # Initial bounded tail starts at a whole line.
                data=rest+chunk;lines=data.split(b'\n');rest=lines.pop()[-8192:]
                self.tails[key]=(new_offset,rest,st.st_ino)
                for raw in lines:
                    line=raw.decode('utf-8',errors='replace')
                    if not line.strip() or '/api/diagnostics/' in line or ' HTTP/1.1" 200' in line:continue
                    level='ERROR' if any(v in line for v in ('[ERROR]','[FATAL]','Traceback','Error:','ERROR:')) else 'WARN' if any(v in line for v in ('[WARN]','WARNING:')) else 'INFO'
                    self.log(level,name+'.stdout',line,channel='service_console')
        except (OSError,ValueError,TypeError,AttributeError,IndexError):pass


class DiagnosticHandler(logging.Handler):
    def __init__(self,diagnostics):super().__init__(logging.INFO);self.diagnostics=diagnostics
    def emit(self,record):
        if record.name in ('uvicorn.access','diagnostic_archive'):return
        try:
            message=self.format(record)
            self.diagnostics.log('ERROR' if record.levelno>=40 else 'WARN' if record.levelno>=30 else 'INFO',
                record.name,message,channel='backend')
        except Exception:pass


runtime_diagnostics=RuntimeDiagnostics()


def install_ros(node):
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String
    node.create_subscription(Log,'/rosout',runtime_diagnostics.ros_log,qos_profile_sensor_data)
    node.create_subscription(String,'/base/serial_frames',lambda m:runtime_diagnostics.serial_batch(m.data),qos_profile_sensor_data)
    node.create_subscription(String,'/base/serial_status',lambda m:runtime_diagnostics.serial_status(m.data),qos_profile_sensor_data)
    for source in ('planning','waypoint','tracking','following'):
        node.create_subscription(String,'/'+source+'/status',lambda m,s=source:runtime_diagnostics.state(s,m.data),qos_profile_sensor_data)
    node.create_subscription(String,'/safety/events',lambda m:runtime_diagnostics.safety_event(m.data),qos_profile_sensor_data)


def install_routes(app):
    router=APIRouter()
    @router.get('/api/diagnostics/{kind}')
    def read(kind:str,after:int=Query(0,ge=0),session:str|None=Query(None,max_length=64),limit:int=Query(200,ge=1,le=300)):
        from fastapi import HTTPException
        if kind not in ('logs','serial'):raise HTTPException(404,'未知诊断模块')
        return runtime_diagnostics.snapshot(kind,after,session,limit)
    app.include_router(router)
