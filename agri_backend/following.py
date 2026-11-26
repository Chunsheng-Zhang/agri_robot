"""Teach-and-repeat API: recording observes localization, never joystick commands.

Replay uses a separate ROS controller/lease. File writes and processing run on
one worker, outside ROS callbacks. A fault preserves raw samples without joining
the subsequent road section to the failed recording.
"""
import copy
import csv
import logging
import math
from threading import Event, RLock, Thread
from time import monotonic

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from .devices import device_cache, service_call
from .missions import ROOT, atomic_json
from .taught_paths import FollowingParameters, ProcessingParameters, TaughtPathStore
from .missions import controller_metadata

BUSY = ('ACQUIRING','RUNNING','PAUSED')


class RecordRequest(BaseModel):
    name: str = Field(min_length=1,max_length=80)
    processing: ProcessingParameters = Field(default_factory=ProcessingParameters)


class RenameRequest(BaseModel):
    name: str = Field(min_length=1,max_length=80)


class ImportRequest(RecordRequest):
    filename: str = Field(min_length=1,max_length=255)
    content: str = Field(min_length=1,max_length=2*1024*1024)


class FollowingManager:
    def __init__(self, root=ROOT):
        self.store = TaughtPathStore(root)
        self.lock, self.stop_event = RLock(), Event()
        self.node = self.publisher = self.thread = None
        self.document, self.samples, self.raw_index, self.raw_file = None, [], 0, None
        self.state, self.reason = 'IDLE',''
        self.loaded = None
        self.reference, self.track, self.error = [], [], None
        self.pose, self.pose_at = None, -math.inf
        self.valid, self.valid_at = False, -math.inf
        self.pending_finish = False
        self.saving = False
        self.started_at = -math.inf

    def install_ros(self, node):
        from agri_base.runtime import LATCHED, fresh_stamp, yaw_of
        from geometry_msgs.msg import Vector3Stamped
        from nav_msgs.msg import Odometry, Path
        from std_msgs.msg import Bool
        self.node = node
        self.store.recover_interrupted()
        self.publisher = node.create_publisher(Path,'/following/reference_path',LATCHED)
        def odom(msg):
            try:
                p = msg.pose.pose.position
                if (msg.header.frame_id!='odom' or msg.child_frame_id!='base_link'
                        or node.count_publishers('/odom')!=1
                        or not fresh_stamp(node,msg.header.stamp,.5)
                        or not all(math.isfinite(v) for v in (p.x,p.y))):
                    raise ValueError('定位帧、时间戳或发布源异常')
                sample = {'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                          'x':float(p.x),'y':float(p.y),'yaw':yaw_of(msg.pose.pose.orientation)}
                self.accept_pose(sample)
            except ValueError as exc:
                with self.lock:
                    self.pose = None
                    self.fault(str(exc))
        def valid(msg):
            with self.lock:
                self.valid,self.valid_at = msg.data,monotonic()
                if not msg.data:
                    self.fault('MGX23 定位无效，录制已终止')
        def track(msg):
            if msg.header.frame_id!='map':
                return
            with self.lock:
                self.track = [{'x':p.pose.position.x,'y':p.pose.position.y} for p in msg.poses]
        def error(msg):
            if msg.header.frame_id=='map' and fresh_stamp(node,msg.header.stamp,.5) and all(
                    math.isfinite(v) for v in (msg.vector.x,msg.vector.y,msg.vector.z)):
                with self.lock:
                    self.error = {'deviation':msg.vector.x,'heading':msg.vector.y,'progress':msg.vector.z,'at':monotonic()}
        node.create_subscription(Odometry,'/odom',odom,1)
        node.create_subscription(Bool,'/localization/valid',valid,1)
        node.create_subscription(Path,'/following/tracked_path',track,1)
        node.create_subscription(Vector3Stamped,'/following/error',error,1)
        # Clear an idle controller's old reference after bridge restart. An active
        # controller retains its immutable task; no start/resume is sent here.
        self.publish([])
        self.stop_event.clear()
        self.thread = Thread(target=self.worker,name='taught-path-writer',daemon=True)
        self.thread.start()

    def accept_pose(self, sample):
        with self.lock:
            self.pose,self.pose_at = sample,monotonic()
            if self.state!='RECORDING':
                return
            if self.samples:
                dt = sample['stamp']-self.samples[-1]['stamp']
                if dt==0:
                    return
                if dt<0 or dt>self.document['processing']['max_gap']+1e-6:
                    self.fault('定位时间回退或采样空档超限，录制已终止')
                    return
            if len(self.samples)>=200000 or monotonic()-self.started_at>3600:
                self.fault('录制达到 1 小时或 20 万样本上限，请分段示教')
                return
            self.samples.append(dict(sample))

    def fault(self, reason):
        if self.state=='RECORDING':
            self.state,self.reason,self.pending_finish = 'FAULT',reason,True
            logging.getLogger(__name__).warning('示教录制终止：%s',reason)

    def healthy_pose(self):
        return self.valid and self.pose is not None and monotonic()-self.pose_at<.5 and monotonic()-self.valid_at<.5

    def start_record(self, request, projection):
        with self.lock:
            if self.state in ('RECORDING','PROCESSING') or self.pending_finish or self.saving:
                raise ValueError('上一轮录制仍在记录或保存')
            safety = device_cache.snapshot()['safety']
            if (not safety['fresh'] or safety.get('mode')!='base' or safety.get('owner') not in ('','manual',None)
                    or safety.get('motion_inhibited') or not self.healthy_pose()
                    or not safety.get('joy_connected')):
                raise ValueError('请在基础待命模式连接手柄、确认有效定位并解除安全锁定后录制')
            self.document = self.store.create(request.name,projection,request.processing.model_dump())
            self.store.update(self.document, mode_revision=safety.get('mode_revision', 0))
            self.samples,self.raw_index = [],0
            self.state,self.reason,self.started_at = 'RECORDING','',monotonic()
            self.pending_finish = False
            self.accept_pose(self.pose)
            return self.document['id']

    def stop_record(self, cancel=False):
        with self.lock:
            if self.state!='RECORDING':
                raise ValueError('当前没有正在录制的示教路径')
            if not cancel and not self.recording_epoch_matches():
                self.fault('运行模式或安全策略已切换，录制已终止；请重新示教')
                raise ValueError(self.reason)
            self.state = 'CANCELED' if cancel else 'PROCESSING'
            self.reason = '用户取消录制' if cancel else ''
            self.pending_finish = True

    def recording_epoch_matches(self):
        safety = device_cache.snapshot()['safety']
        return (safety['fresh'] and safety.get('mode') == 'base'
                and safety.get('mode_revision', 0) == self.document.get('mode_revision', 0))

    def recording_status(self):
        with self.lock:
            return {'state':self.state, 'reason':self.reason,
                    'id':self.document['id'] if self.document else None,
                    'samples':len(self.samples)}

    def path_snapshot(self, reference=True):
        with self.lock:
            points = self.reference if reference else self.track
            return {'frame_id':'map', 'count':len(points), 'points':copy.deepcopy(points),
                    'fingerprint':self.loaded.get('reference_fingerprint') if reference and self.loaded else None}

    def path_counts(self):
        with self.lock:
            return len(self.reference),len(self.track)

    def worker(self):
        while not self.stop_event.wait(.1):
            try:
                self.work_once()
            except (OSError,ValueError,RuntimeError) as exc:
                logging.getLogger(__name__).exception('示教路径处理失败')
                with self.lock:
                    self.state,self.reason = 'FAULT',f'示教数据处理/保存失败：{exc}'
                    self.pending_finish = False
                    if self.raw_file:
                        self.raw_file.close()
                        self.raw_file = None
                    if self.document:
                        try:
                            self.store.update(self.document,state='FAULT',reason=self.reason)
                        except OSError:
                            pass

    def work_once(self):
        with self.lock:
            if self.document is None:
                return
            if self.state=='RECORDING':
                snapshot = device_cache.snapshot()
                safety = snapshot['safety']
                if (not safety['fresh'] or safety.get('mode')!='base'
                        or not self.recording_epoch_matches()
                        or safety.get('owner') not in ('','manual',None) or safety.get('motion_inhibited')
                        or not safety.get('joy_connected') or not self.healthy_pose()
                        or snapshot['origin'].get('projection')!=self.document['projection']):
                    self.fault('手柄、定位、安全状态、工作模式或固定原点发生异常，录制已终止')
            # Bounded memory, disk work off the ROS thread. Raw samples are flushed
            # every 100 ms, including failure/cancel; never overwritten on replay.
            if self.raw_index<len(self.samples):
                if self.raw_file is None:
                    self.raw_file = (self.store.directory(self.document['id'])/'raw.csv').open('x',newline='',encoding='utf-8')
                    self.raw_writer = csv.DictWriter(self.raw_file,fieldnames=['stamp','x','y','yaw'])
                    self.raw_writer.writeheader()
                self.raw_writer.writerows(self.samples[self.raw_index:])
                self.raw_file.flush()
                self.raw_index = len(self.samples)
            if not self.pending_finish:
                return
            document,samples,state,reason = self.document,list(self.samples),self.state,self.reason
            self.pending_finish = False
            self.saving = True
            if self.raw_file:
                self.raw_file.close()
                self.raw_file = None
        try:
            if state=='PROCESSING':
                self.store.finish(document,samples)
                with self.lock:
                    self.state,self.reason = 'READY',''
            else:
                self.store.update(document,state=state,reason=reason)
        finally:
            with self.lock:
                self.saving = False

    def close(self):
        with self.lock:
            self.fault('后端停止，录制中断；请重新示教')
        self.stop_event.set()
        if self.thread:
            self.thread.join(5)
        # Flush remaining recording samples on graceful shutdown.
        try:
            self.work_once()
        except (ValueError,OSError):
            logging.getLogger(__name__).exception('结束示教采样时保存失败')
        finally:
            with self.lock:
                if self.raw_file:
                    self.raw_file.close()
                    self.raw_file = None
                self.node = self.publisher = None

    def publish(self, points):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        if self.publisher is None:
            raise ValueError('ROS 桥接尚未连接')
        msg = Path()
        msg.header.frame_id,msg.header.stamp = 'map',self.node.get_clock().now().to_msg()
        for row in points:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x,p.pose.position.y = float(row['x']),float(row['y'])
            p.pose.orientation.z,p.pose.orientation.w = math.sin(row['yaw']/2),math.cos(row['yaw']/2)
            msg.poses.append(p)
        self.publisher.publish(msg)

    def snapshot(self):
        with self.lock:
            error = self.error
            status = device_cache.snapshot()['following']
            raw = self.samples[::max(1,len(self.samples)//3000)]
            return {'recording':{'state':self.state,'reason':self.reason,'id':self.document['id'] if self.document else None,
                    'samples':len(self.samples),'elapsed_seconds':max(0,monotonic()-self.started_at) if math.isfinite(self.started_at) else 0},
                    'loaded_id':self.loaded['id'] if self.loaded else None,
                    'raw':copy.deepcopy(raw),'reference':copy.deepcopy(self.reference),
                    'track':copy.deepcopy(self.track[::max(1,len(self.track)//5000)]),
                    'error':{**{k:v for k,v in error.items() if k!='at'},'lateral':status.get('lateral_error')}
                        if error and monotonic()-error['at']<.5 and status['fresh'] else None,
                    'pose':dict(self.pose) if self.healthy_pose() else None}


following_manager = FollowingManager()


def install_routes(app, require_bridge):
    router = APIRouter(prefix='/api/following')
    manager = following_manager
    def workspace():
        return app.state.workspace
    def check_editable():
        # A stale controller status cannot authorize replacing/deleting a loaded
        # reference. New recordings are also excluded from every mutation.
        live = device_cache.snapshot()['following']
        if manager.state in ('RECORDING','PROCESSING') or manager.pending_finish or manager.saving:
            raise HTTPException(409,'请先结束示教录制并等待处理完成')
        if not live['fresh'] or live.get('state') in BUSY:
            raise HTTPException(409,'跟随控制器状态未更新或正在执行任务')
    def document(identity):
        try:
            return manager.store.get(identity)
        except (ValueError,OSError,KeyError) as exc:
            raise HTTPException(404,str(exc)) from exc
    def safe(operation):
        try:
            return operation()
        except (ValueError,OSError) as exc:
            raise HTTPException(409,str(exc)) from exc

    @router.get('/context')
    def context():
        workspace().sync()
        snapshot = device_cache.snapshot()
        return {**manager.snapshot(),'paths':manager.store.list(),'status':snapshot['following'],
                'active_run':workspace().active if workspace().active and workspace().active.get('mode')=='following' else None}

    @router.get('/path/reference')
    def reference_path():
        return manager.path_snapshot()

    @router.get('/path/tracked')
    def tracked_path():
        return manager.path_snapshot(reference=False)

    @router.post('/record/start')
    def record_start(body:RecordRequest):
        with workspace().operation():
            require_bridge()
            check_editable()
            snapshot = device_cache.snapshot()
            for source in ('waypoint','tracking','planning'):
                if snapshot[source]['fresh'] and snapshot[source].get('state') in (*BUSY,'PLANNING'):
                    raise HTTPException(409,'请先结束其他导航或规划任务')
            origin = snapshot['origin'].get('projection')
            if not origin:
                raise HTTPException(409,'固定原点尚未由定位节点确认')
            identity = safe(lambda:manager.start_record(body,origin))
            workspace().event('TEACH_RECORD_STARTED',path_id=identity)
            return {'id':identity}

    @router.post('/record/{action}')
    def record_stop(action:str):
        if action not in ('stop','cancel'):
            raise HTTPException(404,'Unknown recording action')
        with workspace().operation():
            safe(lambda:manager.stop_record(cancel=action=='cancel'))
            workspace().event('TEACH_RECORD_'+action.upper())
            return {'success':True}

    @router.get('/paths/{identity}')
    def path_get(identity:str):
        return document(identity)

    @router.post('/import')
    def path_import(body:ImportRequest):
        from .path_import import import_path
        with workspace().operation():
            # Creating a new library item never replaces a loaded reference or
            # publishes ROS data. It can be done offline using configured origin.
            if (manager.state in ('RECORDING','PROCESSING') or manager.pending_finish or manager.saving
                    or device_cache.snapshot()['following'].get('state') in BUSY):
                raise HTTPException(409,'请先结束录制或跟随任务，再导入路径')
            value = safe(lambda:import_path(manager.store,body.name,workspace().origin()[0],
                                            body.filename,body.content,body.processing.model_dump()))
            workspace().event('FOLLOWING_PATH_IMPORTED',path_id=value['id'])
            return value

    @router.patch('/paths/{identity}')
    def rename(identity:str, body:RenameRequest):
        with workspace().operation():
            check_editable()
            value = safe(lambda:manager.store.get(identity,ready=False))
            if not body.name.strip():
                raise HTTPException(422,'路径名称不能为空')
            safe(lambda:manager.store.update(value,name=body.name.strip()))
            return {'success':True}

    @router.delete('/paths/{identity}')
    def delete(identity:str):
        from .missions import now
        with workspace().operation():
            check_editable()
            value = safe(lambda:manager.store.get(identity,ready=False))
            if manager.loaded and manager.loaded['id']==identity:
                safe(lambda:manager.publish([]))
                manager.loaded,manager.reference,manager.track = None,[],[]
            safe(lambda:manager.store.update(value,deleted_at=now()))
            workspace().event('TEACH_PATH_DELETED',path_id=identity)
            return {'success':True}

    @router.post('/paths/{identity}/load')
    def load(identity:str):
        with workspace().operation():
            check_editable()
            workspace().require_idle('following')
            value = document(identity)
            if value['projection']!=workspace().origin()[0]:
                raise HTTPException(409,'示教路径固定原点与当前系统不同，不能加载')
            gateway = workspace().gateway()
            gateway.parameters('path_follower',{'allow_reverse':value['processing']['allow_reverse']})
            safe(lambda:manager.publish(value['points']))
            with manager.lock:
                manager.loaded,manager.reference,manager.track,manager.error = value,value['points'],[],None
            workspace().event('TEACH_PATH_LOADED',path_id=identity)
            return {'success':True,'fingerprint':value['reference_fingerprint']}

    @router.post('/start')
    def start(body:FollowingParameters):
        with workspace().operation():
            check_editable()
            safety = workspace().require_idle('following')
            value = manager.loaded
            if value is None:
                raise HTTPException(409,'请先加载一条示教路径')
            value = document(value['id'])
            status = device_cache.snapshot()['following']
            if status.get('reference_fingerprint')!=value['reference_fingerprint'] or value['projection']!=workspace().origin()[0]:
                raise HTTPException(409,'参考路径或原点未确认，请重新加载并等待就绪')
            if safety.get('motion_inhibited') or not manager.healthy_pose():
                raise HTTPException(409,'请先确认安全状态和实时定位')
            if math.dist((manager.pose['x'],manager.pose['y']),(value['points'][0]['x'],value['points'][0]['y']))>body.start_tolerance:
                raise HTTPException(409,'请先用手柄移动至示教起点附近，再释放手柄控制权')
            limits = safety.get('limits',{})
            if body.speed>limits.get('linear',0) or body.max_angular>limits.get('angular',0) or body.max_path_error>limits.get('path_error',0):
                raise HTTPException(409,'跟随参数超过安全模块限值')
            mission = {'id':value['id'],'revision':value['revision'],'name':value['name'],'mode':'following',
                'parameters':body.model_dump(),'projection':value['projection'],'taught_path_id':value['id'],
                'preview':{'points':value['points']},'processing':value['processing']}
            run = workspace().store.create_run(mission,safety['mode_revision'],'execution')
            directory = workspace().store.run_path(run['id'])
            (directory/'paths').mkdir()
            atomic_json(directory/'paths/following_reference.json',value)
            gateway = workspace().gateway()
            capture = False
            try:
                gateway.parameters('path_follower',{**body.model_dump(),'allow_reverse':value['processing']['allow_reverse'],
                    'trajectory_directory':str(directory/'tracks')})
                run['controller_algorithm'] = body.controller_algorithm
                atomic_json(directory/'config/tracking_controller.json',controller_metadata(body.controller_algorithm))
                gateway.recorder.begin(directory,'following')
                capture = True
                requested = monotonic()
                service_call(require_bridge().node,'/following/start')
            except Exception as exc:
                rejected = isinstance(exc,HTTPException) and exc.status_code==409
                if capture and rejected:
                    gateway.recorder.abort(str(exc.detail))
                # A transport timeout may be accepted; retain capture and track
                # status by the new run directory, rather than guessing a stop.
                run.update(state='FAILED' if not capture or rejected else 'ACQUIRING',reason=str(getattr(exc,'detail',exc)),_requested=monotonic())
                workspace().active = run
                workspace().store.save_run(run)
                raise
            run.update(state='ACQUIRING',_requested=requested,taught_path_id=value['id'])
            workspace().active = run
            workspace().store.save_run(run)
            workspace().event('FOLLOWING_STARTED',run_id=run['id'],path_id=value['id'])
            return {'success':True,'run_id':run['id']}

    @router.post('/task/{action}')
    def control(action:str):
        if action not in ('pause','resume','cancel'):
            raise HTTPException(404,'Unknown task action')
        with workspace().operation():
            result = service_call(require_bridge().node,f'/following/{action}')
            workspace().event('FOLLOWING_'+action.upper())
            return result

    app.include_router(router)
