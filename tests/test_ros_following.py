"""Opt-in localhost teach/replay integration; no hardware drivers or ports."""
import base64
import json
import math
import os
from threading import Thread
from time import monotonic, sleep
from types import SimpleNamespace

import pytest

pytestmark=pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS')!='1',reason='Opt-in isolated DDS test')


@pytest.mark.parametrize('path_source',['teaching','file'])
def test_teach_save_load_follow_pause_resume_estop_and_complete(tmp_path,monkeypatch,path_source):
    assert 200<=int(os.environ['ROS_DOMAIN_ID'])<=230 and os.environ['ROS_LOCALHOST_ONLY']=='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
    from sensor_msgs.msg import NavSatFix, Joy
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from pyproj.enums import TransformDirection
    from fastapi.testclient import TestClient
    from agri_base.localization import Localization
    from agri_base.chassis_adapter import ChassisAdapter
    from agri_safety.node import Safety
    from agri_path_tracking.node import PathTracker
    from agri_backend import main
    from agri_backend.devices import device_cache
    from agri_backend.following import following_manager as manager
    from agri_backend.missions import MissionStore
    from agri_backend.taught_paths import TaughtPathStore
    from agri_backend.ros_workspace import wait_result

    def until(predicate,timeout=8):
        deadline=monotonic()+timeout
        while monotonic()<deadline:
            if predicate():return
            sleep(.02)
        raise AssertionError(f'Following timeout: {device_cache.snapshot()}, state={s}')

    monkeypatch.setattr(device_cache,'values',{})
    workspace=main.app.state.workspace
    store=MissionStore(tmp_path)
    from pathlib import Path as FilePath
    store.config_path=FilePath(__file__).resolve().parents[1]/'config/jetson_system.yaml'
    monkeypatch.setattr(workspace,'store',store)
    monkeypatch.setattr(workspace,'active',None)
    monkeypatch.setattr(manager,'store',TaughtPathStore(tmp_path))
    monkeypatch.setattr(manager,'document',None)
    monkeypatch.setattr(manager,'loaded',None)
    monkeypatch.setattr(manager,'state','IDLE')
    monkeypatch.setattr(manager,'samples',[])
    rclpy.init(args=['--ros-args','-p','require_lidar:=false','-p','require_joy:=false'])
    nodes=[];executor=SingleThreadedExecutor();thread=None
    s={'x':0.,'y':0.,'yaw':0.,'v':0.,'w':0.,'manual_v':0.,'manual_w':0.,'sensors':True,'raw':0}
    try:
        localization,safety,chassis=Localization(),Safety(),ChassisAdapter()
        tracker,follower,bench=PathTracker(),PathTracker(following=True),Node('following_testbench')
        backend=main.RobotStatusNode()
        nodes=[localization,safety,chassis,tracker,follower,bench,backend]
        monkeypatch.setattr(main.app.state,'ros_bridge',SimpleNamespace(started=True,node=backend),raising=False)
        for node in nodes:executor.add_node(node)
        fix=bench.create_publisher(NavSatFix,'/mgx23/position',1)
        attitude=bench.create_publisher(Vector3Stamped,'/mgx23/attitude_stamped',1)
        velocity=bench.create_publisher(Vector3Stamped,'/mgx23/velocity_stamped',1)
        joy=bench.create_publisher(Joy,'/joy',1)
        manual=bench.create_publisher(String,'/manual/status',1)
        command=bench.create_publisher(TwistStamped,'/control/manual/cmd_vel',1)
        feedback=bench.create_publisher(Twist,'/vel_raw',1)
        raw=bench.create_publisher(String,'/mgx23/raw',100)
        def output(msg):s['v'],s['w']=msg.linear.x,msg.angular.z
        bench.create_subscription(Twist,'/base/driver_cmd_vel',output,1)
        def sensors():
            dt=.05
            s['x']+=s['v']*math.cos(s['yaw'])*dt
            s['y']+=s['v']*math.sin(s['yaw'])*dt
            s['yaw']=math.atan2(math.sin(s['yaw']+s['w']*dt),math.cos(s['yaw']+s['w']*dt))
            stamp=bench.get_clock().now().to_msg()
            j=Joy();j.header.stamp=stamp;j.axes=[0.,0.,0.];j.buttons=[0]*15;joy.publish(j)
            manual.publish(String(data=json.dumps({'stamp':stamp.sec+stamp.nanosec*1e-9,'connected':True})))
            c=TwistStamped();c.header.stamp=stamp;c.header.frame_id='base_link'
            c.twist.linear.x,c.twist.angular.z=s['manual_v'],s['manual_w'];command.publish(c)
            feedback.publish(Twist(linear=c.twist.linear,angular=c.twist.angular))
            if not s['sensors']:return
            frame=localization.frame
            lon,lat=frame.transformer.transform(frame.e0+s['x'],frame.n0+s['y'],direction=TransformDirection.INVERSE)
            f=NavSatFix();f.header.stamp=stamp;f.latitude,f.longitude=lat,lon
            a=Vector3Stamped();a.header=f.header
            a.vector.z=math.atan2(math.sin(frame.yaw(lat,lon,0)-s['yaw']),math.cos(frame.yaw(lat,lon,0)-s['yaw']))
            attitude.publish(a);fix.publish(f)
            v=Vector3Stamped();v.header=f.header;v.vector.x=s['v']*math.sin(s['yaw']);v.vector.y=s['v']*math.cos(s['yaw']);velocity.publish(v)
            s['raw']+=1
            raw.publish(String(data=json.dumps({'schema_version':1,'session':'following-synthetic','sequence':s['raw'],
                'stamp_ns':bench.get_clock().now().nanoseconds,'unix_ns':0,'transport':'synthetic',
                'data_base64':base64.b64encode(b'synthetic-teach-test').decode()})))
        bench.create_timer(.05,sensors)
        thread=Thread(target=executor.spin,daemon=True);thread.start()
        client=TestClient(main.app)
        def post(path,body=None):
            response=client.post(path,json=body) if body is not None else client.post(path)
            assert response.status_code==200,response.text
            return response.json()
        def manual_enable(enabled):
            c=backend.create_client(SetBool,'/safety/manual/enable')
            until(c.service_is_ready)
            result=wait_result(c.call_async(SetBool.Request(data=enabled)))
            backend.destroy_client(c)
            assert result.success,result.message
        until(lambda:safety.localization_ok() and safety.joystick_ok() and manager.healthy_pose())
        post('/api/robot/emergency-stop/reset')
        manual_enable(True)
        identity=post('/api/following/record/start',{'name':'隔离道路示教'})['id']
        s['manual_v'],s['manual_w']=.4,.12
        until(lambda:s['x']>.95)
        s['manual_v'],s['manual_w']=0.,0.
        until(lambda:abs(s['v'])<1e-8)
        post('/api/following/record/stop')
        until(lambda:manager.state=='READY')
        manual_enable(False)
        taught=manager.store.get(identity)
        assert taught['statistics']['length_m']>.9
        assert manager.store.directory(identity).joinpath('raw.csv').exists()
        if path_source=='file':
            original=manager.store.directory(identity).joinpath('path.json').read_bytes()
            lines=['longitude,latitude,yaw']
            frame=localization.frame
            for p in taught['points']:
                lon,lat=frame.transformer.transform(frame.e0+p['x'],frame.n0+p['y'],direction=TransformDirection.INVERSE)
                lines.append(f'{lon},{lat},{p["yaw"]}')
            taught=post('/api/following/import',{'name':'文件坐标道路','filename':'road.csv','content':'\n'.join(lines)})
            assert manager.store.directory(identity).joinpath('path.json').read_bytes()==original
            identity=taught['id']
            assert manager.loaded is None and s['v']==0 and not safety.gate.owner
            assert taught['source_kind']=='import' and not taught['import_source']['timing_available']
        post('/api/system/mode',{'mode':'following'})
        until(lambda:follower.mode_matches)
        post(f'/api/following/paths/{identity}/load')
        until(lambda:device_cache.snapshot()['following'].get('reference_fingerprint')==taught['reference_fingerprint'])
        assert tracker.pending_path is None  # Coverage reference was never changed.
        assert client.post('/api/following/start',json={}).status_code==409  # Too far from start.
        start=taught['points'][0]
        s['x'],s['y'],s['yaw']=start['x'],start['y'],start['yaw']
        until(lambda:manager.pose and math.dist((manager.pose['x'],manager.pose['y']),(start['x'],start['y']))<.01)
        settings={'speed':.3,'lookahead':.2,'completion_tolerance':.05,'max_angular':1.}
        first=post('/api/following/start',settings)['run_id']
        until(lambda:follower.state=='RUNNING' and s['v']>.01)
        post('/api/following/task/pause');until(lambda:s['v']==0)
        assert follower.state=='PAUSED'
        post('/api/following/task/resume');until(lambda:s['v']>.01)
        post('/api/robot/emergency-stop')
        until(lambda:follower.state=='FAULT' and s['v']==0)
        post('/api/robot/emergency-stop/reset')
        sleep(.15)
        assert s['v']==0 and not safety.gate.owner
        first_dir=workspace.store.run_path(first)
        until(lambda:not backend.workspace_ros.recorder.busy(first_dir),15)
        s['x'],s['y'],s['yaw']=start['x'],start['y'],start['yaw']
        until(lambda:manager.pose and math.dist((manager.pose['x'],manager.pose['y']),(start['x'],start['y']))<.01)
        second=post('/api/following/start',settings)['run_id']
        until(lambda:follower.state=='COMPLETED',18)
        until(lambda:s['v']==0 and not safety.gate.owner)
        client.get('/api/following/context')
        directory=workspace.store.run_path(second)
        until(lambda:not backend.workspace_ros.recorder.busy(directory),15)
        metrics=json.loads((directory/'analysis/metrics.json').read_text())
        assert metrics['mode']=='following' and metrics['phases']['tracking']['samples']>0
        assert (directory/'analysis/heading_error.png').exists()
        assert (directory/'analysis/report.md').exists()
        assert (directory/'raw/mgx23.bin').stat().st_size>0
        assert '/control/following/cmd_vel' not in metrics.get('missing_topics',[])
        assert len(list((directory/'tracks').glob('track_*.csv')))==1
        assert not list((directory/'tracks').glob('approach_*.csv'))
        assert manager.store.get(identity)['fingerprint']==taught['fingerprint']
    finally:
        executor.shutdown(timeout_sec=3)
        if thread:thread.join(3)
        for node in reversed(nodes):node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
