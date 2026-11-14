"""Read-only Rosmaster UART observation; never opens a port or sends a command.

Frame layout follows the existing Rosmaster V3.3.1 library and the old GUI's
serial_protocol.py. TX is observed after write() returns, not an MCU receipt ACK.
"""
import json
import struct
from collections import deque
from threading import Lock
from time import monotonic, time_ns
from uuid import uuid4

TX_NAMES={1:'自动上报',2:'蜂鸣器',3:'PWM舵机',4:'PWM全部',5:'RGB灯带',6:'RGB灯效',
    0x0f:'状态重置',0x10:'电机PWM',0x11:'车体状态',0x12:'运动控制',0x13:'电机PID',
    0x14:'偏航PID',0x15:'设置车型',0x20:'总线舵机',0x21:'舵机ID',0x22:'舵机扭矩',
    0x23:'机械臂',0x24:'臂校准',0x30:'AKM默认角',0x31:'AKM转向',0x50:'请求数据',0xa0:'重置Flash'}
RX_NAMES={0x0a:'运动数据',0x0b:'MPU原始IMU',0x0c:'IMU姿态',0x0d:'编码器',0x0e:'ICM原始IMU',
    0x0f:'状态重置',0x13:'电机PID回读',0x14:'偏航PID回读',0x20:'舵机位置',0x23:'机械臂角度',
    0x24:'臂校准结果',0x51:'版本'}


def decode_frame(direction, frame):
    """Decode only verified fields; unknown functions retain their original hex."""
    b=bytes(frame)
    if direction not in ('TX','RX') or len(b)<5 or b[:2]!=(b'\xff\xfc' if direction=='TX' else b'\xff\xfb') or b[2]+2!=len(b):
        raise ValueError('串口帧头或长度无效')
    valid=sum(b[2:-1])%256==b[-1];f=b[3];p=b[4:-1]
    result={'direction':direction,'frame':b.hex(' ').upper(),'bytes':len(b),'function':f,
        'function_name':(TX_NAMES if direction=='TX' else RX_NAMES).get(f,'未知功能'),
        'checksum_valid':valid,'summary':'原始数据：'+p.hex(' ').upper()}
    if not valid:
        result['summary']='校验和错误；不解释数据字段';return result
    try:
        if direction=='TX' and f==0x12:
            car=p[0];vx,vy,w=struct.unpack('<hhh',p[1:7])
            result['values']={'car_type':car,'vx_m_s':vx/1000,'vy_m_s':vy/1000,'wz_rad_s':w/1000}
            result['summary']=f'车型={car} vx={vx/1000:+.3f} m/s vy={vy/1000:+.3f} m/s wz={w/1000:+.3f} rad/s'
        elif direction=='RX' and f==0x0a:
            vx,vy,w=struct.unpack('<hhh',p[:6]);voltage=p[6]
            result['values']={'vx_m_s':vx/1000,'vy_m_s':vy/1000,'wz_rad_s':w/1000,'voltage_v':voltage}
            result['summary']=f'vx={vx/1000:+.3f} m/s vy={vy/1000:+.3f} m/s wz={w/1000:+.3f} rad/s 电压={voltage} V'
        elif direction=='RX' and f==0x0d:
            values=list(struct.unpack('<iiii',p[:16]));result['values']={'encoders':values}
            result['summary']=' '.join(f'M{i+1}={v}' for i,v in enumerate(values))
        elif direction=='TX' and f==2:
            value=struct.unpack('<h',p[:2])[0];result['summary']=f'蜂鸣参数={value}（1常响，0关闭，其余按厂商协议）'
        elif direction=='TX' and f==0x15:result['summary']=f'车型={p[0]}'
        elif direction=='TX' and f==0x50:result['summary']=f'请求功能=0x{p[0]:02X} 参数=0x{p[1]:02X}'
        elif direction=='RX' and f==0x51:result['summary']=f'版本 H={p[0]} L={p[1]}'
    except (ValueError,IndexError,struct.error):
        result['summary']='字段长度异常；保留原始帧';result['decode_error']=True
    return result


class FrameStream:
    def __init__(self,direction):
        self.direction=direction;self.header=b'\xff\xfc' if direction=='TX' else b'\xff\xfb'
        self.buffer=bytearray();self.discarded_bytes=0

    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while self.buffer:
            start=self.buffer.find(self.header)
            if start<0:
                keep=1 if self.buffer[-1]==0xff else 0
                self.discarded_bytes+=len(self.buffer)-keep
                self.buffer=self.buffer[-1:] if keep else bytearray();break
            if start:self.discarded_bytes+=start;del self.buffer[:start]
            if len(self.buffer)<3:break
            length=self.buffer[2]+2
            if length<5:
                self.discarded_bytes+=1;del self.buffer[0];continue
            if len(self.buffer)<length:break
            frame=bytes(self.buffer[:length]);frames.append(frame)
            if sum(frame[2:-1])%256==frame[-1]:del self.buffer[:length]
            else:
                # Keep a malformed frame visible, then resync without assuming
                # that its corrupted length accurately identifies the next frame.
                self.discarded_bytes+=1;del self.buffer[0]
        return frames


class SerialCapture:
    def __init__(self,capacity=1024):
        self.lock=Lock();self.frames=deque(maxlen=capacity);self.session=uuid4().hex
        self.streams={d:FrameStream(d) for d in ('TX','RX')};self.sequence=0;self.dropped=0
        self.counts={'TX':0,'RX':0};self.last={'TX':None,'RX':None};self.port='';self.error=''

    def feed(self,direction,data):
        if not data:return
        try:
            with self.lock:
                for frame in self.streams[direction].feed(data):
                    self.sequence+=1;self.counts[direction]+=1;self.last[direction]=monotonic()
                    if len(self.frames)==self.frames.maxlen:self.dropped+=1
                    self.frames.append({'session':self.session,'sequence':self.sequence,'unix_ns':time_ns(),
                        'direction':direction,'frame':frame.hex(' ').upper()})
        except Exception:
            # Observability must not alter a successful UART read/write.
            self.error='串口观察数据处理异常'

    def status(self):
        now=monotonic()
        with self.lock:
            return {'session':self.session,'unix_ns':time_ns(),'port':self.port,'transport':'serial','source':'yahboom_rosmaster_uart',
                'tx_frames':self.counts['TX'],'rx_frames':self.counts['RX'],'observer_dropped':self.dropped,
                'tx_age_seconds':None if self.last['TX'] is None else now-self.last['TX'],
                'rx_age_seconds':None if self.last['RX'] is None else now-self.last['RX'],
                'discarded_bytes':sum(s.discarded_bytes for s in self.streams.values()),'error':self.error}

    def take(self,limit=64):
        with self.lock:return [self.frames.popleft() for _ in range(min(limit,len(self.frames)))]


class SerialTap:
    def __init__(self,serial,capture):
        self._serial=serial;self._capture=capture;capture.port=str(getattr(serial,'port',''))

    def __getattr__(self,name):return getattr(self._serial,name)

    def read(self,*args,**kwargs):
        try:data=self._serial.read(*args,**kwargs)
        except Exception:
            self._capture.error='串口读取失败';raise
        self._capture.feed('RX',data)
        return data

    def write(self,data,*args,**kwargs):
        try:written=self._serial.write(data,*args,**kwargs)
        except Exception:
            self._capture.error='串口写入失败';raise
        # Observe exactly the bytes accepted by write(), not an inferred command.
        if isinstance(written,int) and written>0:self._capture.feed('TX',bytes(data)[:written])
        return written


def install_tap(library,capture):
    """Replace the library's module reference, leaving pyserial and files intact."""
    original=library.serial
    def factory(*args,**kwargs):return SerialTap(original.Serial(*args,**kwargs),capture)
    class SerialModule:
        Serial=staticmethod(factory)
        def __getattr__(self,name):return getattr(original,name)
    library.serial=SerialModule()
    return original


def attach_ros_observer(node,capture):
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    publisher=node.create_publisher(String,'/base/serial_frames',qos_profile_sensor_data)
    status=node.create_publisher(String,'/base/serial_status',qos_profile_sensor_data)
    last=[0.]
    def flush():
        frames=capture.take()
        if frames:publisher.publish(String(data=json.dumps({'schema_version':1,'frames':frames})))
        if monotonic()-last[0]>=.25:
            last[0]=monotonic();status.publish(String(data=json.dumps(capture.status())))
    return node.create_timer(.05,flush)


def main():
    # Instantiate the unchanged vendor node. One node, one port, one reader;
    # retain the original ROS remappings, buttons and MCU control implementation.
    import importlib
    import rclpy
    from Rosmaster_Lib import Rosmaster
    from yahboomcar_bringup.Mcnamu_driver_X3 import yahboomcar_driver
    capture=SerialCapture();library=importlib.import_module(Rosmaster.__module__)
    original=install_tap(library,capture);driver=None
    rclpy.init()
    try:
        driver=yahboomcar_driver('driver_node')
        attach_ros_observer(driver,capture)
        driver.get_logger().info('Jetson—MCU串口旁路观察已启用（原厂驱动，未增加串口读取者）')
        rclpy.spin(driver)
    finally:
        library.serial=original
        if driver is not None:driver.destroy_node()
        if rclpy.ok():rclpy.shutdown()
