#!/usr/bin/env python3
"""End-to-end synthetic PX4-on-PTY regression: no physical actuators involved."""
import os
import pty
import tty
import select
import struct
import subprocess
import threading
import time
import json
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from std_msgs.msg import String
from pixhawk_interfaces.srv import ParamGet, ParamSet, GotoLocal, GotoGlobal, SetMode, MissionDownload, MissionUpload, VelocityVector
from std_srvs.srv import Trigger

def encode_int(i):
    return struct.unpack('<f',struct.pack('<I',i))[0]

class Writer:
    def __init__(self,fd): self.fd=fd
    def write(self,data): return os.write(self.fd,data)

class FC:
    def __init__(self,fd):
        self.fd=fd
        self.mav=mavutil.mavlink.MAVLink(Writer(fd),srcSystem=1,srcComponent=1)
        self.reader=mavutil.mavlink.MAVLink(None)
        self.reader.robust_parsing=True
        self.stop=False
        self.local_sp=0
        self.global_sp=0
        self.set_mode_count=0
        self.missions=[{'x':-62200000,'y':1068000000,'z':100.0},
                       {'x':-62200100,'y':1068000200,'z':101.0}]
        self.pending_upload=None
        self.params={'SYS_AUTOSTART':(4001,mavutil.mavlink.MAV_PARAM_TYPE_INT32),
                     'TEST_GAIN':(1.0,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)}
        self.thread=threading.Thread(target=self.loop,daemon=True)
    def start(self): self.thread.start()
    def param(self,name):
        if name not in self.params:return
        v,tp=self.params[name]
        encoded=encode_int(int(v)) if tp!=mavutil.mavlink.MAV_PARAM_TYPE_REAL32 else float(v)
        self.mav.param_value_send(name.encode(),encoded,tp,len(self.params),list(self.params).index(name))
    def incoming(self,msg):
        t=msg.get_type()
        if t=='PARAM_REQUEST_READ':
            self.param(msg.param_id.rstrip('\x00'))
        elif t=='PARAM_REQUEST_LIST':
            for name in self.params:self.param(name)
        elif t=='PARAM_SET':
            name=msg.param_id.rstrip('\x00')
            if name in self.params and name!='SYS_AUTOSTART':
                self.params[name]=(msg.param_value,msg.param_type)
            self.param(name)
        elif t=='MISSION_REQUEST_LIST':
            n=len(self.missions) if msg.mission_type==0 else 0
            self.mav.mission_count_send(245,191,n,msg.mission_type)
        elif t=='MISSION_REQUEST_INT':
            if msg.mission_type==0 and msg.seq<len(self.missions):
                i=msg.seq;v=self.missions[i]
                self.mav.mission_item_int_send(245,191,i,6,16,1 if i==0 else 0,1,
                    0,0,0,0,v['x'],v['y'],v['z'],0)
        elif t=='MISSION_COUNT':
            if msg.mission_type==0:
                self.pending_upload={'count':msg.count,'items':[]}
                self.mav.mission_request_int_send(245,191,0,0)
        elif t=='MISSION_ITEM_INT':
            if self.pending_upload is not None:
                i=msg.seq
                if i==len(self.pending_upload['items']):
                    self.pending_upload['items'].append({'x':msg.x,'y':msg.y,'z':msg.z})
                if len(self.pending_upload['items'])==self.pending_upload['count']:
                    self.missions=self.pending_upload['items']
                    self.pending_upload=None
                    self.mav.mission_ack_send(245,191,mavutil.mavlink.MAV_MISSION_ACCEPTED,0)
                else:
                    self.mav.mission_request_int_send(245,191,len(self.pending_upload['items']),0)
        elif t=='COMMAND_LONG':
            if msg.command==176:self.set_mode_count+=1
            self.mav.command_ack_send(msg.command,mavutil.mavlink.MAV_RESULT_ACCEPTED)
        elif t=='SET_POSITION_TARGET_LOCAL_NED':
            self.local_sp+=1
        elif t=='SET_POSITION_TARGET_GLOBAL_INT':
            self.global_sp+=1
    def loop(self):
        os.set_blocking(self.fd,False)
        last_hb=last_sensors=last_gps=0.
        while not self.stop:
            now=time.monotonic()
            try:
                if now-last_hb>=0.5:
                    self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                      mavutil.mavlink.MAV_AUTOPILOT_PX4,0,0,mavutil.mavlink.MAV_STATE_STANDBY)
                    last_hb=now
                if now-last_sensors>=0.1:
                    ms=int(now*1000)&0xffffffff
                    self.mav.estimator_status_send(int(now*1e6),37,.1,.1,.1,.1,.1,.1,.4,.4)
                    self.mav.local_position_ned_send(ms,0,0,0,0,0,0)
                    self.mav.highres_imu_send(int(now*1e6),0,0,-9.81,0,0,0,.1,.1,.1,1000,0,100,20,63)
                    last_sensors=now
                if now-last_gps>=0.5:
                    self.mav.gps_raw_int_send(int(now*1e6),3,-62200000,1068000000,100000,100,100,0,0,15)
                    last_gps=now
                r,_,_=select.select([self.fd],[],[],.01)
                if r:
                    data=os.read(self.fd,8192)
                    for b in data:
                        msg=self.reader.parse_char(bytes([b]))
                        if msg:self.incoming(msg)
            except (BlockingIOError,InterruptedError):pass
            except OSError as e:
                if not self.stop:print('SIM_SERIAL_ERROR',repr(e),flush=True)
                return
        print('SIM_COUNTS',self.local_sp,self.global_sp,self.set_mode_count,flush=True)

class Probe(Node):
    def __init__(self):
        super().__init__('pixhawk_sim_probe')
        self.status=None
        self.create_subscription(String,'/pixhawk_sim/status',self.on_status,10)
    def on_status(self,m): self.status=json.loads(m.data)

def call(n,path,cls,request,timeout=5.):
    c=n.create_client(cls,path)
    assert c.wait_for_service(timeout_sec=timeout),path+' missing'
    f=c.call_async(request)
    rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
    assert f.done() and f.result() is not None,path+' response timeout'
    return f.result()

def spin_until(n,predicate,seconds=8.):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if predicate():return True
    return False

def main():
    master,slave=pty.openpty()
    tty.setraw(slave)
    dev=os.ttyname(slave)
    sim=FC(master)
    env=os.environ.copy()
    # Run native compiled executable. All commands go only to the PTY simulator.
    binary=str(Path(get_package_prefix("pixhawk_bridge")) / "lib/pixhawk_bridge/pixhawk_node")
    args=[binary,'--ros-args','-r','__ns:=/pixhawk_sim',
      '-p','device:='+dev,'-p','enable_motion_commands:=true',
      '-p','allow_mode_changes:=true','-p','allow_parameter_writes:=true',
      '-p','allow_mission_writes:=true']
    proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,text=True)
    rclpy.init();n=Probe();sim.start()
    try:
        assert spin_until(n,lambda:n.status is not None and n.status['connected'] and
            n.status['airframe_configured'] and n.status['local_valid'] and n.status['gps_valid']),n.status
        print('SIM_CONNECTED',n.status,flush=True)
        vr=VelocityVector.Request()
        vr.vx_north_mps=0.2
        vr.vy_east_mps=0.1
        vr.duration_sec=0.2
        deny=call(n,'/pixhawk_sim/offboard_velocity_vector',VelocityVector,vr)
        print('SIM_ROVER_VELOCITY_DISABLED',deny.success,deny.message,flush=True)
        assert not deny.success, 'new Rover velocity must remain denied by default'
        req=ParamGet.Request();req.name='SYS_AUTOSTART'
        res=call(n,'/pixhawk_sim/param_get',ParamGet,req)
        print('SIM_PARAM_READ',res.success,res.mav_type,res.value,flush=True)
        assert res.success and res.value==4001
        req=ParamSet.Request();req.name='TEST_GAIN';req.mav_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32;req.value=1.5
        res=call(n,'/pixhawk_sim/param_set',ParamSet,req)
        print('SIM_PARAM_SET',res.success,res.applied_value,flush=True)
        assert res.success and abs(res.applied_value-1.5)<1e-6
        req=GotoLocal.Request();req.x=1.;req.y=1.;req.z=0.;req.yaw=0.;req.duration_sec=5.
        res=call(n,'/pixhawk_sim/goto_local',GotoLocal,req)
        print('SIM_GOTO_LOCAL',res.success,res.message,flush=True)
        assert res.success
        assert spin_until(n,lambda:sim.local_sp>=12,seconds=3.),'no 10Hz local streaming'
        req=SetMode.Request();req.main_mode=6;req.sub_mode=0
        res=call(n,'/pixhawk_sim/set_mode',SetMode,req)
        print('SIM_OFFBOARD_MODE',res.success,res.message,flush=True)
        assert res.success and sim.set_mode_count>=1
        req=GotoGlobal.Request();req.latitude=-6.22;req.longitude=106.8;req.altitude=100.;req.yaw=0.;req.duration_sec=5.
        res=call(n,'/pixhawk_sim/goto_global',GotoGlobal,req)
        print('SIM_GOTO_GLOBAL',res.success,res.message,flush=True)
        assert res.success
        assert spin_until(n,lambda:sim.global_sp>=8,seconds=3.),'no global streaming'
        req=Trigger.Request()
        res=call(n,'/pixhawk_sim/cancel_target',Trigger,req)
        print('SIM_CANCEL',res.success,res.message,flush=True)
        assert res.success
        time.sleep(.3);before=sim.global_sp
        time.sleep(.5)
        assert sim.global_sp==before,'target still streamed after cancel'
        req=MissionDownload.Request();req.mission_type=0
        res=call(n,'/pixhawk_sim/mission_download',MissionDownload,req,timeout=14)
        print('SIM_MISSION_DOWNLOAD',res.success,len(res.items),res.message,flush=True)
        assert res.success and len(res.items)==2
        up=MissionUpload.Request();up.mission_type=0;up.items=res.items
        res=call(n,'/pixhawk_sim/mission_upload',MissionUpload,up,timeout=15)
        print('SIM_MISSION_UPLOAD',res.success,res.message,flush=True)
        assert res.success
        print('SIM_BENCH_PASS',f'local_setpoints={sim.local_sp}',f'global_setpoints={sim.global_sp}',flush=True)
    finally:
        sim.stop=True
        proc.terminate()
        try:
            out,_=proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill();out,_=proc.communicate()
        print('BRIDGE_EXIT',proc.returncode,flush=True)
        print('BRIDGE_LOG_LAST',out[-2500:],flush=True)
        n.destroy_node();rclpy.shutdown()
        os.close(master);os.close(slave)

if __name__=='__main__':
    try:main()
    except Exception as e:
        print('SIM_BENCH_FAIL',repr(e),flush=True)
        sys.exit(1)
