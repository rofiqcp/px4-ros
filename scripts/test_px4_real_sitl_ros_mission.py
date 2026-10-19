#!/usr/bin/env python3
"""PX4 v1.17 Rover SITL ↔ real ROS2 compiled bridge over UDP/PTTY.
Strict isolation: no physical serial device is opened, domain 103 required.
Does NOT arm hardware or change Cube parameters.
"""
import json
import os
import pty
import select
import socket
import subprocess
import threading
import time
import tty
from pathlib import Path

from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
from pixhawk_interfaces.srv import MissionDownload, MissionUpload, GotoLocal, SetMode
from pixhawk_interfaces.msg import MissionItem

BINARY=str(Path(get_package_prefix("pixhawk_bridge")) / "lib/pixhawk_bridge/pixhawk_node")
OUT=(Path(__file__).resolve().parents[1] / "results/cube2_probe/px4_117_real_sitl_ros_mission.json")

class UDPPTY:
    def __init__(self):
        self.master,self.slave=pty.openpty()
        tty.setraw(self.slave)
        self.path=os.ttyname(self.slave)
        assert self.path.startswith('/dev/pts/'),self.path
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',14550))
        self.sock.setblocking(False)
        os.set_blocking(self.master,False)
        self.stop=False
        self.received=0
        self.sent=0
        self.thread=threading.Thread(target=self.loop,daemon=True)
    def loop(self):
        while not self.stop:
            try:
                ready,_,_=select.select([self.sock,self.master],[],[],.04)
                if self.sock in ready:
                    data,_=self.sock.recvfrom(4096)
                    if data:
                        self.received+=1
                        os.write(self.master,data)
                if self.master in ready:
                    data=os.read(self.master,4096)
                    if data:
                        self.sent+=1
                        self.sock.sendto(data,('127.0.0.1',18570))
            except (BlockingIOError,InterruptedError):
                continue
            except OSError:
                if not self.stop: raise
    def close(self):
        self.stop=True
        self.thread.join(timeout=2)
        self.sock.close()
        os.close(self.master);os.close(self.slave)
class ROS(Node):
    def __init__(self):
        super().__init__('px4_real_sitl_verification_client')
        self.status=None
        self.subscription=self.create_subscription(
            String,'/px4_sitl/status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        self.gps=None
        self.gps_subscription=self.create_subscription(
            NavSatFix,'/px4_sitl/gps/fix',
            lambda m:setattr(self,'gps',(m.latitude,m.longitude,m.altitude)),
            qos_profile_sensor_data)
    def spin_until(self,predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.06)
            if predicate():return True
        return False
    def call(self,name,service,request,timeout=16.):
        c=self.create_client(service,'/px4_sitl/'+name)
        assert c.wait_for_service(timeout_sec=5),'Service absent '+name
        f=c.call_async(request)
        rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
        assert f.done() and f.result() is not None,'Service timeout '+name
        return f.result()

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='103':
        raise RuntimeError('Must only run in ROS_DOMAIN_ID=103 isolated simulator')
    assert Path(BINARY).exists()
    pty_udp=UDPPTY()
    args=[BINARY,'--ros-args','-r','__ns:=/px4_sitl',
          '-p','device:='+pty_udp.path,
          '-p','enable_motion_commands:=true',
          '-p','allow_parameter_writes:=false',
          '-p','allow_mission_writes:=true',
          '-p','allow_mode_changes:=true',
          '-p','allow_stock_rover_velocity:=false']
    p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    pty_udp.thread.start()
    rclpy.init()
    n=ROS()
    result={'source':'ACTUAL_PX4_V1.17_SITL_GAZEBO_OVER_LOOPBACK_UDP',
            'physical_cube_used':False,'cube_mission_or_params_modified':False,
            'serial_path':pty_udp.path,'tests':{}}
    try:
        assert n.spin_until(lambda:n.status is not None and n.status.get('connected'),15),n.status
        print('REAL_PX4_SITL_STATUS',n.status,flush=True)
        result['starting_status']=n.status
        assert n.status.get('vehicle_type')==10,'not Rover SITL'
        assert n.status.get('airframe_configured'),'SYS_AUTOSTART not configured in SITL'
        m=MissionDownload.Request();m.mission_type=0
        got=n.call('mission_download',MissionDownload,m,timeout=25)
        result['tests']['mission_before']={'success':got.success,'count':len(got.items),'message':got.message}
        print('SITL_MISSION_BEFORE',got.success,len(got.items),got.message,flush=True)
        assert got.success
        # Only simulation: use actual SITL GPS so waypoints remain near its home.
        assert n.spin_until(lambda:n.gps is not None,8),'SITL GPS not received'
        home_lat,home_lon,home_alt=n.gps
        assert abs(home_lat)<90 and abs(home_lon)<180 and abs(home_lat)>0.1
        result['sim_home_wgs84']={'latitude':home_lat,'longitude':home_lon,'altitude_m':home_alt}
        items=[]
        for seq,offset in enumerate([90,160]):
            q=MissionItem()
            q.seq=seq;q.frame=6;q.command=16
            q.current=(seq==0);q.autocontinue=True;q.params=[0.,0.,0.,0.]
            q.x=int(round(home_lat*1e7))+offset
            q.y=int(round(home_lon*1e7))+offset
            q.z=0.;q.mission_type=0
            items.append(q)
        up=MissionUpload.Request();up.mission_type=0;up.items=items
        posted=n.call('mission_upload',MissionUpload,up,timeout=29)
        result['tests']['mission_upload']={'success':posted.success,'message':posted.message}
        print('SITL_MISSION_UPLOAD',posted.success,posted.message,flush=True)
        check=n.call('mission_download',MissionDownload,m,timeout=25)
        result['tests']['mission_after']={'success':check.success,'count':len(check.items),
                                         'message':check.message}
        print('SITL_MISSION_AFTER',check.success,len(check.items),check.message,flush=True)
        # No GOTO unless model has an estimator. This is SIMULATION only.
        if n.status.get('local_valid',False):
            goto=GotoLocal.Request()
            goto.x=0.3;goto.y=0.;goto.z=0.;goto.yaw=0.;goto.duration_sec=3.
            response=n.call('goto_local',GotoLocal,goto,timeout=10)
            result['tests']['goto_local_setpoint']={'accepted':response.success,'reason':response.message}
            print('SITL_GOTO_LOCAL_SETPOINT',response.success,response.message,flush=True)
            if response.success:
                # PX4 requires >1 s prestream before explicit OFFBOARD request.
                time.sleep(1.4)
                requested=SetMode.Request()
                requested.main_mode=6;requested.sub_mode=0
                mode=n.call('set_mode',SetMode,requested,timeout=9.)
                n.spin_until(lambda:n.status is not None and n.status.get('px4_main_mode')==6,2.)
                result['tests']['offboard_mode_actual_px4_sitl']={
                  'accepted':mode.success,'ack':mode.message,
                  'observed_main_mode':n.status.get('px4_main_mode'),
                  'rover_armed':n.status.get('armed'),
                  'position_target_reached':False
                }
                print('SITL_OFFBOARD_MODE_ACTUAL_PX4',mode.success,mode.message,
                      'observed_mode',n.status.get('px4_main_mode'),flush=True)

        else:
            result['tests']['goto_local_setpoint']={
              'accepted':False,'reason':'SITL estimator/local position invalid; not allowed to assume GOTO works'}
            print('SITL_GOTO_LOCAL_SKIPPED_ESTIMATOR_INVALID',flush=True)
        result['tests']['mission_roundtrip_verified']=bool(posted.success and check.success and len(check.items)==2)
        print('PX4_117_REAL_SITL_ROS_BRIDGE_MISSION_ROUNDTRIP',
              result['tests']['mission_roundtrip_verified'],flush=True)
    finally:
        result['udp_in_packets']=pty_udp.received
        result['udp_out_packets']=pty_udp.sent
        OUT.parent.mkdir(parents=True,exist_ok=True)
        OUT.write_text(json.dumps(result,indent=2))
        print('EVIDENCE',OUT,flush=True)
        n.destroy_node();rclpy.shutdown()
        p.terminate()
        try: logs,_=p.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill();logs,_=p.communicate(timeout=4)
        print('ROS_BRIDGE_LAST_LINES',logs[-1800:],flush=True)
        pty_udp.close()

if __name__=='__main__':
    main()
