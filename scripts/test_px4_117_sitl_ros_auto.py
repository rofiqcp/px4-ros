#!/usr/bin/env python3
"""ROS2 commands actual PX4 Differential Rover SITL AUTO mission, no physical Cube.
Safeguards: exact ROS_DOMAIN_ID=104, localhost SITL UDP, PTY not ttyACM,
command time cap, and simulator-only disarm in finally.
"""
import json
import math
import os
import re
from pathlib import Path
import subprocess

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from pixhawk_interfaces.msg import MissionItem
from pixhawk_interfaces.srv import MissionUpload, MissionDownload, CommandLong, SetMode
from test_px4_real_sitl_ros_mission import UDPPTY, ROS, BINARY

ROOT=(Path(os.environ.get('PX4_SITL_BUILD',str(Path(__file__).resolve().parents[2] / 'px4-firmware/build/px4_sitl_default'))))
OUT=(Path(__file__).resolve().parents[1] / "results/cube2_probe/px4_117_auto_from_ros2.json")

class Observer(ROS):
    def __init__(self):
        super().__init__()
        self.poses=[]
        self.mission_events=[]
        self.odom_sub=self.create_subscription(
          Odometry,'/px4_sitl/odometry',
          lambda msg:self.poses.append((msg.pose.pose.position.x,msg.pose.pose.position.y)),
          qos_profile_sensor_data)
        self.events_sub=self.create_subscription(
          String,'/px4_sitl/mission/events',lambda msg:self.mission_events.append(msg.data),10)

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='104','Not isolated simulator ROS domain'
    assert (ROOT/'bin/px4').exists()
    assert subprocess.run(['pgrep','-f',
       '^' + re.escape(str(ROOT / 'bin/px4')) + '$'],
       capture_output=True).returncode==0,'PX4 SITL not running'
    io=UDPPTY()
    assert io.path.startswith('/dev/pts/')
    args=[BINARY,'--ros-args','-r','__ns:=/px4_sitl',
      '-p','device:='+io.path,'-p','enable_motion_commands:=true',
      '-p','allow_parameter_writes:=false','-p','allow_mission_writes:=true',
      '-p','allow_mode_changes:=true','-p','allow_stock_rover_velocity:=false']
    p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    io.thread.start()
    rclpy.init();node=Observer()
    evidence={'target':'ACTUAL_PX4_1_17_DIFFERENTIAL_SITL_ONLY',
      'physical_cube_used':False,'physical_cube_armed':False,
      'port':io.path,'ros_domain_id':104,'tests':{}}
    try:
        assert node.spin_until(lambda:node.status and node.status['connected'] and
          node.status['airframe_configured'] and not node.status['armed'] and node.gps,15)
        assert node.status['vehicle_type']==10, 'expected MAV_TYPE_GROUND_ROVER'
        lat,lon,alt=node.gps
        assert node.status['local_valid'],node.status
        evidence['home_gps']=[lat,lon,alt]
        print('SIM_ROVER_READY',node.status['mav_state'],lat,lon,flush=True)
        mission=MissionUpload.Request();mission.mission_type=0
        pts=[]
        for i,offset in enumerate((90,160)):
            q=MissionItem()
            q.seq=i;q.mission_type=0;q.frame=6;q.command=16
            q.current=i==0;q.autocontinue=True;q.params=[0.,0.,0.,0.]
            q.x=int(round(lat*1e7))+offset
            q.y=int(round(lon*1e7))+offset
            q.z=0.
            pts.append(q)
        mission.items=pts
        posted=node.call('mission_upload',MissionUpload,mission,timeout=25)
        print('ROS_UPLOAD_TO_ACTUAL_PX4',posted.success,posted.message,flush=True)
        assert posted.success,posted.message
        req=MissionDownload.Request();req.mission_type=0
        got=node.call('mission_download',MissionDownload,req,timeout=25)
        assert got.success and len(got.items)==2,got.message
        assert [(x.x,x.y) for x in got.items]==[(x.x,x.y) for x in pts]
        print('ROS_READBACK_FROM_ACTUAL_PX4_PASS',len(got.items),flush=True)
        evidence['tests']['mission_roundtrip']=True
        reqmode=SetMode.Request();reqmode.main_mode=4;reqmode.sub_mode=4
        selected=node.call('set_mode',SetMode,reqmode,timeout=10)
        evidence['tests']['auto_mode_request']={
          'success':selected.success,'ack':selected.message}
        print('ROS_AUTO_MISSION_MODE_REQUEST',selected.success,selected.message,flush=True)
        assert selected.success,selected.message
        node.spin_until(lambda:node.status and node.status['px4_main_mode']==4 and
          node.status['px4_sub_mode']==4,4)
        if not node.status or node.status['px4_sub_mode']!=4:
            raise RuntimeError('PX4 did not enter AUTO Mission')
        node.spin_until(lambda:len(node.poses)>=2,3)
        initial=node.poses[-1] if node.poses else None
        evidence['initial_odom_xy']=initial
        arm=CommandLong.Request();arm.command=400
        arm.params=[1.,0.,0.,0.,0.,0.,0.]
        armed=node.call('command_long',CommandLong,arm,timeout=10)
        evidence['tests']['arm_simulation_only']={'success':armed.success,'ack':armed.message}
        print('ROS_ARM_SIMULATION_ONLY',armed.success,armed.message,flush=True)
        assert armed.success,armed.message
        assert node.spin_until(lambda:node.status and node.status['armed'],4)
        # Observe real PX4+Gazebo model, never issue a physical Cube movement.
        node.spin_until(lambda:False,10)
        final=node.poses[-1] if node.poses else None
        displacement=math.dist(initial,final) if initial and final else 0.
        evidence['final_odom_xy']=final
        evidence['displacement_m']=displacement
        print('ROS_TO_PX4_SITL_MODEL_MOVED',displacement,flush=True)
        assert displacement>0.4,'No detected simulated position change'
        evidence['tests']['moved_in_gazebo']=True
        # ROS disarm, regardless of mission outcome.
        disarm=CommandLong.Request();disarm.command=400
        disarm.params=[0.,0.,0.,0.,0.,0.,0.]
        stopped=node.call('command_long',CommandLong,disarm,timeout=10)
        print('ROS_DISARM_SIMULATOR',stopped.success,stopped.message,flush=True)
        assert stopped.success
        evidence['tests']['disarmed_after_test']=True
        node.spin_until(lambda:node.status and not node.status['armed'],3)
        listener=subprocess.run([str(ROOT/'bin/px4-listener'),'mission_result'],
               cwd=ROOT,capture_output=True,text=True,timeout=5)
        relevant=[line.strip() for line in listener.stdout.splitlines()
                  if any(t in line for t in ('seq_reached:','seq_current:','seq_total:',
                                             'valid:','finished:','failure:'))]
        evidence['mission_result_from_actual_px4']=relevant
        print('PX4_REAL_SITL_MISSION_RESULT',relevant,flush=True)
        evidence['tests']['mission_finished']='finished: True' in listener.stdout and (
          'failure: False' in listener.stdout)
        print('ROS_ONLY_PX4_DIFFERENTIAL_MISSION_RESULT',
          evidence['tests']['mission_finished'],flush=True)
    finally:
        # Hardware-safe: command is local PX4 SITL client binary ONLY.
        subprocess.run([str(ROOT/'bin/px4-commander'),'disarm'],cwd=ROOT,
                       capture_output=True,timeout=5)
        evidence['udp_in']=io.received
        evidence['udp_out']=io.sent
        OUT.parent.mkdir(parents=True,exist_ok=True)
        OUT.write_text(json.dumps(evidence,indent=2))
        print('REPORT',OUT,flush=True)
        node.destroy_node();rclpy.shutdown()
        p.terminate()
        try:logs,_=p.communicate(timeout=4)
        except subprocess.TimeoutExpired:p.kill();logs,_=p.communicate(timeout=4)
        print('ROS_BRIDGE_LAST',logs[-1300:],flush=True)
        io.close()

if __name__=='__main__':main()
