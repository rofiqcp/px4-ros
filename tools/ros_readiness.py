#!/usr/bin/env python3
"""Read-only ROS2 integration doctor, no Cube/actuator commands."""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml

ROOT=Path(__file__).resolve().parents[1]
EXTRINSICS=(ROOT.parent / 'slam3d-livox-mid360s/config/extrinsics.yaml')
WATCH=[
  ('/livox/lidar',PointCloud2),('/livox/imu',Imu),
  ('/lio/odometry',Odometry),('/pixhawk/status',String),
  ('/pixhawk/differential/diagnostics',String),
  ('/pixhawk/avoidance/diagnostics',String),('/pixhawk/stock/status',String)
]
class Observer(Node):
    def __init__(self):
        super().__init__('stock_px4_readonly_ros_doctor')
        self.samples=defaultdict(list)
        self.values={}
        self.subs=[]
        for topic,cls in WATCH:
            self.subs.append(self.create_subscription(
                cls,topic,lambda m,t=topic:self.receive(t,m),10))
    def receive(self,topic,msg):
        self.samples[topic].append(time.monotonic())
        if isinstance(msg,String):
            try:self.values[topic]=json.loads(msg.data)
            except ValueError:self.values[topic]={'raw':msg.data}
        else:self.values[topic]={'frame_id':msg.header.frame_id}
def bridge_param(name):
    cmd=['ros2','param','get','/pixhawk/pixhawk_bridge',name]
    try:
        r=subprocess.run(cmd,capture_output=True,text=True,timeout=6)
        return r.stdout.strip() if r.returncode==0 else 'UNKNOWN'
    except (subprocess.TimeoutExpired, OSError):
        return 'UNKNOWN'
def main():
    p=argparse.ArgumentParser()
    p.add_argument('--duration',type=float,default=7.)
    p.add_argument('--output',default=str(ROOT/'results/ros_readiness_20261009.json'))
    args=p.parse_args()
    if not 2<=args.duration<=30:raise ValueError('duration must be 2..30 sec')
    rclpy.init()
    n=Observer()
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(n,timeout_sec=.1)
        topics={}
        for t,cls in WATCH:
            stamps=n.samples[t]
            rate=((len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>=2
                  and stamps[-1]>stamps[0] else 0.)
            topics[t]={'messages':len(stamps),'observed_rate_hz':round(rate,2),
                       'message_type':cls.__name__}
        try:calib=yaml.safe_load(EXTRINSICS.read_text())['extrinsics']['status']
        except (OSError,KeyError,TypeError,ValueError):calib='UNKNOWN'
        perm={k:bridge_param(k) for k in
           ('enable_motion_commands','allow_parameter_writes','allow_mode_changes',
            'allow_mission_writes')}
        locks=all(v=='Boolean value is: False' for v in perm.values())
        safety=n.values.get('/pixhawk/stock/status',{})
        px4=n.values.get('/pixhawk/status',{})
        avoid=n.values.get('/pixhawk/avoidance/diagnostics',{})
        diff=n.values.get('/pixhawk/differential/diagnostics',{})
        lidar_live=topics['/livox/lidar']['messages']>=3
        imu_live=topics['/livox/imu']['messages']>=3
        lio_live=topics['/lio/odometry']['messages']>=3
        active=len(n.samples['/pixhawk/stock/status'])>=3 and bool(safety)
        perception=len(n.samples['/pixhawk/avoidance/diagnostics'])>=3 and bool(avoid)
        sensor_prepared=lidar_live and imu_live and lio_live
        report={
          'timestamp_utc':datetime.now(timezone.utc).isoformat(),
          'ros_domain_id':int(os.environ.get('ROS_DOMAIN_ID','0')),
          'sample_window_s':args.duration,
          'topics':topics,'extrinsic_calibration':calib,
          'cube_motion_locked':locks,'cube_live_permissions':perm,
          'ros_observer_present':active,'perception_present':perception,
          'sensor_streams_live':sensor_prepared,
          'safety_state':safety.get('state','UNKNOWN'),
          'safety_reason':safety.get('reason','UNKNOWN'),
          'perception_state':avoid.get('state','UNKNOWN'),
          'differential_advisor_present':len(n.samples['/pixhawk/differential/diagnostics'])>=3,
          'differential_controller_type':diff.get('controller_type','UNKNOWN'),
          'differential_preview_valid':diff.get('preview_valid',False),
          'differential_wheel_track_verified':diff.get('wheel_track_verified',False),
          'differential_hardware_command_enabled':diff.get('hardware_command_enabled',False),
          'px4_link_connected':px4.get('connected',False),
          'px4_airframe_configured':px4.get('airframe_configured',False),
          'px4_localization_valid':px4.get('local_valid',False),
          'hardware_avoidance_verified':False,
          'autonomous_mission_ready':False,
          'ros_advisory_runtime_ready':bool(active and perception and locks),
          'ros_sensor_pipeline_ready':bool(active and perception and locks and sensor_prepared and calib=='verified'),
          'safe_to_enable_motors':False,
          'blockers':[]
        }
        if calib!='verified':report['blockers'].append('LiDAR-to-base extrinsics not calibrated and verified')
        if not sensor_prepared:report['blockers'].append('LiDAR/IMU/LIO live streams not all observed')
        if not active:report['blockers'].append('Stock PX4 safety supervisor unavailable')
        if not perception:report['blockers'].append('Obstacle perception diagnostics unavailable')
        if not locks:report['blockers'].append('Cube command permissions not verified locked')
        if not report['px4_localization_valid']:report['blockers'].append('PX4 horizontal localization invalid')
        if not report['px4_airframe_configured']:report['blockers'].append('PX4 Rover airframe not configured')
        if not px4.get('rc_telemetry_fresh',False):report['blockers'].append('Physical receiver priority unverified')
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        temp=path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(report,indent=2,sort_keys=True))
        temp.replace(path)
        print(json.dumps(report,indent=2),flush=True)
        print('ROS_READINESS_REPORT_SAVED',path,flush=True)
    finally:
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
