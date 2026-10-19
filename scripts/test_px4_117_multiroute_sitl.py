#!/usr/bin/env python3
"""Integration test against actual PX4 v1.17 Differential Rover SITL + Gazebo.

ROS_DOMAIN_ID=105 ONLY; localhost UDP 18570 via pseudo-TTY ONLY.
Never opens real Cube USB (/dev/ttyACM0). No force ARM, no firmware/param writes.
Mission bounds are validated in tools/waypoint_planner.py before upload.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from pixhawk_interfaces.msg import MissionItem
from pixhawk_interfaces.srv import CommandLong, MissionDownload, MissionUpload, SetMode
from test_px4_real_sitl_ros_mission import BINARY, UDPPTY, ROS

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/"tools"))
from waypoint_planner import ROUTES, plan_route, route_length_m

SITL=(Path(os.environ.get('PX4_SITL_BUILD',str(Path(__file__).resolve().parents[2] / 'px4-firmware/build/px4_sitl_default'))))
EVIDENCE_DIR=BASE/"results/planner_waypoints_20261009"


class MissionObserver(ROS):
    def __init__(self):
        super().__init__()
        self.positions=[]
        self.reached=[]
        self.events=[]
        self.odom_sub=self.create_subscription(
            Odometry,"/px4_sitl/odometry",
            lambda m:self.positions.append([m.pose.pose.position.x,m.pose.pose.position.y]),
            qos_profile_sensor_data)
        self.events_sub=self.create_subscription(
            String,"/px4_sitl/mission/events",self.on_event,10)
    def on_event(self,msg):
        try:
            event=json.loads(msg.data)
        except ValueError:
            return
        self.events.append(event)
        if event.get("type")=="MISSION_ITEM_REACHED":
            self.reached.append(event["seq"])


def px4_mission_result():
    exe=SITL/"bin/px4-listener"
    output=subprocess.run([str(exe),"mission_result"],cwd=SITL,
                          capture_output=True,text=True,timeout=5,check=False)
    if output.returncode!=0:
        raise RuntimeError("PX4 mission_result unavailable: "+output.stderr[:400])
    parsed={}
    for line in output.stdout.splitlines():
        if ":" not in line:
            continue
        key,value=(x.strip() for x in line.split(":",1))
        if key in ("mission_id","seq_reached","seq_current","seq_total"):
            parsed[key]=int(value.split()[0])
        if key in ("valid","finished","failure"):
            parsed[key]=value.split()[0]=="True"
    return parsed


def make_item(i,waypoint):
    item=MissionItem()
    item.seq=i
    item.frame=6  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    item.command=16  # MAV_CMD_NAV_WAYPOINT
    item.current=i==0
    item.autocontinue=True
    item.params=[0.,0.,0.,0.]
    item.x=waypoint.lat_e7
    item.y=waypoint.lon_e7
    item.z=0.
    item.mission_type=0
    return item


def run_route(route,timeout):
    if os.environ.get("ROS_DOMAIN_ID")!="105":
        raise RuntimeError("ABORT: simulator test requires ROS_DOMAIN_ID=105")
    if not (SITL/"bin/px4").exists() or not Path(BINARY).exists():
        raise RuntimeError("PX4 SITL or ROS bridge binary absent")
    matched=subprocess.run(["pgrep","-f",
        '^' + re.escape(str(SITL / 'bin/px4')) + '$'],
        capture_output=True)
    if matched.returncode!=0:
        raise RuntimeError("SITL PX4 1.17 process not running")
    assert route in ROUTES
    connection=UDPPTY()
    assert connection.path.startswith("/dev/pts/"),connection.path
    args=[BINARY,"--ros-args","-r","__ns:=/px4_sitl",
          "-p","device:="+connection.path,
          "-p","enable_motion_commands:=true",
          "-p","allow_parameter_writes:=false",
          "-p","allow_mode_changes:=true",
          "-p","allow_mission_writes:=true",
          "-p","allow_stock_rover_velocity:=false",
          "-p","stock_rover_interface_confirmed:=false"]
    process=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    connection.thread.start()
    rclpy.init()
    ros=MissionObserver()
    record={
        "route":route,"point_count":len(ROUTES[route]),"offsets_ne_m":ROUTES[route],
        "planned_path_length_m":route_length_m(ROUTES[route]),
        "sim_only":True,"physical_cube_used":False,"real_cube_arm":False,
        "ros_domain_id":105,"port":connection.path,"checks":{},
    }
    try:
        assert ros.spin_until(lambda:ros.status is not None and
                              ros.status.get("connected") and
                              ros.status.get("airframe_configured") and
                              ros.status.get("local_valid") and
                              ros.gps is not None and not ros.status.get("armed"),18),ros.status
        assert ros.status["vehicle_type"]==10,"PX4 differential SITL not a Rover"
        lat,lon,alt=ros.gps
        targets=plan_route(lat,lon,ROUTES[route])
        record["home"]=[lat,lon,alt]
        record["targets_wgs84_e7"]=[[p.lat_e7,p.lon_e7] for p in targets]
        record["checks"]["validation"]=True
        before=px4_mission_result()
        upload=MissionUpload.Request()
        upload.mission_type=0
        upload.items=[make_item(i,p) for i,p in enumerate(targets)]
        sent=ros.call("mission_upload",MissionUpload,upload,timeout=27)
        record["checks"]["upload_ack"]=sent.success
        print("MISSION_UPLOAD",route,sent.success,sent.message,flush=True)
        assert sent.success,sent.message
        request=MissionDownload.Request()
        request.mission_type=0
        received=ros.call("mission_download",MissionDownload,request,timeout=25)
        assert received.success and len(received.items)==len(targets)
        assert [(m.x,m.y) for m in received.items]==[
               (w.lat_e7,w.lon_e7) for w in targets]
        record["checks"]["waypoint_roundtrip"]=True
        print("MISSION_READBACK",route,len(received.items),flush=True)
        # Reject invalid WGS84 and unsupported mission frames before any
        # MAVLink MISSION_COUNT is emitted; stored mission must stay intact.
        if route=="line2":
            for defect in ("latitude_out_of_range","frame_unsupported"):
                negative=MissionUpload.Request()
                negative.mission_type=0
                negative.items=[make_item(i,p) for i,p in enumerate(targets)]
                if defect=="latitude_out_of_range":
                    negative.items[0].x=950000000
                else:
                    negative.items[0].frame=255
                denied=ros.call("mission_upload",MissionUpload,negative,timeout=5)
                assert not denied.success and "invalid frame or WGS84" in denied.message,(
                    defect,denied.success,denied.message)
                reread=ros.call("mission_download",MissionDownload,request,timeout=18)
                assert reread.success and [(m.x,m.y) for m in reread.items]==[
                    (w.lat_e7,w.lon_e7) for w in targets],"Rejected mission mutated PX4"
                record["checks"]["reject_"+defect]=True
                print("INVALID_MISSION_REJECTED",defect,denied.message,flush=True)
        # Detect stale completion from previous mission BEFORE accepting mission result.
        ros.spin_until(lambda:False,0.3)
        uploaded_state=px4_mission_result()
        if uploaded_state.get("seq_total")!=len(targets):
            raise AssertionError(("New mission not activated",uploaded_state))
        record["pre_mission_id"]=before.get("mission_id")
        record["uploaded_mission_id"]=uploaded_state.get("mission_id")
        if before.get("mission_id")==uploaded_state.get("mission_id"):
            raise AssertionError("Mission ID unchanged (stale completion hazard)")
        ros.reached.clear()
        ros.events.clear()
        select=SetMode.Request()
        select.main_mode=4
        select.sub_mode=4  # PX4_AUTO_SUB_MODE_MISSION
        mode=ros.call("set_mode",SetMode,select,timeout=10)
        assert mode.success,mode.message
        assert ros.spin_until(lambda:ros.status is not None and
                               ros.status.get("px4_main_mode")==4 and
                               ros.status.get("px4_sub_mode")==4,4)
        record["checks"]["auto_mode"]=True
        ros.spin_until(lambda:len(ros.positions)>1,3)
        starting_pose=ros.positions[-1]
        arm=CommandLong.Request()
        arm.command=400
        arm.params=[1.,0.,0.,0.,0.,0.,0.]
        response=ros.call("command_long",CommandLong,arm,timeout=9)
        assert response.success,response.message
        assert ros.spin_until(lambda:ros.status is not None and ros.status.get("armed"),5)
        record["checks"]["armed_sitl"]=True
        start=time.monotonic()
        progress=[]
        finished=False
        while time.monotonic()-start<timeout:
            ros.spin_until(lambda:False,0.65)
            elapsed=time.monotonic()-start
            state=px4_mission_result()
            progress.append({
                "elapsed_s":round(elapsed,2),
                "sequence":state.get("seq_current"),
                "seq_reached":state.get("seq_reached"),
                "finished":state.get("finished"),
                "failure":state.get("failure"),
            })
            if state.get("failure"):
                break
            if (state.get("seq_total")==len(targets) and
                state.get("seq_reached")==len(targets)-1 and
                state.get("finished") and state.get("valid") and
                not state.get("failure")):
                finished=True
                break
        record["progress"]=progress
        record["mission_result"]=px4_mission_result()
        record["reached_events"]=ros.reached
        record["positions_recorded"]=len(ros.positions)
        distances=[math.dist(a,b) for a,b in zip(ros.positions,ros.positions[1:])]
        assert distances and max(distances)<0.5,("odometry teleport",max(distances) if distances else None)
        record["odometry_path_length_m"]=round(sum(distances),3)
        record["max_step_m"]=round(max(distances),4)
        record["start_position_enu_m"]=starting_pose
        record["end_position_enu_m"]=ros.positions[-1]
        record["net_displacement_m"]=round(
            math.dist(starting_pose,ros.positions[-1]),3)
        record["elapsed_s"]=round(time.monotonic()-start,2)
        record["checks"]["all_waypoints_reached"]=finished
        print("MISSION_RESULT",route,finished,record["mission_result"],
              "time",record["elapsed_s"],"movement",record["net_displacement_m"],flush=True)
        assert finished,f"MISSION NOT COMPLETED {route}: {record['mission_result']}"
        expected=list(range(len(targets)))
        unique=[]
        for seq in ros.reached:
            if not unique or unique[-1]!=seq:
                unique.append(seq)
        assert unique==expected,("Incomplete/out-of-order reached event sequence",unique,expected)
        record["unique_reached_sequence"]=unique
        record["checks"]["ros_waypoint_events"]=True
        record["checks"]["model_moved"]=record["odometry_path_length_m"]>=1.0
        assert record["checks"]["model_moved"],record
        print("MULTI_WAYPOINT_SITL_PASS",route,len(targets),flush=True)
    finally:
        # Never disarm physical Cube: this binary is exclusive to local SITL process.
        subprocess.run([str(SITL/"bin/px4-commander"),"disarm"],cwd=SITL,
                       capture_output=True,timeout=5,check=False)
        ros.spin_until(lambda:ros.status and not ros.status.get("armed"),2)
        record["disarmed_sitl"]=not (ros.status or {}).get("armed",True)
        record["udp_received_packets"]=connection.received
        record["udp_sent_packets"]=connection.sent
        EVIDENCE_DIR.mkdir(parents=True,exist_ok=True)
        output=EVIDENCE_DIR/(route+".json")
        output.write_text(json.dumps(record,indent=2))
        print("RESULT_FILE",output,flush=True)
        ros.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            bridge_log,_=process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            bridge_log,_=process.communicate(timeout=4)
        (EVIDENCE_DIR/(route+"_bridge.log")).write_text(bridge_log)
        connection.close()
        print("SIM_ONLY_CLEANUP_DONE",flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--route",choices=ROUTES.keys(),required=True)
    parser.add_argument("--timeout",type=float,default=65.)
    args=parser.parse_args()
    if not 5<=args.timeout<=95:
        parser.error("timeout must be 5..95 s")
    run_route(args.route,args.timeout)


if __name__=="__main__":
    main()
