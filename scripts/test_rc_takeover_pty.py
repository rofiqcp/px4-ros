#!/usr/bin/env python3
"""No hardware: PX4-like MAVLink PTY receiver + RC mode takeover regression."""
import os
import pty
import select
import subprocess
import time
import tty

os.environ['MAVLINK20'] = '1'
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy
from pymavlink import mavutil
from pixhawk_interfaces.srv import VelocityVector, GotoLocal, SetMode, MissionDownload
from sim_bench import FC, Probe, call, spin_until

BINARY = str(Path(get_package_prefix("pixhawk_bridge")) / "lib/pixhawk_bridge/pixhawk_node")

class RoverFC(FC):
    def __init__(self, fd):
        self.px4_main_mode = 6  # OFFBOARD heartbeat in test only
        self.rc_link = True
        self.velocity_packets = []
        self.rc_transmissions = 0
        super().__init__(fd)

    def incoming(self, msg):
        if msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
            self.velocity_packets.append((time.monotonic(), msg.type_mask, msg.coordinate_frame, msg.vx, msg.vy))
        super().incoming(msg)

    def loop(self):
        os.set_blocking(self.fd, False)
        last_hb = last_sensor = last_rc = 0.
        while not self.stop:
            now = time.monotonic()
            try:
                if now-last_hb >= 0.1:
                    custom = self.px4_main_mode << 16
                    self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                        mavutil.mavlink.MAV_AUTOPILOT_PX4,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        custom, mavutil.mavlink.MAV_STATE_STANDBY)
                    last_hb = now
                if now-last_sensor >= 0.1:
                    ms = int(now*1000) & 0xffffffff
                    self.mav.estimator_status_send(int(now*1e6),37,.1,.1,.1,.1,.1,.1,.4,.4)
                    self.mav.local_position_ned_send(ms,0,0,0,0,0,0)
                    self.mav.highres_imu_send(int(now*1e6),0,0,-9.81,0,0,0,.1,.1,.1,1000,0,100,20,63)
                    last_sensor = now
                if self.rc_link and now-last_rc >= 0.1:
                    chans = [1500] * 4 + [0] * 14
                    self.mav.rc_channels_send(int(now*1000)&0xffffffff, 4, *chans, 100)
                    self.rc_transmissions += 1
                    last_rc = now
                readable,_,_ = select.select([self.fd],[],[],0.01)
                if readable:
                    data=os.read(self.fd,8192)
                    for byte in data:
                        msg=self.reader.parse_char(bytes([byte]))
                        if msg:self.incoming(msg)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError as e:
                if not self.stop:print('PTY_IO_ERROR',str(e),flush=True)
                break

def main():
    master, slave=pty.openpty()
    tty.setraw(slave)
    sim=RoverFC(master)
    env=os.environ.copy()
    args=[BINARY,'--ros-args','-r','__ns:=/pixhawk_sim',
          '-p','device:='+os.ttyname(slave),
          '-p','enable_motion_commands:=true',
          '-p','allow_mode_changes:=true',
          '-p','allow_stock_rover_velocity:=true',
          '-p','stock_rover_interface_confirmed:=true']
    proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,text=True)
    rclpy.init()
    node=Probe()
    sim.start()
    try:
        assert spin_until(node,lambda:node.status is not None and
            node.status['airframe_configured'] and node.status['local_valid'] and
            node.status['vehicle_type']==mavutil.mavlink.MAV_TYPE_GROUND_ROVER and
            node.status['rc_telemetry_fresh'],seconds=6),node.status
        print('ROVER_SIM_MODE',node.status['px4_main_mode'],'RC_RECEIVER_TELEM',node.status['rc_telemetry_fresh'])
        req=VelocityVector.Request()
        req.vx_north_mps=.25
        req.vy_east_mps=.1
        req.duration_sec=.3
        start=time.monotonic()
        while time.monotonic()-start<1.5:
            res=call(node,'/pixhawk_sim/offboard_velocity_vector',VelocityVector,req,timeout=4.)
            assert res.success,res.message
            time.sleep(.09)
        assert len(sim.velocity_packets)>=7, ('no velocity frame stream',sim.velocity_packets)
        for t,mask,frame,x,y in sim.velocity_packets:
            assert mask==0x0DE7,hex(mask)
            assert frame==mavutil.mavlink.MAV_FRAME_LOCAL_NED
            assert abs(x-.25)<1e-5 and abs(y-.1)<1e-5
        print('OFFBOARD_VECTOR_PTY_PASS',len(sim.velocity_packets),'frames, planar mask',hex(sim.velocity_packets[0][1]),flush=True)
        sim.px4_main_mode=1  # Physical MODE SWITCH in virtual PX4
        assert spin_until(node,lambda:node.status is not None and
            node.status['px4_main_mode']==1 and node.status['manual_mode_selected'],seconds=3),node.status
        time.sleep(.35)
        stop_count=len(sim.velocity_packets)
        time.sleep(.40)
        assert len(sim.velocity_packets)==stop_count, 'setpoint continued after MANUAL takeover'
        result=call(node,'/pixhawk_sim/offboard_velocity_vector',VelocityVector,req)
        assert not result.success, 'new ROS motion erroneously accepted during manual control'
        mode=SetMode.Request()
        mode.main_mode=6
        mode.sub_mode=0
        rejected=call(node,'/pixhawk_sim/set_mode',SetMode,mode)
        assert not rejected.success and 'RC_MANUAL_TAKEOVER_LATCHED' in rejected.message
        target=GotoLocal.Request()
        target.x=1.;target.y=1.;target.z=0.;target.yaw=0.;target.duration_sec=5.
        rejected_target=call(node,'/pixhawk_sim/goto_local',GotoLocal,target)
        assert not rejected_target.success and 'RC_MANUAL_TAKEOVER_LATCHED' in rejected_target.message
        read=MissionDownload.Request()
        read.mission_type=0
        mission=call(node,'/pixhawk_sim/mission_download',MissionDownload,read,timeout=14.)
        assert mission.success and len(mission.items)==2,mission.message
        print('RC_MANUAL_TAKEOVER_PASS','stopped_frames=',stop_count,
              'new_requests_denied=',not result.success,
              'mode_requests_denied=',not rejected.success,
              'mission_preserved=',len(mission.items),flush=True)
        sim.px4_main_mode=6
        assert spin_until(node,lambda:node.status and node.status['px4_main_mode']==6,seconds=3)
        time.sleep(.40)
        assert len(sim.velocity_packets)==stop_count,'ROS setpoint restarted without explicit new lease'
        denied=call(node,'/pixhawk_sim/offboard_velocity_vector',VelocityVector,req)
        assert not denied.success and 'RC_MANUAL_TAKEOVER_LATCHED' in denied.message,denied.message
        print('NO_AUTO_RESUME_AFTER_RC_PASS','manual latch remains active',flush=True)
        sim.rc_link=False
        assert spin_until(node,lambda:node.status is not None and
            not node.status['rc_telemetry_fresh'],seconds=4),'RC stale detection failed'
        print('RC_TELEMETRY_TIMEOUT_PASS',flush=True)
        # While ROS bridge stops, simulated receiver is still an independent PX4 input.
        before_rc=sim.rc_transmissions
        sim.rc_link=True
        proc.terminate()
        proc.communicate(timeout=4)
        time.sleep(.4)
        assert sim.rc_transmissions>before_rc,'RC depended on ROS bridge'
        print('ROS_PROCESS_STOP_RC_INDEPENDENT_PTY_PASS',flush=True)
        print('ALL_RC_TAKEOVER_PTY_PASS',flush=True)
    finally:
        sim.stop=True
        if proc.poll() is None:
            proc.terminate()
            try:proc.communicate(timeout=4)
            except subprocess.TimeoutExpired:proc.kill();proc.communicate()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master);os.close(slave)

if __name__=='__main__':main()
