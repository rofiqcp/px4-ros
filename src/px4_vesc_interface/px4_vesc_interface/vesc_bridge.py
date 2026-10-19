"""VESC dual-motor preview, real UART disabled unless independently interlocked.

VESC dual FOC firmware is not an Ackermann steering servo. We project an
Ackermann curvature into two differential wheel speeds and separately publish
steering angle for a *future* calibrated steering actuator. Never assume that
PX4 AUTO.MISSION allows direct external steering setpoints.
"""
import json
import os
from pathlib import Path
import sys
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from .kinematics import drive

ARM_GUARD = 'I_UNDERSTAND_LIVE_MOTOR_CONTROL'

class Preview(Node):
    def __init__(self):
        super().__init__('vesc_preview')
        for name,value in [
            ('enable_output',False),('hardware_verified',False),
            ('external_drive_authorized',False),('vesc_port',''),
            ('vesc_firmware_root',''),('vesc_baud',115200),
            ('wheelbase_m',0.),('track_width_m',0.),('wheel_radius_m',0.),
            ('motor_pole_pairs',15),('transmission_ratio',1.),
            ('max_speed_mps',.35),('max_steer_rad',.45),
            ('max_erpm',1500),('command_timeout_s',.30),('px4_timeout_s',2.5)]:
            self.declare_parameter(name,value)
        self.command=None;self.command_time=0.;self.px4={};self.px4_time=0.
        self.transport=None;self.last_error=''
        self.create_subscription(TwistStamped,'/px4_mission/preview_twist',self.on_cmd,10)
        self.create_subscription(String,'/pixhawk/status',self.on_px4,10)
        self.pub=self.create_publisher(String,'/vesc_ackermann/diagnostics',10)
        self.create_timer(.1,self.tick)
        self.get_logger().info('VESC dual motor preview; outputs DISABLED until manual commissioning')

    def prm(self,name):return self.get_parameter(name).value
    def on_cmd(self,msg):
        x,z=msg.twist.linear.x,msg.twist.angular.z
        if math.isfinite(x) and math.isfinite(z):
            self.command=(x,z);self.command_time=time.monotonic()
    def on_px4(self,msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict):
                self.px4=value;self.px4_time=time.monotonic()
        except (TypeError,ValueError):
            self.px4={};self.px4_time=0.
    def connect_vesc(self):
        port=self.prm('vesc_port')
        if not port or port.startswith('/dev/ttyACM'):
            raise ValueError('Dedicated VESC USB-UART required; PX4 ttyACM is forbidden')
        if not Path(port).exists():
            raise OSError('VESC USB-UART port not found')
        candidates = [
            self.prm('vesc_firmware_root'),
            os.environ.get('VESC_FIRMWARE_ROOT'),
            str(Path.home() / 'Project' / 'hoverboard-vesc'),
            str(Path(__file__).resolve().parents[4] / 'hoverboard-vesc'),
        ]
        firmware = next((Path(p).expanduser().resolve() for p in candidates
                         if p and (Path(p).expanduser() / 'tools' / 'vesc_dual.py').is_file()), None)
        if firmware is None:
            raise FileNotFoundError('Clone https://github.com/rofiqcp/hoverboard-vesc into ~/Project/hoverboard-vesc or set vesc_firmware_root')
        tools = firmware / 'tools'
        if str(tools) not in sys.path:sys.path.insert(0,str(tools))
        from vesc_dual import VescDual
        return VescDual(port,baud=int(self.prm('vesc_baud')),timeout=.08)
    def tick(self):
        now=time.monotonic()
        live=now-self.px4_time<max(.5,float(self.prm('px4_timeout_s')))
        offboard=(self.px4.get('px4_main_mode')==6)
        mission=(self.px4.get('px4_main_mode')==4 and self.px4.get('px4_sub_mode')==4)
        enabled=(self.prm('enable_output') and self.prm('hardware_verified') and
                 self.prm('external_drive_authorized') and
                 os.environ.get('PX4_VESC_ARM_GUARD')==ARM_GUARD)
        rc_ok=(self.px4.get('rc_telemetry_fresh') is True and
               self.px4.get('manual_takeover_latched') is False)
        gate=(enabled and live and offboard and not mission and rc_ok and
              self.px4.get('connected') is True and
              self.px4.get('airframe_configured') is True and
              self.px4.get('armed') is True)
        fresh=self.command is not None and now-self.command_time<float(self.prm('command_timeout_s'))
        cmd=self.command if fresh else (0.,0.)
        result=drive(*cmd,self.prm('wheelbase_m'),self.prm('track_width_m'),
            self.prm('wheel_radius_m'),self.prm('motor_pole_pairs'),
            self.prm('transmission_ratio'),self.prm('max_speed_mps'),
            self.prm('max_steer_rad'),self.prm('max_erpm'))
        steering,left,right,l_erpm,r_erpm,reason=result
        # Require valid, calibrated geometry. Never drive using default zero geometry.
        if reason=='GEOMETRY_UNVERIFIED' or not fresh:
            gate=False
        if gate and self.transport is None:
            try:self.transport=self.connect_vesc();self.last_error=''
            except Exception as exc:self.last_error=str(exc);gate=False
        if self.transport is not None:
            try:self.transport.set_rpm(l_erpm if gate else 0,r_erpm if gate else 0)
            except Exception as exc:
                self.last_error=str(exc);gate=False
                # Never send another nonzero command on a failed transport.
                try:self.transport.close()
                except Exception:pass
                self.transport=None
            if not gate and self.transport is not None:
                try:self.transport.close()
                except Exception:pass
                self.transport=None
        state={
            'output_enabled':bool(gate and self.transport is not None),
            'armed_guard_requested':bool(enabled),'px4_auto_mission':mission,
            'px4_offboard':offboard,'rc_verified':rc_ok,
            'cmd_fresh':fresh,'geometry':reason,
            'virtual_ackermann_steering_rad':steering,
            'wheel_left_mps':left,'wheel_right_mps':right,
            'left_erpm_preview':l_erpm,'right_erpm_preview':r_erpm,
            'steering_servo_installed':False,
            'firmware_source':'https://github.com/rofiqcp/hoverboard-vesc',
            'mode':'LIVE_EXTERNAL_MOTOR' if gate and self.transport else 'MONITOR_ONLY',
            'error':self.last_error,
            'warning':'Do not use independent VESC outputs concurrently with PX4 AUTO.MISSION'
        }
        msg=String();msg.data=json.dumps(state,separators=(',',':'))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args);node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.transport is not None:
            try:node.transport.set_rpm(0,0);node.transport.close()
            except Exception:pass
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
