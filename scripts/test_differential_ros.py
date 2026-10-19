#!/usr/bin/env python3
"""Isolated ROS_DOMAIN_ID test of differential monitor. NO PX4/actuator output."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pixhawk_interfaces.msg import AvoidanceAdvisory

class Test(Node):
    def __init__(self):
        super().__init__('differential_fixture_readonly')
        self.advisory=self.create_publisher(AvoidanceAdvisory,'/pixhawk/stock/advisory',10)
        self.status=self.create_publisher(String,'/pixhawk/stock/status',10)
        self.latest=None
        self.subscription=self.create_subscription(String,'/pixhawk/differential/diagnostics',self.recv,10)
    def recv(self,msg):
        self.latest=json.loads(msg.data)
    def tick(self,allowed,stop,valid=True,seq=11,v=.2,yaw=.2,timeout=.6):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            a=AvoidanceAdvisory()
            a.valid=valid
            a.stop=stop
            a.source_scan_seq=seq
            a.vx_limit_mps=v
            a.yaw_correction_radps=yaw
            self.advisory.publish(a)
            s=String()
            s.data=json.dumps({'state':'CLEAR' if allowed else 'MANUAL_RC',
                'stop':not allowed,'hazard_latched':not allowed,
                'hardware_command_enabled':False})
            self.status.publish(s)
            rclpy.spin_once(self,timeout_sec=.03)
        assert self.latest is not None
        assert self.latest['hardware_command_enabled'] is False
        assert self.latest['ros_motion_publisher_enabled'] is False
        assert self.latest['stock_px4_interface_supported'] is False
        return self.latest

def main():
    rclpy.init()
    node=Test()
    try:
        end=time.monotonic()+2
        while node.latest is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert node.latest is not None
        assert node.latest['left_wheel_mps']==0
        assert node.latest['right_wheel_mps']==0
        print('STARTUP_ZERO_PASS')
        v=node.tick(True,False)
        assert v['preview_valid'] and abs(v['left_wheel_mps']-.14)<1e-4 and abs(v['right_wheel_mps']-.26)<1e-4,v
        print('SIM_DIFFERENTIAL_LEFT_RIGHT_PREVIEW_PASS')
        v=node.tick(True,True)
        assert v['left_wheel_mps']==0 and v['right_wheel_mps']==0 and not v['preview_valid']
        print('OBSTACLE_STOP_PASS')
        v=node.tick(False,False)
        assert not v['preview_valid'] and v['left_wheel_mps']==0
        print('RC_MANUAL_PRIORITY_PASS')
        v=node.tick(True,False,valid=False)
        assert not v['preview_valid']
        print('INVALID_ADVISORY_PASS')
        v=node.tick(True,False,seq=0)
        assert not v['preview_valid']
        print('STALE_OR_UNKNOWN_SCAN_PASS')
        v=node.tick(True,False,timeout=.45)
        assert v['preview_valid']
        finish=time.monotonic()+.55
        while time.monotonic()<finish:rclpy.spin_once(node,timeout_sec=.04)
        v=node.latest
        assert v['left_wheel_mps']==0 and v['right_wheel_mps']==0 and v['reason']=='SOURCE_TIMEOUT',v
        print('ROS_DISCONNECT_TIMEOUT_FAIL_CLOSED_PASS')
        print('DIFFERENTIAL_ROS_TOPIC_SUITE_PASS')
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':main()
