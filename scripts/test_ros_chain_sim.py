#!/usr/bin/env python3
"""True ROS2 topic-level perception -> supervisor fixture in isolated ROS_DOMAIN_ID.
Does not use MAVLink, serial, RC hardware, PX4 parameters, or actuators.
"""
import json
import time
import rclpy
from std_msgs.msg import String
from test_ros_perception_faults import Fixture


def main():
    rclpy.init()
    f=Fixture()
    status=[]
    f.stock=f.create_subscription(String,'/pixhawk/stock/status',
                                  lambda m: status.append(json.loads(m.data)),10)
    f.inject=f.create_publisher(String,'/pixhawk/stock/sim_input',10)
    sim_state=dict(rc_known=True,manual_rc=False,mission_auto=True,
       mav_connected=True,estimator_valid=True,imu_valid=True,
       steering_feedback_valid=True,calibration_verified=True,speed_mps=.1,
       simulate_synthetic_scan=False)
    def publish_input():
        msg=String();msg.data=json.dumps(sim_state);f.inject.publish(msg)
    f.sim_timer=f.create_timer(.12,publish_input)
    scenario={'value':'clear'}
    f.cloud_timer=f.create_timer(.09,lambda:f.cloud(scenario['value']))
    def feed(kind,count=14):
        scenario['value']=kind
        f.sequence(kind,count=count)
    def input_state(**kwargs):
        sim_state.update(kwargs)
        publish_input()
        f.spin(.2)
    def require(name,expected,limit=1.2):
        until=time.monotonic()+limit
        while time.monotonic()<until:
            f.spin(.08)
            if status and status[-1]['state']==expected:
                s=status[-1]
                assert s['hardware_command_enabled'] is False
                assert s['stock_firmware_interface']=='UNPROVEN'
                print(name,expected,s.get('reason'),flush=True)
                return
        raise AssertionError((name,expected,status[-1] if status else None,
                              f.diag[-1].data if f.diag else None))
    try:
        f.spin(.7)
        input_state()
        feed('clear')
        require('BOOT_LATCH','LATCHED')
        feed('clear',count=10)
        input_state(operator_ack=True)
        feed('clear')
        require('OBSERVED_CLEAR_WITH_ACK','CLEAR')
        input_state(operator_ack=False)
        feed('unknown')
        require('UPSTREAM_PERCEPTION_STOP','STOP')
        feed('clear')
        f.spin(.35)
        guard=status[-1]
        assert guard['stop'] and guard['hazard_latched'] and guard['state'] in ('LATCHED','UNKNOWN'),guard
        print('NO_AUTO_RESUME_AFTER_OBSTACLE',guard['state'],guard['reason'],flush=True)
        feed('clear',count=12)
        input_state(operator_ack=True)
        feed('clear')
        require('OPERATOR_ACK_PLUS_DISTINCT_SCANS','CLEAR')
        input_state(manual_rc=True)
        require('MANUAL_RC_PRIORITY','MANUAL_RC')
        input_state(manual_rc=False,mav_connected=False)
        require('MINIPC_MAVLINK_LOSS','UNKNOWN')
        print('ROS_PERCEPTION_SUPERVISOR_CHAIN_PASS',flush=True)
    finally:
        f.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
