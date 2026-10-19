#!/usr/bin/env python3
"""Read-only end-to-end bench check; intentionally tests that motion is blocked."""
import json
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from pixhawk_interfaces.srv import ParamGet, ParamSet, GotoLocal, CommandLong

class Smoke(Node):
    def __init__(self):
        super().__init__('pixhawk_smoke_test')
        self.status = None
        self.imu_count = 0
        self.create_subscription(String, '/pixhawk/status', self.on_status, 10)
        self.create_subscription(Imu, '/pixhawk/imu', self.on_imu, qos_profile_sensor_data)

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_imu(self, msg):
        self.imu_count += 1

def call(node, path, cls, req, seconds=6):
    client=node.create_client(cls,path)
    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError(path + ' service unavailable')
    future=client.call_async(req)
    rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
    if not future.done():
        raise RuntimeError(path + ' timeout')
    value=future.result()
    if value is None:
        raise RuntimeError(path + ' returned error ' + repr(future.exception()))
    return value

def main():
    rclpy.init()
    n=Smoke()
    try:
        deadline=time.monotonic()+10
        while (not n.status or n.imu_count < 5) and time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.2)
        print('STATUS', json.dumps(n.status, sort_keys=True))
        print('IMU_COUNT',n.imu_count)
        if not n.status or not n.status['connected'] or n.imu_count<5:
            raise RuntimeError('No healthy MAVLink heartbeat and IMU telemetry')
        p=ParamGet.Request();p.name='SYS_AUTOSTART'
        pr=call(n,'/pixhawk/param_get',ParamGet,p)
        print('PARAM_GET',pr.success,pr.mav_type,pr.value,pr.message)
        if not pr.success:
            raise RuntimeError('PARAM_GET failed')
        q=GotoLocal.Request();q.x=1.;q.y=0.;q.z=0.;q.yaw=0.;q.duration_sec=3.
        qr=call(n,'/pixhawk/goto_local',GotoLocal,q)
        print('MOTION_GUARD',qr.success,qr.message)
        if qr.success:
            raise RuntimeError('SAFETY FAILURE: goto accepted with default disabled controls')
        write=ParamSet.Request();write.name='TEST_GAIN';write.mav_type=9;write.value=2.0
        wr=call(n,'/pixhawk/param_set',ParamSet,write)
        print('PARAM_WRITE_GUARD',wr.success,wr.message)
        if wr.success: raise RuntimeError('SAFETY FAILURE: writes enabled by default')
        arm=CommandLong.Request();arm.command=400;arm.params=[1.,0.,0.,0.,0.,0.,0.]
        ar=call(n,'/pixhawk/command_long',CommandLong,arm)
        print('ARM_GUARD',ar.success,ar.message)
        if ar.success: raise RuntimeError('SAFETY FAILURE: arming enabled by default')
        c=CommandLong.Request();c.command=512;c.params=[148.,0.,0.,0.,0.,0.,0.]
        cr=call(n,'/pixhawk/command_long',CommandLong,c)
        print('READ_ONLY_COMMAND',cr.success,cr.result,cr.message)
        if not cr.success:
            raise RuntimeError('Read-only MAV_CMD_REQUEST_MESSAGE was not acknowledged')
        print('SMOKE_TEST_PASS')
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    try: main()
    except Exception as e:
        print('SMOKE_TEST_FAIL',repr(e))
        sys.exit(1)
