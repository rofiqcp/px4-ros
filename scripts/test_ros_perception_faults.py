#!/usr/bin/env python3
"""Standalone ROS2 perception fault injection. No serial or MAVLink connection."""
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from pixhawk_interfaces.msg import AvoidanceAdvisory

class Fixture(Node):
    def __init__(self):
        super().__init__('perception_fault_injection')
        self.tx=self.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
        self.out=[]
        self.diag=[]
        self.sub=self.create_subscription(AvoidanceAdvisory,'/pixhawk/avoidance/advisory',self.out.append,10)
        self.monitor=self.create_subscription(String,'/pixhawk/avoidance/diagnostics',self.diag.append,10)
        self.tf=StaticTransformBroadcaster(self)
        t=TransformStamped()
        t.header.frame_id='base_link';t.child_frame_id='livox_frame'
        t.header.stamp=self.get_clock().now().to_msg()
        t.transform.rotation.w=1.0
        self.tf.sendTransform(t)
        self.last_stamp=None
    def spin(self,seconds=.2):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.025)
    def cloud(self,scenario='clear',same_stamp=False):
        pts=[]
        if scenario not in ('ground','blind','malformed'):
            pts.extend((4.5,(i%30-15)*.025,.6) for i in range(360))
        if scenario in ('ground','blind'):
            pts.extend((4.5,(i%30-15)*.025,0.0) for i in range(360))
        if scenario in ('stop','unknown','left','right'):
            pts.extend((1.65+(i%5)*.02,(i%16-8)*.04,.6) for i in range(160))
        if scenario=='stop':
            pts.extend((.65,(i%15-7)*.025,.6) for i in range(100))
        if scenario=='left':
            pts.extend((1.6,2.1+(i%5)*.03,.6) for i in range(80))
            pts.extend((1.6,-.9-(i%4)*.025,.6) for i in range(60))
        if scenario=='right':
            pts.extend((1.6,-2.1-(i%5)*.03,.6) for i in range(80))
            pts.extend((1.6,.9+(i%4)*.025,.6) for i in range(60))
        m=PointCloud2()
        m.header.stamp=(self.last_stamp if same_stamp and self.last_stamp
                        else self.get_clock().now().to_msg())
        self.last_stamp=m.header.stamp
        m.header.frame_id='livox_frame'
        m.height=1;m.width=len(pts)
        m.fields=[PointField(name=x,offset=idx*4,datatype=PointField.FLOAT32,count=1)
                  for idx,x in enumerate(('x','y','z'))]
        m.point_step=12;m.row_step=12*len(pts)
        m.is_dense=True
        m.data=b''.join(struct.pack('<fff',*pt) for pt in pts)
        if scenario=='malformed':
            m.width=360;m.height=1;m.row_step=4320;m.data=b'bad'
        self.tx.publish(m)
    def get(self,reason,timeout=.8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.025)
            if self.diag and reason in self.diag[-1].data:return self.out[-1]
        raise AssertionError('expected '+reason+' last '+self.diag[-1].data if self.diag else reason)
    def sequence(self,scenario,count=15):
        for i in range(count):
            self.cloud(scenario)
            self.spin(.065)
        assert self.out,'No advisory for '+scenario
        return self.out[-1]

def main():
    rclpy.init()
    f=Fixture()
    passed=[]
    try:
        f.spin(.8)
        r=f.get('CLOUD_STALE')
        assert r.stop and not r.valid and r.source_scan_seq==0
        passed.append('STARTUP_STOP')
        r=f.sequence('clear')
        assert r.valid and not r.stop and not r.obstacle and r.source_scan_seq>0,r
        f.spin(.09)  # Let ROS receive all pending messages before sampling stable scan ID.
        seq=f.out[-1].source_scan_seq
        assert seq>0
        f.spin(.12)
        assert f.out[-1].source_scan_seq==seq,'Timer produced fictitious new LiDAR scan'
        passed.append('CLEAR_AND_DISTINCT_SCAN_ID')
        f.cloud('clear',same_stamp=True)
        r=f.get('CLOUD_NONMONOTONIC')
        assert r.stop and not r.valid and r.source_scan_seq==0
        passed.append('DUPLICATE_STAMP_BLOCKED')
        r=f.sequence('clear')
        assert r.valid and not r.stop
        f.cloud('malformed')
        r=f.get('MALFORMED_CLOUD')
        assert r.stop and not r.valid
        passed.append('MALFORMED_BLOCKED')
        r=f.sequence('ground')
        assert r.stop and not r.valid
        passed.append('GROUND_REJECTION')
        r=f.sequence('blind')
        assert r.stop and not r.valid
        passed.append('FRONT_UNKNOWN')
        r=f.sequence('unknown')
        assert r.valid and r.stop and r.vx_limit_mps==0 and abs(r.yaw_correction_radps)<1e-6
        passed.append('UNKNOWN_SIDE_NO_AUTO_TURN')
        r=f.sequence('left')
        assert r.valid and r.stop and r.vx_limit_mps==0 and r.yaw_correction_radps>0
        passed.append('LEFT_ADVISORY_STOP_ONLY')
        r=f.sequence('right')
        assert r.valid and r.stop and r.vx_limit_mps==0 and r.yaw_correction_radps<0
        passed.append('RIGHT_ADVISORY_STOP_ONLY')
        r=f.sequence('stop')
        assert r.valid and r.stop and r.vx_limit_mps==0
        passed.append('FRONT_STOP')
        f.spin(.8)
        r=f.get('CLOUD_STALE')
        assert r.stop and not r.valid and r.source_scan_seq==0
        passed.append('TIMEOUT_FAIL_CLOSED')
        print('ROS_PERCEPTION_FAULT_SUITE_PASS',len(passed),' '.join(passed),flush=True)
    finally:
        f.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
