#!/usr/bin/env python3
"""Software-only ROS2 fixture. Sends fake MID360 clouds, NEVER a MAVLink command."""
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import PointCloud2,PointField
from pixhawk_interfaces.msg import AvoidanceAdvisory

class Test(Node):
    def __init__(self):
        super().__init__('test_avoidance_perception')
        self.pub=self.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
        self.sub=self.create_subscription(AvoidanceAdvisory,'/pixhawk/avoidance/advisory',self.on_msg,10)
        self.out=[]
        self.tf=StaticTransformBroadcaster(self)
        t=TransformStamped()
        t.header.frame_id='base_link'
        t.child_frame_id='livox_frame'
        t.transform.rotation.w=1.0
        t.header.stamp=self.get_clock().now().to_msg()
        self.tf.sendTransform(t)
    def on_msg(self,m):
        self.out.append(m)
    def send(self,scenario):
        pts=[]
        for i in range(360):
            xx=4.6
            yy=(i%60-30)*.06
            zz=.6 + (i%5)*.05
            pts.append((xx,yy,zz))
        if scenario=='avoid':
            for i in range(180):
                xx=2.1 + (i%8)*.02
                yy=(i%12-6)*.06
                pts.append((xx,yy,.65))
            for i in range(120):
                pts.append((1.7, -1.0-(i%10)*.02,.7))
        if scenario=='stop':
            for i in range(180):
                pts.append((.75,(i%12-6)*.04,.6))
        cloud=PointCloud2()
        cloud.header.stamp=self.get_clock().now().to_msg()
        cloud.header.frame_id='livox_frame'
        cloud.height=1
        cloud.width=len(pts)
        cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])]
        cloud.point_step=12;cloud.row_step=12*len(pts)
        cloud.is_dense=True
        cloud.data=b''.join(struct.pack('<fff',*p) for p in pts)
        self.pub.publish(cloud)
def spin(node,duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
def run():
    rclpy.init()
    node=Test()
    try:
        spin(node,.8)
        for scenario in ['clear','avoid','stop']:
            for _ in range(14):
                node.send(scenario);spin(node,.06)
            latest=node.out[-1] if node.out else None
            print(scenario, {'valid':latest.valid,'obstacle':latest.obstacle,
                               'stop':latest.stop,'vx':latest.vx_limit_mps,
                               'yaw':latest.yaw_correction_radps} if latest else None,flush=True)
            assert latest and latest.valid
            if scenario=='clear': assert not latest.obstacle and not latest.stop and latest.source_scan_seq>0
            if scenario=='avoid': assert latest.obstacle and latest.stop and latest.vx_limit_mps==0
            if scenario=='stop': assert latest.stop and latest.vx_limit_mps==0
        spin(node,.9)
        latest=node.out[-1]
        print('cloud_stale',latest.valid,latest.obstacle,latest.stop,flush=True)
        assert latest.stop and not latest.valid and latest.source_scan_seq==0
        print('PASS perception scenario clear/avoid/stop/cloud stale',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':run()
