#!/usr/bin/env python3
"""Read-only hardware sensor telemetry request and availability audit."""
import json,time,pathlib
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from pixhawk_interfaces.srv import MessageInterval

IDS={26:"SCALED_IMU",116:"SCALED_IMU2",129:"SCALED_IMU3",230:"ESTIMATOR_STATUS",241:"VIBRATION"}
class Probe(Node):
    def __init__(self):
        super().__init__('pixhawk_sensor_probe')
        self.client=self.create_client(MessageInterval,'/pixhawk/set_message_interval')
        self.availability={}
        self.status=None
        self.estimator=[]
        self.imu_counts=[0,0,0]
        self.create_subscription(String,'/pixhawk/imu/availability',self.available,10)
        self.create_subscription(String,'/pixhawk/status',self.setstatus,10)
        self.create_subscription(String,'/pixhawk/estimator/status',lambda m:self.estimator.append(m.data),10)
        for i in range(3):
            self.create_subscription(Imu,f'/pixhawk/imu/instance_{i}',
                lambda m,index=i:self.bump(index),qos_profile_sensor_data)
    def bump(self,i): self.imu_counts[i]+=1
    def available(self,m):
        try:self.availability=json.loads(m.data)
        except:pass
    def setstatus(self,m):
        try:self.status=json.loads(m.data)
        except:pass
    def request(self,mid,rate):
        if not self.client.wait_for_service(timeout_sec=4):return 'service unavailable'
        req=MessageInterval.Request()
        req.message_id=mid
        req.interval_us=int(1000000/rate)
        future=self.client.call_async(req)
        deadline=time.monotonic()+6
        while time.monotonic()<deadline and not future.done():
            rclpy.spin_once(self,timeout_sec=.05)
        if not future.done() or future.result() is None:return 'timeout'
        r=future.result()
        return {'success':r.success,'reason':r.message}
def main():
    rclpy.init()
    p=Probe()
    try:
        responses={}
        for mid,name in IDS.items():
            responses[name]=p.request(mid,10 if name.startswith('SCALED_IMU') else 2)
            print('REQUEST',mid,name,responses[name],flush=True)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:rclpy.spin_once(p,timeout_sec=.1)
        out={'timestamp':datetime.now().isoformat(),
             'message_interval_ack':responses,
             'imu_stream_counts_during_probe':p.imu_counts,
             'imu_availability':p.availability,
             'estimator_last':p.estimator[-1:] or None,
             'status':p.status}
        path=(pathlib.Path(__file__).resolve().parents[1] / 'backups')/('sensor_probe_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(out,indent=2))
        print('SENSOR_RESULT',json.dumps(out,allow_nan=True),flush=True)
        print('PROBE_FILE',path,flush=True)
    finally:
        p.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
