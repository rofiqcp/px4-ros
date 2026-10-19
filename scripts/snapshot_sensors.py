#!/usr/bin/env python3
"""Observe (not control) actual Pixhawk USB IMUs, EKF and auxiliary sensors."""
from pathlib import Path
from datetime import datetime,timezone
import json,statistics,time,math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,MagneticField,FluidPressure,NavSatFix,BatteryState
from std_msgs.msg import String
def stats(values):
    if not values:return {'count':0}
    return {'count':len(values),'mean':round(statistics.mean(values),6),
            'min':round(min(values),6),'max':round(max(values),6),
            'std':round(statistics.pstdev(values),6)}
class Recorder(Node):
    def __init__(self):
        super().__init__('pixhawk_sensors_snapshot')
        self.imu={k:{'acc_norm':[],'gyro_norm':[]} for k in ['primary','0','1','2']}
        self.events={}
        for label,path in [('primary','/pixhawk/imu'),('0','/pixhawk/imu/instance_0'),
                           ('1','/pixhawk/imu/instance_1'),('2','/pixhawk/imu/instance_2')]:
            self.create_subscription(Imu,path,lambda msg,key=label:self.on_imu(key,msg),qos_profile_sensor_data)
        for path in ['/pixhawk/estimator/status','/pixhawk/estimator/odometry_raw',
                     '/pixhawk/imu/availability','/pixhawk/sensors/health',
                     '/pixhawk/sensors/vibration','/pixhawk/status']:
            self.create_subscription(String,path,lambda msg,key=path:self.on_text(key,msg),10)
        self.pressure=[]
        self.create_subscription(FluidPressure,'/pixhawk/barometer',
            lambda m:self.pressure.append(m.fluid_pressure),qos_profile_sensor_data)
        self.mag=[]
        self.create_subscription(MagneticField,'/pixhawk/mag',
            lambda m:self.mag.append([m.magnetic_field.x,m.magnetic_field.y,m.magnetic_field.z]),
            qos_profile_sensor_data)
    def on_text(self,key,msg):
        try:self.events[key]=json.loads(msg.data)
        except: self.events[key]=msg.data
    def on_imu(self,key,msg):
        a=msg.linear_acceleration;g=msg.angular_velocity
        self.imu[key]['acc_norm'].append(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))
        self.imu[key]['gyro_norm'].append(math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z))
def main():
    rclpy.init();n=Recorder()
    try:
        end=time.monotonic()+10
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
        data={'datetime':datetime.now(timezone.utc).isoformat(),'period_seconds':10,
              'units':{'acc_norm':'m/s^2','gyro_norm':'rad/s','pressure':'Pa','mag':'Tesla'},
              'imu':{k:{axis:stats(samples) for axis,samples in v.items()} for k,v in n.imu.items()},
              'pressure_pa':stats(n.pressure),'magnetic_samples':len(n.mag),'latest':n.events}
        folder=(Path(__file__).resolve().parents[1] / 'backups')
        path=folder/('cube_sensor_snapshot_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(data,indent=2,allow_nan=False))
        print('SENSOR_SNAPSHOT',path,flush=True)
        for name,v in data['imu'].items():print('IMU',name,v,flush=True)
        print('EKF',data['latest'].get('/pixhawk/estimator/status'),flush=True)
        print('RAW_ODOM',data['latest'].get('/pixhawk/estimator/odometry_raw'),flush=True)
        print('PRESSURE',data['pressure_pa'],flush=True)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
