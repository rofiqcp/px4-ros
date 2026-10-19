#!/usr/bin/env python3
"""Stock safety ROS2 integration fixture. Isolated ROS_DOMAIN_ID; zero MAVLink IO."""
import json,time,sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Test(Node):
 def __init__(self):
  super().__init__('stock_px4_sim_test')
  self.pub=self.create_publisher(String,'/pixhawk/stock/sim_input',10)
  self.sub=self.create_subscription(String,'/pixhawk/stock/status',self.recv,10)
  self.latest={}
 def recv(self,m):
  self.latest=json.loads(m.data)
 def send(self,**kw):
  m=String();m.data=json.dumps(kw);self.pub.publish(m)
 def tick(self,kw,n=1):
  for _ in range(n):
   self.send(**kw)
   until=time.monotonic()+0.18
   while time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.03)
  return self.latest
def check(d,states,title):
 print(title,d.get('state'),d.get('reason'),d.get('hardware_command_enabled'),flush=True)
 assert d.get('state') in states,(title,d)
 assert d.get('hardware_command_enabled') is False
def main():
 rclpy.init();n=Test()
 try:
  start=time.monotonic()
  while time.monotonic()-start<2:rclpy.spin_once(n,timeout_sec=.05)
  ok={'front_m':8.,'left_m':2.,'right_m':2.,'speed_mps':0.3}
  check(n.tick(ok,n=5),('LATCHED',),'BOOT_LATCH_NO_OPERATOR_ACK')
  check(n.tick(dict(ok,operator_ack=True),n=5),('CLEAR',),'ACK_AND_CLEAR_FRAMES')
  check(n.tick(dict(ok,front_m=.45),n=2),('STOP',),'FRONT_STOP')
  check(n.tick(ok,n=5),('LATCHED',),'HAZARD_PERSISTS_WITHOUT_ACK')
  check(n.tick(dict(ok,operator_ack=True),n=5),('CLEAR',),'LATCH_RESET')
  check(n.tick(dict(ok,manual_rc=True),n=1),('MANUAL_RC',),'RC_PRIORITY')
  check(n.tick(dict(ok,estop=True,manual_rc=True),n=1),('ESTOP',),'ESTOP_OVER_RC')
  check(n.tick(dict(ok,mav_connected=False),n=2),('UNKNOWN',),'MAV_LOSS')
  check(n.tick(ok,n=5),('LATCHED',),'HOTPLUG_NO_AUTORESUME')
  print('STOCK_ROS2_SIM_INTEGRATION_PASS',flush=True)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
