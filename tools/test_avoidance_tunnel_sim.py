#!/usr/bin/env python3
"""Isolated pseudo-tty, fake Pixhawk: no real hardware device opened."""
import os,pty,tty,select,struct,subprocess,threading,time,sys,zlib
os.environ['MAVLINK20']='1'
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from pixhawk_interfaces.msg import AvoidanceAdvisory
from std_msgs.msg import String
class Writer:
 def __init__(self,fd):self.fd=fd
 def write(self,b):return os.write(self.fd,b)
class FakePX4:
 def __init__(self,fd):
  self.fd=fd
  self.mav=mavutil.mavlink.MAVLink(Writer(fd),srcSystem=1,srcComponent=1)
  self.reader=mavutil.mavlink.MAVLink(None)
  self.reader.robust_parsing=True
  self.packets=[]
  self.stop=False
  self.thread=threading.Thread(target=self.loop,daemon=True)
 def loop(self):
  os.set_blocking(self.fd,False)
  last=0
  while not self.stop:
   now=time.monotonic()
   try:
    if time.monotonic()-last>.45:
     self.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
      mavutil.mavlink.MAV_AUTOPILOT_PX4,0,0,mavutil.mavlink.MAV_STATE_STANDBY)
     self.mav.mission_current_send(2,4)
     self.mav.servo_output_raw_send(int(time.monotonic()*1e6)&0xffffffff,0,1500,1600,1500,1500,1500,1500,1500,1500)
     self.mav.attitude_quaternion_send(int(now*1000)&0xffffffff,1.,0.,0.,0.,0.,0.,.12)
     self.mav.vfr_hud_send(0.,.52,45,0,0.,0.)
     last=time.monotonic()
    if select.select([self.fd],[],[],.015)[0]:
     for byte in os.read(self.fd,8000):
      m=self.reader.parse_char(bytes([byte]))
      if not m:continue
      if m.get_type()=='PARAM_REQUEST_READ':
       if m.param_id.rstrip('\x00')=='SYS_AUTOSTART':
        val=struct.unpack('<f',struct.pack('<I',4001))[0]
        self.mav.param_value_send(b'SYS_AUTOSTART',val,mavutil.mavlink.MAV_PARAM_TYPE_INT32,1,0)
      if m.get_type()=='TUNNEL':
       b=bytes(m.payload[:m.payload_length])
       if m.payload_type==42001 and len(b)==36:
        fields=struct.unpack('<IBBHIfffffI',b)
        assert fields[0]==0x31445641 and fields[1]==1
        assert fields[10]==(zlib.crc32(b[:32])&0xffffffff)
        self.packets.append(fields)
   except (BlockingIOError,InterruptedError):pass
   except OSError:break
def spin(n,sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
def main():
 master,slave=pty.openpty()
 tty.setraw(slave)
 dev=os.ttyname(slave)
 fake=FakePX4(master);fake.thread.start()
 env=os.environ.copy()
 bin=str(Path(get_package_prefix("pixhawk_bridge")) / "lib/pixhawk_bridge/pixhawk_node")
 args=[bin,'--ros-args','-r','__ns:=/pixhawk_sim',
  '-p','device:='+dev,'-p','enable_motion_commands:=false',
  '-p','enable_avoidance_advisory_tx:=true',
  '-p','avoidance_firmware_confirmed:=true']
 proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                       env=env,text=True)
 rclpy.init()
 node=Node('test_avoidance_tunnel_sim')
 pub=node.create_publisher(AvoidanceAdvisory,'/pixhawk_sim/avoidance/advisory',10)
 feedback=[];missions=[]
 subscriber=node.create_subscription(String,'/pixhawk_sim/rover/feedback',lambda m:feedback.append(m.data),10)
 sub_mission=node.create_subscription(String,'/pixhawk_sim/mission/events',lambda m:missions.append(m.data),10)
 try:
  for flags in ('clear','avoid','stop'):
   for _ in range(18):
    m=AvoidanceAdvisory()
    m.valid=True;m.obstacle=(flags!='clear');m.stop=(flags=='stop')
    m.vx_limit_mps=0.0 if m.stop else (.25 if m.obstacle else .7)
    m.yaw_correction_radps=.2 if m.obstacle else 0.
    m.front_clearance_m=.8 if m.stop else (2. if m.obstacle else 5.)
    m.left_clearance_m=1.5;m.right_clearance_m=.8
    pub.publish(m);spin(node,.065)
   print('SENT',flags,'TX_COUNT',len(fake.packets),flush=True)
  assert len(fake.packets)>=10,'No advisory MAVLink TUNNEL frames'
  flags=[p[2] for p in fake.packets]
  assert any(v&1 and not (v&2) for v in flags),'clear not transmitted'
  assert any(v&2 and not(v&4) for v in flags),'avoid not transmitted'
  assert any(v&4 for v in flags),'stop not transmitted'
  # In ROS FLU, +yaw is LEFT. Payload PX4 FRD/NED must be negative.
  assert all(p[6]<0 for p in fake.packets if (p[2]&2) and not (p[2]&4)), 'Yaw FLU to FRD conversion failed'
  sessions={p[4] for p in fake.packets}
  assert len(sessions)==1 and 0 not in sessions
  before=len(fake.packets);spin(node,.6)
  assert len(fake.packets)<=before+3,'Expired ROS command still sent'
  assert any('SERVO_OUTPUT_RAW' in s and 'servo2_pwm' in s for s in feedback),feedback[-8:]
  assert any('ATTITUDE_QUATERNION' in s and 'yaw_rate_flu_radps' in s for s in feedback),feedback[-8:]
  assert any('VFR_HUD' in s for s in feedback),feedback[-8:]
  assert any('MISSION_CURRENT' in s for s in missions),missions[-8:]
  print('PASS mission index, servo PWM (not measured steering), yaw rate, ground speed feedback',flush=True)
  print('PASS MAVLink2 TUNNEL CRC/session, clear/avoid/stop, expired-transmit-block',
        'packets=',len(fake.packets),flush=True)
 finally:
  fake.stop=True;proc.terminate()
  try:out,_=proc.communicate(timeout=4)
  except subprocess.TimeoutExpired:proc.kill();out,_=proc.communicate()
  print('BRIDGE_EXIT',proc.returncode,'LOG',out[-380:],flush=True)
  node.destroy_node();rclpy.shutdown()
  os.close(master);os.close(slave)
if __name__=='__main__':
 try:main()
 except Exception as e:print('FAIL',repr(e),flush=True);sys.exit(1)
