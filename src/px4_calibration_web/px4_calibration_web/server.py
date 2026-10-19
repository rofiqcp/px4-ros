"""Local-only ROS 2 PX4 calibration/readiness dashboard.

Only PARAM_REQUEST_READ and MAV_CMD_RUN_PREARM_CHECKS are used by default.
No arm/disarm, mission writes or param-set endpoints exist in this server.
Sensor calibration requires *two* explicit opt-in flags, detached motors,
a disarmed PX4 and an operator confirmation; it is disabled by default.
"""
import json
import ipaddress
import math
import queue
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix, BatteryState, MagneticField, FluidPressure
from pixhawk_interfaces.srv import ParamGet, CommandLong

from .readiness import PARAM_GROUPS, PARAMS, CALIBRATIONS, blockers

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('px4_calibration_web')
        self.declare_parameter('address','127.0.0.1')
        self.declare_parameter('port',8093)
        self.declare_parameter('enable_sensor_calibration',False)
        self.declare_parameter('refresh_interval_s',30.)
        self.lock=threading.RLock()
        self.status={}; self.status_time=0.
        self.sensors={}; self.params={}; self.events=deque(maxlen=80)
        self.last_action={'kind':'none','state':'idle','message':'Belum ada permintaan'}
        self.action_busy=False
        self.param_index=0;self.param_future=None;self.param_sent_at=0.
        self.epoch=time.monotonic()
        self.action_requests=queue.Queue(maxsize=4)
        self.param_client=self.create_client(ParamGet,'/pixhawk/param_get')
        self.command_client=self.create_client(CommandLong,'/pixhawk/command_long')
        self.create_subscription(String,'/pixhawk/status',self.on_status,10)
        self.create_subscription(String,'/pixhawk/events',self.on_event,20)
        self.create_subscription(String,'/pixhawk/firmware/version',self.on_version,10)
        self.create_subscription(String,'/px4_mission/status',self.on_mission,10)
        self.create_subscription(String,'/vesc_ackermann/diagnostics',self.on_vesc,10)
        self.create_subscription(Imu,'/pixhawk/imu',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(NavSatFix,'/pixhawk/gps/fix',self.on_gps,qos_profile_sensor_data)
        self.create_subscription(BatteryState,'/pixhawk/battery',self.on_battery,qos_profile_sensor_data)
        self.create_subscription(MagneticField,'/pixhawk/mag',self.on_magnetic,qos_profile_sensor_data)
        self.create_subscription(FluidPressure,'/pixhawk/barometer',self.on_pressure,qos_profile_sensor_data)
        self.create_subscription(String,'/pixhawk/sensors/health',self.on_health,10)
        self.create_subscription(String,'/pixhawk/estimator/status',self.on_estimator,10)
        self.create_timer(0.35,self.tick)

    def set_sensor(self,key,obj):
        with self.lock:self.sensors[key]={'value':obj,'received_at':time.monotonic()}
    def on_status(self,m):
        try:d=json.loads(m.data)
        except ValueError:return
        if not isinstance(d,dict):return
        with self.lock:self.status=d;self.status_time=time.monotonic()
    def on_event(self,m):
        with self.lock:self.events.appendleft({'at':time.time(),'message':m.data[:400]})
    def on_version(self,m):self.set_sensor('firmware',m.data)
    def on_mission(self,m):
        try:self.set_sensor('mission',json.loads(m.data))
        except ValueError:pass
    def on_vesc(self,m):
        try:self.set_sensor('vesc',json.loads(m.data))
        except ValueError:pass
    def on_imu(self,m):
        q=m.orientation
        v=m.angular_velocity;a=m.linear_acceleration
        self.set_sensor('imu',{'angular_velocity':[v.x,v.y,v.z],
            'acceleration':[a.x,a.y,a.z],'orientation':[q.x,q.y,q.z,q.w]})
    def on_gps(self,m):
        self.set_sensor('gps',{'fix_type_ros':int(m.status.status),
            'latitude':m.latitude,'longitude':m.longitude,'altitude':m.altitude})
    def on_battery(self,m):
        self.set_sensor('battery',{'voltage':m.voltage,'current':m.current,
            'percentage':m.percentage})
    def on_magnetic(self,m):
        v=m.magnetic_field
        self.set_sensor('mag',{'tesla_xyz':[v.x,v.y,v.z]})
    def on_pressure(self,m):
        self.set_sensor('barometer',{'pressure_pa':m.fluid_pressure,
            'variance':m.variance})
    def on_health(self,m):
        self.set_sensor('sensor_health',m.data)
    def on_estimator(self,m):
        self.set_sensor('estimator_status',m.data)
    def refresh_all(self):
        with self.lock:self.param_index=0;self.epoch=0.

    def tick(self):
        now=time.monotonic()
        if self.status_time and now-self.status_time>4:
            # Do not accidentally show a previously connected FCU as current.
            pass
        while not self.action_requests.empty() and not self.action_busy:
            try:kind,arg=self.action_requests.get_nowait()
            except queue.Empty:break
            if kind=='refresh':
                self.refresh_all();continue
            if not self.command_client.service_is_ready():
                with self.lock:self.last_action={'kind':kind,'state':'unavailable','message':'MAVLink command service belum siap'}
                continue
            req=CommandLong.Request()
            req.command=401 if kind=='prearm' else 241
            req.params=[0.]*7 if kind=='prearm' else list(CALIBRATIONS[arg]['params'])
            self.action_busy=True
            with self.lock:self.last_action={'kind':arg if kind=='calibration' else kind,
                'state':'running','message':'Meminta COMMAND_ACK dari PX4'}
            future=self.command_client.call_async(req)
            future.add_done_callback(lambda f,k=kind: self.done_command(f,k))
        if self.param_future is not None or not self.param_client.service_is_ready():
            return
        interval=max(15.,float(self.get_parameter('refresh_interval_s').value))
        if self.param_index>=len(PARAMS):
            if now-self.epoch<interval:return
            self.param_index=0
        name=PARAMS[self.param_index];self.param_index+=1
        req=ParamGet.Request();req.name=name
        self.param_future=self.param_client.call_async(req)
        self.param_sent_at=now
        self.param_future.add_done_callback(lambda f,n=name:self.done_param(f,n))
        if self.param_index>=len(PARAMS):self.epoch=now

    def done_param(self,future,name):
        try:
            result=future.result()
            record={'available':bool(result.success),'value':float(result.value) if result.success else None,
                    'mav_type':int(result.mav_type),'message':result.message,
                    'observed_at':time.time()}
        except Exception as exc:
            record={'available':False,'value':None,'message':str(exc)[:150],'observed_at':time.time()}
        with self.lock:self.params[name]=record
        self.param_future=None

    def done_command(self,future,kind):
        try:
            result=future.result()
            state='accepted' if result.success else 'rejected'
            detail=str(result.message)
            if kind=='prearm' and result.success:
                detail+='; hanya permintaan pemeriksaan, bukan izin ARM'
            record={'kind':kind,'state':state,'message':detail,
                    'mav_result':int(result.result),'observed_at':time.time()}
        except Exception as exc:
            record={'kind':kind,'state':'error','message':str(exc)[:180],'observed_at':time.time()}
        with self.lock:self.last_action=record
        self.action_busy=False

    def enqueue_action(self,kind,arg=None):
        with self.lock:
            connected=self.status.get('connected') is True and time.monotonic()-self.status_time<3
            armed=self.status.get('armed') is True
            if not connected:return False,'PX4 belum terhubung'
            if armed:return False,'PX4 ARMED: tindakan diagnostik/kalibrasi ditolak'
            if kind=='calibration' and not bool(self.get_parameter('enable_sensor_calibration').value):
                return False,'Sensor calibration disabled until explicit guarded launch'
            if self.action_busy or self.action_requests.qsize():
                return False,'Permintaan sebelumnya masih berlangsung'
            if kind=='calibration' and arg not in CALIBRATIONS:return False,'Kalibrasi tidak dikenal'
            try:self.action_requests.put_nowait((kind,arg))
            except queue.Full:return False,'Antrean penuh'
        return True,'Perintah diagnostik ditambahkan ke antrean'

    def snapshot(self):
        with self.lock:
            status=dict(self.status); params=dict(self.params)
            sensor=dict(self.sensors);events=list(self.events)
            action=dict(self.last_action);age=time.monotonic()-self.status_time
        if age>3:status['connected']=False
        normalized_sensors={}
        for name,detail in sensor.items():
            d=dict(detail);d['age_s']=round(time.monotonic()-d.pop('received_at'),2)
            normalized_sensors[name]=d
        return {'status':status,'status_age_s':round(age,2),
          'parameters':params,'parameter_groups':{k:list(v) for k,v in PARAM_GROUPS.items()},
          'calibrations':CALIBRATIONS,'sensor':normalized_sensors,'events':events,
          'blockers':[{'code':a,'message':b} for a,b in blockers(status,params)],
          'action':action,
          'calibration_enabled':bool(self.get_parameter('enable_sensor_calibration').value),
          'manual_control_source':'ROS/MAVLink planned; RC telemetry not required by preview',
          'arm_endpoint_available':False,
          'notice':'Web adalah dashboard diagnosis. Tidak ada API untuk ARM, force-arm atau bypass failsafe.'}

def handler_for(node,html):
    class Handler(BaseHTTPRequestHandler):
        server_version='PX4CalibrationConsole/1.0'
        def log_message(self,format,*args):
            # Avoid high-volume polling logs.
            if self.path!='/api/status':
                node.get_logger().info('%s - %s' % (self.address_string(),format%args))
        def headers_safe(self,length,content_type):
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(length))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('X-Frame-Options','DENY')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
        def respond(self,code,data):
            def normalize(item):
                if isinstance(item,float):
                    return item if math.isfinite(item) else None
                if isinstance(item,dict):
                    return {k:normalize(v) for k,v in item.items()}
                if isinstance(item,(tuple,list)):
                    return [normalize(v) for v in item]
                return item
            body=json.dumps(normalize(data),allow_nan=False).encode('utf-8')
            self.send_response(code);self.headers_safe(len(body),'application/json; charset=utf-8')
            self.wfile.write(body)
        def do_GET(self):
            if self.path in ('/','/index.html'):
                self.send_response(200);self.headers_safe(len(html),'text/html; charset=utf-8')
                self.wfile.write(html);return
            if self.path=='/api/status':return self.respond(200,node.snapshot())
            return self.respond(404,{'error':'Not found'})
        def do_OPTIONS(self):self.respond(405,{'error':'Cross-site requests prohibited'})
        def do_POST(self):
            if self.headers.get('X-PX4-Action')!='local-operator':
                return self.respond(403,{'error':'Missing operator action header'})
            origin=self.headers.get('Origin','')
            host=self.headers.get('Host','')
            if origin and origin not in (f'http://{host}',f'https://{host}'):
                return self.respond(403,{'error':'Cross-origin action rejected'})
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':
                return self.respond(415,{'error':'application/json required'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if size<1 or size>4096:raise ValueError('Request size invalid')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise ValueError('JSON object required')
            except (ValueError,TypeError) as exc:return self.respond(400,{'error':str(exc)})
            if self.path=='/api/refresh':
                ok,msg=node.enqueue_action('refresh');return self.respond(202 if ok else 409,{'ok':ok,'message':msg})
            if self.path=='/api/prearm':
                ok,msg=node.enqueue_action('prearm');return self.respond(202 if ok else 409,{'ok':ok,'message':msg})
            if self.path=='/api/calibrate':
                if data.get('confirm')!='MOTORS_ISOLATED':
                    return self.respond(403,{'error':'Konfirmasi isolasi motor belum diberikan'})
                ok,msg=node.enqueue_action('calibration',data.get('kind'))
                return self.respond(202 if ok else 403,{'ok':ok,'message':msg})
            return self.respond(404,{'error':'Not found'})
    return Handler

def main(args=None):
    rclpy.init(args=args)
    node=CalibrationNode();address=str(node.get_parameter('address').value)
    port=int(node.get_parameter('port').value)
    if not ipaddress.ip_address(address).is_loopback:
        raise RuntimeError('Only loopback binding permitted: use SSH tunnel for remote access')
    web=Path(get_package_share_directory('px4_calibration_web'))/'webui/index.html'
    content=web.read_bytes()
    server=ThreadingHTTPServer((address,port),handler_for(node,content))
    server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    node.get_logger().info(f'PX4 read-only calibration web: http://{address}:{port}')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        server.shutdown();server.server_close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
