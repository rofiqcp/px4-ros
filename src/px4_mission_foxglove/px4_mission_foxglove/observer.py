"""Read-only MissionDownload over existing MAVLink USB bridge; no mission writes."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, TwistStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from pixhawk_interfaces.msg import AvoidanceAdvisory
from pixhawk_interfaces.srv import MissionDownload
from .math_utils import waypoint, xy, preview

class Observer(Node):
    def __init__(self):
        super().__init__('mission_observer')
        for name, value in [('mission_refresh_s',30.),('status_timeout_s',2.5),
            ('attitude_timeout_s',1.5),('advisory_timeout_s',.35),
            ('cruise_speed_mps',.35),('max_yaw_rate_radps',.25),
            ('heading_kp',.6),('mission_arrival_tolerance_m',1.5)]:
            self.declare_parameter(name,value)
        self.status={};self.status_time=0.;self.items=[]
        self.seq=None;self.last_fetch=-1e9;self.pending=None
        self.gps=None;self.gps_time=0.;self.yaw=None;self.yaw_time=0.
        self.advisory=None;self.advisory_time=0.
        self.path_pub=self.create_publisher(Path,'/px4_mission/path',10)
        self.marker_pub=self.create_publisher(MarkerArray,'/px4_mission/markers',10)
        self.status_pub=self.create_publisher(String,'/px4_mission/status',10)
        self.twist_pub=self.create_publisher(TwistStamped,'/px4_mission/preview_twist',10)
        self.create_subscription(String,'/pixhawk/status',self.recv_status,10)
        self.create_subscription(String,'/pixhawk/mission/events',self.recv_event,10)
        self.create_subscription(NavSatFix,'/pixhawk/gps/fix',self.recv_gps,qos_profile_sensor_data)
        self.create_subscription(QuaternionStamped,'/pixhawk/attitude',self.recv_attitude,qos_profile_sensor_data)
        self.create_subscription(AvoidanceAdvisory,'/pixhawk/stock/advisory',self.recv_advisory,10)
        self.mission_client=self.create_client(MissionDownload,'/pixhawk/mission_download')
        self.create_timer(.2,self.tick)
        self.get_logger().info('Mission observer ready: PX4 MissionDownload READ ONLY; previews never forwarded to motor')

    def param(self,name): return self.get_parameter(name).value
    def recv_status(self,m):
        try:
            value=json.loads(m.data)
            if isinstance(value,dict):
                self.status=value;self.status_time=time.monotonic()
        except (ValueError,TypeError):
            self.status={};self.status_time=0.
    def recv_event(self,m):
        try: value=json.loads(m.data)
        except (ValueError,TypeError):return
        if isinstance(value,dict) and value.get('type')=='MISSION_CURRENT':
            seq=value.get('seq')
            if type(seq) is int and 0<=seq<=1000 and seq!=self.seq:
                self.seq=seq
                if self.items:
                    self.publish_waypoints()
    def recv_gps(self,m):
        if m.status.status>=0 and all(math.isfinite(v) for v in (m.latitude,m.longitude)):
            if -90<=m.latitude<=90 and -180<=m.longitude<=180:
                self.gps=(m.latitude,m.longitude);self.gps_time=time.monotonic()
    def recv_attitude(self,m):
        q=m.quaternion
        values=(q.w,q.x,q.y,q.z)
        if all(math.isfinite(v) for v in values) and abs(sum(x*x for x in values)-1.)<.2:
            self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.yaw_time=time.monotonic()
    def recv_advisory(self,m):
        values=(m.vx_limit_mps,m.yaw_correction_radps,m.front_clearance_m,
                m.left_clearance_m,m.right_clearance_m)
        if all(math.isfinite(v) for v in values):
            self.advisory=m;self.advisory_time=time.monotonic()
    def downloaded(self,f):
        self.pending=None
        try:
            result=f.result()
            if result and result.success:
                self.items=list(result.items)
                self.publish_waypoints()
            else:self.get_logger().warn('Mission download: '+str(getattr(result,'message','unavailable')))
        except Exception as exc:self.get_logger().warn('Mission download failed: '+str(exc))
    def publish_waypoints(self):
        points=[m for m in self.items if waypoint(m)]
        path=Path();path.header.stamp=self.get_clock().now().to_msg()
        path.header.frame_id='px4_mission_preview'
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL
        markers.markers.append(clear)
        if points:
            lat0,lon0=points[0].x/1e7,points[0].y/1e7
            for item in points:
                x,y=xy(item.x/1e7,item.y/1e7,lat0,lon0)
                p=PoseStamped();p.header=path.header;p.pose.position.x=x
                p.pose.position.y=y;p.pose.orientation.w=1.
                path.poses.append(p)
                marker=Marker();marker.header=path.header;marker.id=int(item.seq)
                marker.ns='px4_waypoints';marker.action=Marker.ADD;marker.type=Marker.SPHERE
                marker.pose=p.pose;marker.scale.x=marker.scale.y=marker.scale.z=.6
                marker.color.a=1.;marker.color.g=1. if item.seq==self.seq else .3
                marker.color.b=.1 if item.seq==self.seq else 1.
                markers.markers.append(marker)
        self.path_pub.publish(path);self.marker_pub.publish(markers)
    def tick(self):
        now=time.monotonic()
        live=now-self.status_time<max(.5,self.param('status_timeout_s')) and self.status.get('connected') is True
        mission=live and self.status.get('px4_main_mode')==4 and self.status.get('px4_sub_mode')==4
        if live and self.pending is None and self.mission_client.service_is_ready():
            interval=5. if not self.items else max(10.,self.param('mission_refresh_s'))
            if now-self.last_fetch>=interval:
                req=MissionDownload.Request();req.mission_type=0
                self.pending=self.mission_client.call_async(req)
                self.pending.add_done_callback(self.downloaded)
                self.last_fetch=now
        v=w=0.;reason='PX4_LINK_STALE'
        valid_advisory=self.advisory is not None and now-self.advisory_time<self.param('advisory_timeout_s')
        valid_pose=(self.gps is not None and self.yaw is not None and
                    now-self.gps_time<self.param('attitude_timeout_s') and
                    now-self.yaw_time<self.param('attitude_timeout_s'))
        target=next((x for x in self.items if x.seq==self.seq and waypoint(x)),None)
        if live:
            if not mission:reason='PX4_NOT_AUTO_MISSION'
            elif self.status.get('manual_takeover_latched'):
                reason='MANUAL_TAKEOVER_ACTIVE'
            elif not self.status.get('airframe_configured') or not self.status.get('gps_valid'):
                reason='PX4_AIRFRAME_OR_GPS_INVALID'
            elif not valid_pose:reason='GPS_OR_HEADING_STALE'
            elif not valid_advisory or not self.advisory.valid:reason='WALL_ADVISORY_INVALID'
            elif self.advisory.stop:reason='OBSTACLE_STOP'
            elif target is None:reason='ACTIVE_WAYPOINT_UNAVAILABLE'
            else:
                pos=xy(self.gps[0],self.gps[1],target.x/1e7,target.y/1e7)
                v,w,reason=preview(pos,(0.,0.),self.yaw,self.advisory.vx_limit_mps,
                    self.advisory.yaw_correction_radps,self.param('cruise_speed_mps'),
                    self.param('max_yaw_rate_radps'),self.param('heading_kp'),
                    self.param('mission_arrival_tolerance_m'))
        twist=TwistStamped();twist.header.stamp=self.get_clock().now().to_msg()
        twist.header.frame_id='base_link';twist.twist.linear.x=v;twist.twist.angular.z=w
        self.twist_pub.publish(twist)
        msg=String();msg.data=json.dumps({
          'px4_connected':live,'px4_auto_mission':mission,'active_mission_seq':self.seq,
          'downloaded_items':len(self.items),'source':'PX4 USB MAVLink MISSION_ITEM_INT',
          'advisory_fresh':valid_advisory,'position_fresh':valid_pose,
          'preview_speed_mps':v,'preview_yaw_radps':w,
          'motors_enabled':False,'px4_commands_sent':False,'reason':reason,
          'warning':'PX4 AUTO.MISSION cannot accept independent OFFBOARD steering commands'
        },separators=(',',':'))
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args);node=Observer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
