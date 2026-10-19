#!/usr/bin/env python3
"""PX4 USB integration probe: reads real FCU; tests default-DENIED motion APIs.
Absolutely NO arm, parameter set, mission upload to FC, or GOTO wire transmissions.
Requires all hardware write flags already false; refuses any potentially
dangerous service invocation if a flag is enabled.
"""
from datetime import datetime, timezone
import hashlib
import json
import pathlib
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from pixhawk_interfaces.srv import (
    MissionDownload, MissionUpload, ParamGet, GotoLocal, GotoGlobal,
    SetMode, CommandLong
)

ROOT=pathlib.Path(__file__).resolve().parents[1]
BACKUP=ROOT/'backups/cube_px4_20261008_213146_complete.json'
REPORT=ROOT/'results/cube_usb_mission_goto_readonly_20261009.json'
FLAGS=('enable_motion_commands','allow_parameter_writes',
       'allow_mode_changes','allow_mission_writes')

class Probe(Node):
    def __init__(self):
        super().__init__('cube_usb_mission_goto_readonly_probe')
        self.received=[]
        self.firmware=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._version_subscription=self.create_subscription(
            String,'/pixhawk/firmware/version',
            lambda msg:self.firmware.append(msg.data),qos)
        self.subscription=self.create_subscription(
            String,'/pixhawk/status',lambda m:self.received.append(json.loads(m.data)),10)
    def get_status(self,seconds=4):
        old=len(self.received)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.1)
            if len(self.received)>old:return self.received[-1]
        raise RuntimeError('No live Pixhawk heartbeat status')
    def call(self,name,cls,request,timeout=12):
        cli=self.create_client(cls,name)
        if not cli.wait_for_service(timeout_sec=3):
            raise RuntimeError(name+' service unavailable')
        future=cli.call_async(request)
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(name+' service timeout')
        result=future.result()
        return result
    def permissions(self):
        req=GetParameters.Request()
        req.names=list(FLAGS)
        response=self.call('/pixhawk/pixhawk_bridge/get_parameters',GetParameters,req,timeout=7)
        if len(response.values)!=len(FLAGS) or any(
            x.type!=ParameterType.PARAMETER_BOOL for x in response.values):
            raise RuntimeError('Unknown or non-boolean safety lock parameter')
        result={k:x.bool_value for k,x in zip(FLAGS,response.values)}
        if len(result)!=len(FLAGS) or any(result.values()):
            raise RuntimeError('HARD STOP: live hardware bridge write enabled or unknown: '+str(result))
        return result

def main():
    rclpy.init();n=Probe()
    results={'timestamp_utc':datetime.now(timezone.utc).isoformat(),
      'physical_device':'/dev/ttyACM0 via existing ROS2 bridge',
      'probe_mode':'READ_ONLY_AND_DENIAL_TESTS',
      'firmware_flashing_performed':False,
      'cube_motor_command_sent':False,
      'cube_mission_changed':False,
      'cube_parameter_changed':False,
      'simulation_vs_hardware':'HARDWARE_READS_ONLY',
      'observations':{},'blocked_calls':{}}
    try:
        a=n.get_status()
        if not a.get('connected') or a.get('armed',True):
            raise RuntimeError('HARD STOP: Cube disconnected or armed')
        locks=n.permissions();results['write_permissions']=locks
        results['observations']['initial_status']=a
        for _ in range(35):
            rclpy.spin_once(n,timeout_sec=.1)
            if n.firmware:break
        if not n.firmware:
            raise RuntimeError('Read-only bridge firmware version topic unavailable')
        import re
        match=re.fullmatch(
          r'version_raw=(\d+) board=(\d+) capabilities=(\d+) git=([0-9a-f]+)',
          n.firmware[-1])
        if match is None:
            raise RuntimeError('Unexpected AUTOPILOT_VERSION payload')
        vraw=int(match.group(1))
        results['observations']['firmware_version']={
            'px4_semver':f'{vraw>>24}.{(vraw>>16)&255}.{(vraw>>8)&255}',
            'release_type':vraw&255,
            'official_release':(vraw&255)==255,
            'board_version':int(match.group(2)),
            'capabilities':int(match.group(3)),
            'flight_git_hex':match.group(4),
            'raw':vraw,
            'source':'live /pixhawk/firmware/version from physical Cube over USB'
        }
        print('CUBE_ACTUAL_FIRMWARE',results['observations']['firmware_version'],flush=True)

        backup_bytes=BACKUP.read_bytes()
        backup=json.loads(backup_bytes)
        if not backup.get('complete') or len(backup.get('parameters',[]))!=732:
            raise RuntimeError('Backup is not complete 732-param snapshot')
        results['backup_sha256']=hashlib.sha256(backup_bytes).hexdigest()
        for name in ('SYS_AUTOSTART','MAV_TYPE','COM_RC_IN_MODE','RC_CHAN_CNT','RD_WHEEL_TRACK'):
            req=ParamGet.Request();req.name=name
            res=n.call('/pixhawk/param_get',ParamGet,req,timeout=8)
            results['observations']['param_'+name]={
                'success':res.success,'value':res.value if res.success else None,
                'mav_type':res.mav_type,'message':res.message}
            print('CUBE_PARAM',name,res.success,res.value if res.success else 'N/A',res.message,flush=True)
        m=MissionDownload.Request();m.mission_type=0
        res=n.call('/pixhawk/mission_download',MissionDownload,m,timeout=22)
        results['observations']['mission_download']={
            'success':res.success,'count':len(res.items),'message':res.message,
            'read_only':True}
        print('CUBE_MISSION_DOWNLOAD',res.success,len(res.items),res.message,flush=True)
        ver=CommandLong.Request();ver.command=512
        ver.params=[148.,0.,0.,0.,0.,0.,0.]
        vers=n.call('/pixhawk/command_long',CommandLong,ver,timeout=9)
        results['observations']['firmware_message_request']={
            'acknowledged':vers.success,'ack_result':vers.result,'message':vers.message,
            'version_decoded':False}
        print('CUBE_FIRMWARE_REQUEST',vers.success,vers.result,vers.message,flush=True)
        # IMPORTANT: All following calls are ROS-local denial tests, not physical FCU commands.
        # Must not make these calls if any write/movement flag has been enabled.
        g=GotoLocal.Request()
        g.x=1.;g.y=0.;g.z=0.;g.yaw=0.;g.duration_sec=1.
        gr=n.call('/pixhawk/goto_local',GotoLocal,g)
        results['blocked_calls']['goto_local']={'accepted':gr.success,'reason':gr.message}
        print('GOTO_LOCAL_BLOCKED',not gr.success,gr.message,flush=True)
        x=GotoGlobal.Request()
        x.latitude=-6.2;x.longitude=106.8;x.altitude=0.
        x.yaw=0.;x.duration_sec=1.
        gx=n.call('/pixhawk/goto_global',GotoGlobal,x)
        results['blocked_calls']['goto_global']={'accepted':gx.success,'reason':gx.message}
        print('GOTO_GLOBAL_BLOCKED',not gx.success,gx.message,flush=True)
        md=SetMode.Request();md.main_mode=4;md.sub_mode=4
        mc=n.call('/pixhawk/set_mode',SetMode,md)
        results['blocked_calls']['auto_mode']={'accepted':mc.success,'reason':mc.message}
        print('AUTO_MODE_BLOCKED',not mc.success,mc.message,flush=True)
        upload=MissionUpload.Request();upload.mission_type=0;upload.items=[]
        uu=n.call('/pixhawk/mission_upload',MissionUpload,upload)
        results['blocked_calls']['mission_upload']={'accepted':uu.success,'reason':uu.message}
        print('MISSION_UPLOAD_BLOCKED',not uu.success,uu.message,flush=True)
        b=n.get_status()
        results['observations']['final_status']=b
        results['assertions']={
           'px4_v1_17_official_firmware_observed':results['observations']['firmware_version']['px4_semver']=='1.17.0' and results['observations']['firmware_version']['official_release'],
           'hardware_link_still_connected':b.get('connected') is True,
           'remained_disarmed':not b.get('armed',True),
           'read_back_sys_autostart_zero':results['observations']['param_SYS_AUTOSTART']['value']==0,
           'read_back_mav_type_zero':results['observations']['param_MAV_TYPE']['value']==0,
           'real_mission_download_responded':res.success,
           'read_only_version_request_acknowledged':vers.success,
           'all_motion_and_config_services_denied':all(not v['accepted'] for v in results['blocked_calls'].values()),
           'all_hardware_command_permissions_disabled':not any(locks.values()),
           'motor_goto_and_real_mission_execution_verified':False,
        }
        for name,value in results['assertions'].items():
            print('ASSERT',name,value,flush=True)
        results['hardware_mission_goto_ready']=False
        if not all(v for k,v in results['assertions'].items()
                   if k!='motor_goto_and_real_mission_execution_verified'):
            raise RuntimeError('At least one fail-closed assertion failed')
        print('REAL_CUBE_USB_READONLY_PASS_MISSION_GOTO_BLOCKED_BY_DESIGN',flush=True)
    except Exception as e:
        results['error']=str(e)
        print('FAIL_READONLY_PROBE',repr(e),flush=True)
        raise
    finally:
        REPORT.parent.mkdir(parents=True,exist_ok=True)
        temp=REPORT.with_suffix('.json.tmp')
        temp.write_text(json.dumps(results,indent=2,sort_keys=True))
        temp.replace(REPORT)
        n.destroy_node();rclpy.shutdown()
        print('REPORT',REPORT,flush=True)

if __name__=='__main__':main()
