#!/usr/bin/env python3
"""Build a blocked, read-only differential Rover PX4 staging report from backup.
No MAVLink, serial, flight controller, mode change, or actuator access.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_BACKUP=ROOT/'backups/cube_px4_20261008_213146_complete.json'
DEFAULT_OUT=ROOT/'config/px4_differential_staged.yaml'
DEFAULT_REPORT=ROOT/'results/px4_differential_preflight.json'

def analyze(backup):
    blob=Path(backup).read_bytes()
    data=json.loads(blob)
    params=data['parameters']
    if not params or not data.get('complete',False) or len(params)!=len({x['name'] for x in params}):
        raise RuntimeError('Parameter backup incomplete / duplicate; abort')
    settings={x['name']:x['value'] for x in params}
    rd=sorted(k for k in settings if k.startswith('RD_'))
    ro=sorted(k for k in settings if k.startswith('RO_'))
    ca=sorted(k for k in settings if k.startswith('CA_'))
    rc_mode=settings.get('COM_RC_IN_MODE',None)
    rc_maps={k:settings[k] for k in settings if k.startswith('RC_MAP_')}
    pwm={k:settings[k] for k in settings if k.startswith(('PWM_MAIN_FUNC','PWM_AUX_FUNC'))}
    blocks=[]
    if not rd:
        blocks.append('No RD_* differential Rover parameters: installed firmware support unproven')
    if 'RD_WHEEL_TRACK' not in settings:
        blocks.append('RD_WHEEL_TRACK absent: do not set SYS_AUTOSTART to 50000')
    if not ca:
        blocks.append('No CA_* actuator assignment parameters: output mapping schema unknown')
    if settings.get('SYS_AUTOSTART')==0:
        blocks.append('SYS_AUTOSTART=0: airframe not configured')
    if settings.get('MAV_TYPE')==0:
        blocks.append('MAV_TYPE=0: generic, differential rover not confirmed')
    if settings.get('RC_CHAN_CNT',0)==0:
        blocks.append('No RC channels seen in backup (RC_CHAN_CNT=0)')
    if not any(v for v in rc_maps.values()):
        blocks.append('No calibrated RC_MAP_* mappings')
    blocks.extend([
        'Installed AUTOPILOT_VERSION and PX4 Rover module capabilities unverified',
        'Motor left/right pins, ESC type, reverse direction, disarmed PWM, failsafe and wheel track not measured',
        'Vehicle must stay disarmed with independent motor enable/kill path until bench approval'
    ])
    stage={
      'status':'BLOCKED_NOT_APPLIED',
      'writes_to_cube':False,
      'hardware_arming_allowed':False,
      'firmware_flashing_allowed':False,
      'backup_sha256':hashlib.sha256(blob).hexdigest(),
      'backup_parameter_count':len(params),
      'snapshot':{
        'SYS_AUTOSTART':settings.get('SYS_AUTOSTART'),
        'MAV_TYPE':settings.get('MAV_TYPE'),
        'COM_RC_IN_MODE':rc_mode,
        'RC_CHAN_CNT':settings.get('RC_CHAN_CNT'),
        'PWM_output_assignments':dict(sorted(pwm.items())),
        'RD_parameter_names':rd,
        'RO_parameter_names':ro,
        'CA_parameter_names':ca,
      },
      'candidate_only_after_exact_firmware_verification':{
        'frame':'Generic Rover Differential',
        'SYS_AUTOSTART':50000,
        'RD_WHEEL_TRACK':'REQUIRES_MEASURED_TRACK_M_AND_PARAMETER_PRESENT',
        'left_motor_output_pin':'UNKNOWN_NOT_ASSIGNED',
        'right_motor_output_pin':'UNKNOWN_NOT_ASSIGNED',
        'left_ESC_type':'UNKNOWN',
        'right_ESC_type':'UNKNOWN',
        'RC_receiver_protocol':'UNKNOWN',
        'RC_manual_switch':'UNKNOWN',
      },
      'ros_only_differential_model':{
        'left_wheel_mps':'v_forward_mps - yaw_rate_radps * measured_wheel_track_m / 2',
        'right_wheel_mps':'v_forward_mps + yaw_rate_radps * measured_wheel_track_m / 2',
        'geometry_verified':False,
        'control_output_enabled':False,
      },
      'blockers':blocks,
    }
    report={
      'generated_at_utc':datetime.now(timezone.utc).isoformat(),
      'cube_modified':False,
      'backup_sha256':stage['backup_sha256'],
      'supported_firmware_confirmed':False,
      'motor_output_mapping_confirmed':False,
      'receiver_confirmed':False,
      'ready_to_apply_to_cube':False,
      'blocked_reasons':blocks,
    }
    return stage,report

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--backup',default=str(DEFAULT_BACKUP))
    parser.add_argument('--output',default=str(DEFAULT_OUT))
    parser.add_argument('--report',default=str(DEFAULT_REPORT))
    args=parser.parse_args()
    stage,report=analyze(args.backup)
    for pathname,datum,is_yaml in ((args.output,stage,True),(args.report,report,False)):
        p=Path(pathname);p.parent.mkdir(parents=True,exist_ok=True)
        tmp=p.with_suffix(p.suffix+'.tmp')
        tmp.write_text(yaml.safe_dump(datum,sort_keys=False) if is_yaml else json.dumps(datum,indent=2))
        tmp.replace(p)
        print('SAVED',p)
    print('BLOCKED_NOT_APPLIED: found RD*',len(stage['snapshot']['RD_parameter_names']),
          'CA*',len(stage['snapshot']['CA_parameter_names']),
          'original parameter count',stage['backup_parameter_count'])
    print('Cube writes=0; arm=0; flash=0')

if __name__=='__main__':
    main()
