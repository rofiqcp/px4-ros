#!/usr/bin/env python3
"""Audited, reversible *hardware* PX4 parameter test.
Only SDLOG_UTC_OFFSET; initial snapshot must match pre-existing full backup.
Raising on restore failure intentionally signals intervention is needed.
"""
import glob,json,time,sys
from datetime import datetime,timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pixhawk_interfaces.srv import ParamGet,ParamSet

TEST_PARAM='SDLOG_UTC_OFFSET'
ALTERNATE=60
def wait(node,cls,name,req,seconds=8):
    cli=node.create_client(cls,name)
    if not cli.wait_for_service(timeout_sec=4):
        raise RuntimeError(name+' unavailable')
    f=cli.call_async(req)
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline and not f.done():
        rclpy.spin_once(node,timeout_sec=.05)
    if not f.done() or f.result() is None:
        raise RuntimeError(name+' timeout')
    return f.result()
def main():
    backups=sorted(glob.glob(str(Path(__file__).resolve().parents[1] / 'backups/cube_px4_*_complete.json')))
    if not backups:raise RuntimeError('No verified complete param snapshot: ABORT')
    backup=json.load(open(backups[0]))
    if not backup.get('complete') or len(backup['parameters'])!=backup['expected']:
        raise RuntimeError('PARAM BACKUP INCOMPLETE, ABORT')
    key=[v for v in backup['parameters'] if v['name']==TEST_PARAM]
    if len(key)!=1 or key[0]['value']!=0 or key[0]['type']!=6:
        raise RuntimeError('Unexpected initial logging timezone; ABORT')
    rclpy.init()
    node=Node('pixhawk_reversible_param_test')
    status={'connected':False}
    sub=node.create_subscription(String,'/pixhawk/status',
        lambda m:status.update(json.loads(m.data)),10)
    log={'date_utc':datetime.now(timezone.utc).isoformat(),
         'backup':backups[0],'param':TEST_PARAM,
         'before':0,'test_value':ALTERNATE,'steps':[]}
    def read():
        req=ParamGet.Request();req.name=TEST_PARAM
        r=wait(node,ParamGet,'/pixhawk/param_get',req)
        log['steps'].append(['read',r.success,r.value,r.mav_type,r.message])
        if not r.success or r.mav_type!=6:raise RuntimeError('PARAM_GET invalid')
        return r.value
    def write(value):
        req=ParamSet.Request()
        req.name=TEST_PARAM;req.mav_type=6;req.value=float(value)
        r=wait(node,ParamSet,'/pixhawk/param_set',req)
        log['steps'].append(['write',value,r.success,r.applied_value,r.message])
        if not r.success:raise RuntimeError('PARAM_SET not confirmed '+r.message)
        return r.applied_value
    try:
        deadline=time.monotonic()+7
        while not status['connected'] and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        print('HARDWARE_STATUS',status,flush=True)
        if not status['connected'] or status.get('armed') is not False:
            raise RuntimeError('Disarmed, connected Pixhawk required')
        before=read()
        if before!=key[0]['value']:raise RuntimeError('Live value differs from full backup: abort')
        try:
            write(ALTERNATE)
            changed=read()
            if changed!=ALTERNATE:raise RuntimeError('Change not verified via readback')
            print('HARDWARE_PARAM_TEMPORARILY_CHANGED',TEST_PARAM,before,'->',changed,flush=True)
        finally:
            # Always attempt restore, even when intermediate read/write fails.
            last_error=None
            for trial in range(3):
                try:write(before)
                except Exception as error:
                    last_error=repr(error)
                    log['steps'].append(['restore_confirmation_warning',trial,last_error])
                restored=read()
                if restored==before:break
                time.sleep(.3)
            if restored!=before:raise RuntimeError('RESTORE FAILED: original value not recovered: '+repr(last_error))
            log['restored']=True
            print('HARDWARE_PARAM_RESTORED',TEST_PARAM,restored,flush=True)
        log['result']='PASS'
    except Exception as e:
        log['result']='FAIL '+repr(e)
        print('PARAM_TEST_ERROR',repr(e),flush=True)
        raise
    finally:
        folder=(Path(__file__).resolve().parents[1] / 'backups')
        path=folder/('cube_reversible_param_test_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(log,indent=2))
        print('PARAM_TEST_LOG',path,log['result'] if 'result' in log else 'INCOMPLETE',flush=True)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
