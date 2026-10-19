#!/usr/bin/env python3
"""Read-only backup of waypoint/fence/rally mission lists from PX4 via ROS."""
import hashlib
import json
import time
from pathlib import Path
from datetime import datetime,timezone
import rclpy
from rclpy.node import Node
from pixhawk_interfaces.srv import MissionDownload

def main():
    rclpy.init()
    node=Node('pixhawk_mission_backup')
    client=node.create_client(MissionDownload,'/pixhawk/mission_download')
    results={}
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('mission_download unavailable')
        for mission_type,name in ((0,'mission'),(1,'geofence'),(2,'rally')):
            req=MissionDownload.Request();req.mission_type=mission_type
            f=client.call_async(req)
            deadline=time.monotonic()+25
            while not f.done() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
            if not f.done() or f.result() is None:
                results[name]={'success':False,'message':'Service timeout'}
                continue
            r=f.result()
            data=[]
            for item in r.items:
                data.append(dict(seq=item.seq,frame=item.frame,command=item.command,
                    current=item.current,autocontinue=item.autocontinue,params=list(item.params),
                    x=item.x,y=item.y,z=item.z,mission_type=item.mission_type))
            results[name]={'success':r.success,'message':r.message,'items':data}
            print('MISSION_BACKUP',name,r.success,r.message,'items',len(data),flush=True)
        out={'timestamp_utc':datetime.now(timezone.utc).isoformat(),
             'read_only':True,'vehicle':'Cube PX4','mission_data':results}
        folder=(Path(__file__).resolve().parents[1] / 'backups');folder.mkdir(parents=True,exist_ok=True)
        path=folder/('cube_missions_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(out,indent=2))
        sha=path.with_suffix('.sha256')
        sha.write_text(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.name+'\n')
        print('MISSION_BACKUP_FILE',path,flush=True)
        if not results['mission']['success']:
            raise RuntimeError('Waypoint mission backup incomplete')
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':main()
