#!/usr/bin/env python3
"""Read-only PX4 parameter export via the running ROS2 bridge. No writes to FC."""
import argparse
import hashlib
import json
import os
import pathlib
import time
from collections import Counter
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class Collector(Node):
    def __init__(self):
        super().__init__('pixhawk_parameter_backup')
        self.entries={}
        self.by_index={}
        self.expected=0
        self.messages=0
        self.last_new=time.monotonic()
        self.create_subscription(String,'/pixhawk/parameters',self.cb,1000)
        self.client=self.create_client(Trigger,'/pixhawk/param_request_list')
        self.status=None
        self.create_subscription(String,'/pixhawk/status',self.st,10)
    def st(self,msg):
        try:self.status=json.loads(msg.data)
        except ValueError:pass
    def cb(self,msg):
        try:
            data=json.loads(msg.data)
            index=int(data['index'])
            name=data['name']
            expected=int(data['count'])
            if not (0<=index<max(expected,1)) or not name:
                return
            self.messages+=1
            if name not in self.entries:self.last_new=time.monotonic()
            self.entries[name]=data
            self.by_index[index]=name
            self.expected=max(self.expected,expected)
        except (ValueError,KeyError,TypeError):pass
    def request(self):
        if not self.client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('param_request_list service missing')
        future=self.client.call_async(Trigger.Request())
        end=time.monotonic()+4
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.1)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Could not request PX4 full parameter list')

def save(collector,directory,complete,reason):
    directory.mkdir(parents=True,exist_ok=True)
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')
    state='complete' if complete else 'partial'
    base=directory/f'cube_px4_{tag}_{state}'
    rows=sorted(collector.entries.values(),key=lambda v:int(v['index']))
    data={'recorded_at':datetime.now(timezone.utc).isoformat(),
          'device':'3D Robotics PX4 FMU v2.x USB 26ac:0011',
          'source':'/pixhawk/param_request_list -> /pixhawk/parameters',
          'expected':collector.expected,
          'unique':len(rows),'received_messages':collector.messages,
          'complete':complete,'reason':reason,'status':collector.status,
          'missing_indices':sorted(set(range(collector.expected))-set(collector.by_index)),
          'parameters':rows}
    raw=json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False).encode()
    j=base.with_suffix('.json'); j.write_bytes(raw)
    param=base.with_suffix('.params')
    lines=['# MAVLink/PX4 parameter export - no changes performed','# System\tComponent\tName\tValue\tMAV_PARAM_TYPE']
    for item in rows:
        value=item['value']
        formatted=repr(value) if isinstance(value,float) else str(value)
        lines.append(f"1\t1\t{item['name']}\t{formatted}\t{item['type']}")
    param.write_text('\n'.join(lines)+'\n')
    manifest=base.with_suffix('.sha256')
    manifest.write_text(f'{hashlib.sha256(j.read_bytes()).hexdigest()}  {j.name}\n'+
      f'{hashlib.sha256(param.read_bytes()).hexdigest()}  {param.name}\n')
    return j,param,manifest

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--timeout',type=float,default=85)
    ap.add_argument('--output',default=str(pathlib.Path(__file__).resolve().parents[1] / 'backups'))
    a=ap.parse_args()
    rclpy.init()
    c=Collector()
    try:
        deadline=time.monotonic()+a.timeout
        for attempt in range(4):
            c.request()
            print(f'PARAM_LIST_REQUEST attempt={attempt+1} expected={c.expected} unique={len(c.entries)}',flush=True)
            last=time.monotonic()
            while time.monotonic()<deadline:
                rclpy.spin_once(c,timeout_sec=.05)
                if c.expected and len(c.by_index)==c.expected:break
                if time.monotonic()-last>4:
                    print(f'PROGRESS {len(c.entries)}/{c.expected} messages={c.messages}',flush=True)
                    last=time.monotonic()
                if c.expected and time.monotonic()-c.last_new>8:break
            if c.expected and len(c.by_index)==c.expected:break
            if time.monotonic()>deadline:break
        complete=c.expected>0 and len(c.by_index)==c.expected and len(c.entries)==c.expected
        path=pathlib.Path(a.output)
        files=save(c,path,complete,'Complete index set' if complete else 'Missing parameters or response timeout')
        print(f'BACKUP_RESULT complete={complete} expected={c.expected} unique={len(c.entries)} indices={len(c.by_index)}',flush=True)
        for f in files:print('BACKUP_FILE',f,'BYTES',f.stat().st_size,flush=True)
        return 0 if complete else 2
    finally:
        c.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
