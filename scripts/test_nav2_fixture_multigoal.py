#!/usr/bin/env python3
"""Nav2 global planner action fixture: multi-goal planning only (no cmd_vel).

ROS_DOMAIN_ID=106 required. Start+goal supplied explicitly in map frame.
This script never calls NavigateToPose/FollowPath or arms PX4.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid

OUT=(Path(__file__).resolve().parents[1] / "results/planner_waypoints_20261009/nav2_fixture_paths.json")


class PlannerFixture(Node):
    def __init__(self):
        super().__init__("nav2_multigoal_planner_test")
        self.grid=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub=self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                                           self._on_map,qos)
        self.client=ActionClient(self,ComputePathToPose,"/compute_path_to_pose")
    def _on_map(self,m):
        self.grid=m
    def wait_for(self,what,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if what():return True
        return False
    def pose(self,pair):
        p=PoseStamped()
        p.header.frame_id="map"
        p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x,p.pose.position.y=pair
        p.pose.orientation.w=1.
        return p
    def path(self,origin,dest):
        goal=ComputePathToPose.Goal()
        goal.goal=self.pose(dest)
        goal.start=self.pose(origin)
        goal.planner_id="GridBased"
        goal.use_start=True
        fut=self.client.send_goal_async(goal)
        assert self.wait_for(lambda:fut.done(),8),"nav2 action goal response timeout"
        handle=fut.result()
        if not handle or not handle.accepted:return None,"NAV2_GOAL_REJECTED"
        result_future=handle.get_result_async()
        assert self.wait_for(lambda:result_future.done(),12),"nav2 path timeout"
        result=result_future.result()
        if result is None:return None,"NO_RESULT"
        poses=result.result.path.poses
        return [(p.pose.position.x,p.pose.position.y) for p in poses],str(result.status)


def index_to_xy(m,i):
    x=i%m.info.width;y=i//m.info.width
    return (m.info.origin.position.x+(x+0.5)*m.info.resolution,
            m.info.origin.position.y+(y+0.5)*m.info.resolution)


def largest_clear_area(m):
    w=m.info.width;h=m.info.height
    # Avoid inflated obstacle/unknown cells: 0 indicates fully free.
    good={idx for idx,val in enumerate(m.data) if val==0}
    visited=set()
    largest=[]
    while good:
        seed=good.pop()
        queue=deque([seed]);part=[seed]
        visited.add(seed)
        while queue:
            n=queue.popleft()
            x=n%w;y=n//w
            for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                j=ny*w+nx
                if 0<=nx<w and 0<=ny<h and j in good:
                    good.remove(j);visited.add(j);queue.append(j);part.append(j)
        if len(part)>len(largest):largest=part
    return largest


def cell_value(m,x,y):
    resolution=m.info.resolution
    ix=math.floor((x-m.info.origin.position.x)/resolution)
    iy=math.floor((y-m.info.origin.position.y)/resolution)
    if not (0<=ix<m.info.width and 0<=iy<m.info.height):
        return 100
    return m.data[iy*m.info.width+ix]


def detour_candidates(m, area):
    coords=[index_to_xy(m,i) for i in area]
    sampled=coords[::max(1,len(coords)//155)]
    produced=0
    for start in sampled:
        for goal in sampled:
            direct=math.dist(start,goal)
            if not 3.0<direct<9.0:
                continue
            steps=max(2,int(direct/0.12))
            blocked=sum(
                cell_value(m,start[0]+(goal[0]-start[0])*k/steps,
                           start[1]+(goal[1]-start[1])*k/steps)>=99
                for k in range(1,steps))
            if blocked>=4:
                yield start,goal,direct,blocked
                produced+=1
                if produced>=12:
                    return


def pick_goals(m,area):
    coords={i:index_to_xy(m,i) for i in area}
    # Choose start centrally among the largest free region, away from map edges.
    mean_x=sum(p[0] for p in coords.values())/len(coords)
    mean_y=sum(p[1] for p in coords.values())/len(coords)
    start=min(coords.values(),key=lambda p:math.dist(p,(mean_x,mean_y)))
    selections=[]
    for target in ((start[0]+1.5,start[1]),
                   (start[0]+1.5,start[1]+1.5),
                   (start[0]-1.8,start[1]-1.8),
                   (start[0]+3.0,start[1]-1.0)):
        point=min(coords.values(),key=lambda p:math.dist(p,target))
        if math.dist(point,start)>0.8 and point not in selections:
            selections.append(point)
    return start,selections


def main():
    if os.environ.get("ROS_DOMAIN_ID")!="106":
        raise RuntimeError("Fixture only. Never run in hardware domain.")
    rclpy.init()
    node=PlannerFixture()
    results={"ros_domain_id":106,"hardware_commands":False,
             "planner":"nav2_navfn_planner/NavfnPlanner","paths":[]}
    try:
        assert node.client.wait_for_server(timeout_sec=12),"nav2 global planner not active"
        assert node.wait_for(lambda:node.grid is not None,9),"No global Nav2 costmap"
        m=node.grid
        assert m.header.frame_id=="map"
        area=largest_clear_area(m)
        print("MAP",m.info.width,m.info.height,"free_component",len(area),flush=True)
        assert len(area)>200
        start,goals=pick_goals(m,area)
        assert len(goals)>=3,(start,goals)
        results["map_stats"]={"width":m.info.width,"height":m.info.height,
                              "resolution":m.info.resolution,"largest_clear_area":len(area)}
        results["start"]=start
        results["goals"]=goals
        for i,goal in enumerate(goals):
            path,status=node.path(start,goal)
            outcome={"goal_index":i,"target_xy_m":goal,"status":status}
            if path:
                outcome["poses"]=len(path)
                outcome["start_xy_m"]=path[0]
                outcome["end_xy_m"]=path[-1]
                outcome["path_length_m"]=round(sum(math.dist(a,b) for a,b in zip(path,path[1:])),3)
                outcome["goal_distance_m"]=math.dist(goal,path[-1])
                visited_cells=[cell_value(m,*point) for point in path]
                outcome["unknown_cells"]=sum(cost<0 for cost in visited_cells)
                outcome["lethal_cells"]=sum(cost>=99 for cost in visited_cells)
                outcome["maximum_cost"]=max(visited_cells)
                outcome["valid"]=(outcome["poses"]>=2 and outcome["goal_distance_m"]<0.65
                                  and not outcome["unknown_cells"] and not outcome["lethal_cells"])
            else:
                outcome["valid"]=False
            results["paths"].append(outcome)
            print("NAV2_COMPUTE_PATH",i,outcome,flush=True)
        # Positive obstacle case: direct segment crosses lethal cells, while
        # a collision-free Nav2 plan must detour around the obstacle.
        detour=None
        for a,b,direct,blocked in detour_candidates(m,area):
            path,status=node.path(a,b)
            if not path:
                continue
            occupancy=[cell_value(m,*xy) for xy in path]
            length=sum(math.dist(x,y) for x,y in zip(path,path[1:]))
            if (len(path)>=2 and all(0<=v<99 for v in occupancy)
                    and length>direct*1.03
                    and math.dist(path[-1],b)<0.65):
                detour={"start_xy_m":a,"goal_xy_m":b,
                        "direct_distance_m":round(direct,3),
                        "path_length_m":round(length,3),
                        "direct_line_lethal_samples":blocked,
                        "nav2_path_lethal_or_unknown":0,
                        "pose_count":len(path)}
                break
        results["obstacle_detour"]=detour
        print("NAV2_OBSTACLE_DETOUR",detour,flush=True)
        assert detour is not None,"No verified obstacle detour found in synthetic occupancy map"
        # Negative case: planner must NOT return a path to a goal 50 metres
        # beyond the occupancy map. A response indicating no path is success.
        unreachable=(start[0]+60.0,start[1]+60.0)
        no_path,no_status=node.path(start,unreachable)
        results["outside_map_goal_rejected"]=not no_path
        results["outside_map_response"]=no_status
        print("NAV2_OUTSIDE_MAP_REJECT",results["outside_map_goal_rejected"],no_status,flush=True)
        assert results["outside_map_goal_rejected"],"Unsafe planner accepted off-map goal"
        count=sum(p["valid"] for p in results["paths"])
        results["success_count"]=count
        results["goal_count"]=len(goals)
        assert count>=3,f"Nav2 did not plan three candidate point goals: {results}"
        print("NAV2_MULTI_POINT_GLOBAL_PLANNER_PASS",count,flush=True)
    finally:
        OUT.parent.mkdir(parents=True,exist_ok=True)
        OUT.write_text(json.dumps(results,indent=2))
        print("NAV2_PLANNER_EVIDENCE",OUT,flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
