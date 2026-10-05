#!/usr/bin/env python3
"""Read-only live experiment evidence; truth is NEVER published to navigation.

Reports partial exploration, not full coverage; ROI counts are not labelled MOS
accuracy. Save positive baseline before claiming old standing-person clearing.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt8
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import xyz
from analyze_dynamic_demo_bag import rotation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=600)
    parser.add_argument('--output',required=True)
    parser.add_argument('--person-start',nargs=2,type=float,default=[-8.,-4.],metavar=('X','Y'))
    args=parser.parse_args()
    rospy.init_node('exploration_evidence',anonymous=True)
    data={}
    for key,topic,cls in (
        ('truth','/gazebo/model_states',ModelStates),('odom','/uav1/fastlio/odom',Odometry),
        ('map','/uav1/fastlio/cloud_map',PointCloud2),('scan','/uav1/fastlio/registered_scan',PointCloud2),
        ('status','/uav1/exploration/status',String),('risk','/uav1/fused_collision_risk_level',UInt8),
        ('walk','/demo/pedestrian/walk_status',String)):
        rospy.Subscriber(topic,cls,lambda m,k=key:data.update({k:m}),queue_size=1,buff_size=16000000)
    samples=[]
    began=time.monotonic()
    report=dict(note='Partial flight functional test. XY area is not full 3D map completeness. '
                     'ROI contains possible background; min surface is point-cloud proxy, not contacts.',samples=samples)
    try:
        while time.monotonic()-began<args.seconds and not rospy.is_shutdown():
            snap=data.copy()
            if all(k in snap for k in ('truth','odom','map','status','scan')):
                truth=snap['truth']
                if all(k in truth.name for k in ('0','iris_mid360')):
                    actor=truth.pose[truth.name.index('0')].position
                    body=truth.pose[truth.name.index('iris_mid360')]
                    odom=snap['odom'].pose.pose
                    r=rotation(odom.orientation).dot(rotation(body.orientation).T)
                    t=np.array([odom.position.x,odom.position.y,odom.position.z])-r.dot(
                        [body.position.x,body.position.y,body.position.z])
                    pts=xyz(snap['map'])
                    world=(pts-t).dot(r)
                    sx,sy=args.person_start
                    roi=(np.hypot(world[:,0]-sx,world[:,1]-sy)<.4)&(world[:,2]>.4)&(world[:,2]<1.8)
                    scan=xyz(snap['scan'])-np.array([odom.position.x,odom.position.y,odom.position.z])
                    distances=np.linalg.norm(scan[(scan[:,2]>-.35)&(scan[:,2]<.45),:2],axis=1)
                    sample=dict(wall=time.monotonic()-began,sim=rospy.Time.now().to_sec(),
                        status=json.loads(snap['status'].data),old_person_roi_points=int(roi.sum()),
                        person_displacement=math.hypot(actor.x-sx,actor.y-sy),
                        person_world=[actor.x,actor.y],uav_world=[body.position.x,body.position.y,body.position.z],
                        uav_person_xy=math.hypot(actor.x-body.position.x,actor.y-body.position.y),
                        surface_xy=float(distances.min()) if len(distances) else None,
                        risk=snap['risk'].data if 'risk' in snap else None,
                        walk=snap['walk'].data if 'walk' in snap else None)
                    samples.append(sample)
                    report['sample_count']=len(samples)
                    report['before_move_roi_max']=max((s['old_person_roi_points'] for s in samples
                        if s['person_displacement']<.15),default=None)
                    report['after_move_roi_latest']=next((s['old_person_roi_points'] for s in reversed(samples)
                        if s['person_displacement']>1.5),None)
                    report['goals_reached_max']=max(s['status']['goals_reached'] for s in samples)
                    report['known_free_area_max_m2']=max(s['status']['known_free_area_m2'] for s in samples)
                    report['min_person_center_xy_m']=min(s['uav_person_xy'] for s in samples)
                    report['min_surface_xy_m']=min((s['surface_xy'] for s in samples if s['surface_xy'] is not None),default=None)
                    report['min_cruise_surface_xy_m']=min((s['surface_xy'] for s in samples
                        if s['surface_xy'] is not None and s['status']['state'] not in
                        ('TAKEOFF','WAIT_FRESH_DATA','WAIT_TAKEOFF_CONFIRMATION')),default=None)
                    report['state_counts']={state:sum(s['status']['state']==state for s in samples)
                                            for state in set(s['status']['state'] for s in samples)}
                    Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf8')
            time.sleep(1.)
    finally:
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)


if __name__=='__main__':
    main()
