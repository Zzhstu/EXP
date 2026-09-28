#!/usr/bin/env python3
"""Read-only rack retention evidence; never sends truth to the UAV.

Accumulate actually measured registered returns in shelf-box ROIs, then compare
with maintained map. This measures preservation of OBSERVED points, not coverage
of unobserved mesh surfaces. Boxes and approximate world/map alignment are only
evaluation masks. Neighbour tolerance is explicitly reported.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import xyz
from analyze_dynamic_demo_bag import rotation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--seconds',type=float,default=1800)
    args=p.parse_args()
    boxes=[b for b in json.loads(Path(args.scene).read_text())['boxes']
           if 'shelf' in b['name'] and b['center'][0]>0]
    rospy.init_node('rack_retention_evidence',anonymous=True)
    data={}
    for key,topic,cls in (('truth','/gazebo/model_states',ModelStates),
                         ('odom','/uav1/fastlio/odom',Odometry),
                         ('scan','/uav1/fastlio/registered_scan',PointCloud2),
                         ('map','/uav1/fastlio/cloud_map',PointCloud2)):
        rospy.Subscriber(topic,cls,lambda m,k=key:data.update({k:(m,time.monotonic())}),
                         queue_size=1,buff_size=16000000)
    refs=[Counter() for _ in boxes]
    r=t=None
    last_scan=None
    began=time.monotonic()
    history=[]
    try:
        while time.monotonic()-began<args.seconds and not rospy.is_shutdown():
            snap=data.copy()
            if len(snap)<4 or any(time.monotonic()-v[1]>3 for v in snap.values()):
                time.sleep(.5)
                continue
            if r is None:
                truth=snap['truth'][0]
                body=truth.pose[truth.name.index('iris_mid360')]
                odom=snap['odom'][0].pose.pose
                r=rotation(odom.orientation).dot(rotation(body.orientation).T)
                t=np.array([odom.position.x,odom.position.y,odom.position.z])-r.dot(
                    [body.position.x,body.position.y,body.position.z])
            scan=snap['scan'][0]
            if scan is not last_scan:
                pts=xyz(scan)
                world=(pts-t).dot(r)
                for box,ref in zip(boxes,refs):
                    delta=world-np.asarray(box['center'])
                    c,s=np.cos(box['yaw']),np.sin(box['yaw'])
                    local=np.column_stack([c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1],delta[:,2]])
                    mask=(np.abs(local)<=np.asarray(box['size'])/2+.12).all(axis=1)&(world[:,2]>.25)&(world[:,2]<1.9)
                    if not mask.any():
                        continue
                    keys=np.unique(np.floor(pts[mask]/.15).astype(np.int32),axis=0)
                    ref.update(map(tuple,keys))
                last_scan=scan
            map_points=xyz(snap['map'][0])
            if not len(map_points):
                time.sleep(.5)
                continue
            keys=np.unique(np.floor(map_points/.15).astype(np.int32),axis=0)
            occupied=set(map(tuple,keys))
            offsets=[(x,y,z) for x in (-1,0,1) for y in (-1,0,1) for z in (-1,0,1)]
            rows=[]
            for box,ref in zip(boxes,refs):
                eligible=[k for k,n in ref.items() if n>=3]
                retained=sum(any((k[0]+dx,k[1]+dy,k[2]+dz) in occupied for dx,dy,dz in offsets) for k in eligible)
                rows.append(dict(name=box['name'],center_world=box['center'],observed_reference_voxels=len(eligible),
                                 retained_voxels=retained,retention=retained/len(eligible) if eligible else None))
            total=sum(v['observed_reference_voxels'] for v in rows)
            kept=sum(v['retained_voxels'] for v in rows)
            history.append(dict(sim=rospy.Time.now().to_sec(),wall=time.monotonic()-began,
                                reference=total,retained=kept))
            report=dict(note='Observed-return preservation, NOT whole-rack completeness. ROI box geometry only used by this reader.',
                voxel_m=.15,neighbour_tolerance_cells=1,min_reference_observations=3,
                map_from_world_rotation=r.tolist(),map_from_world_translation=t.tolist(),
                retention=kept/total if total else None,reference_voxels=total,retained_voxels=kept,
                racks=rows,history=history)
            Path(args.output).write_text(json.dumps(report,indent=2))
            time.sleep(2.)
    finally:
        print('Rack retention evidence: '+args.output,flush=True)


if __name__=='__main__':
    main()
