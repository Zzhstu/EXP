#!/usr/bin/env python3
"""Opt-in simulation check: pause/resume + exact exported PCD source."""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import xyz


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    rospy.init_node('check_exploration_controls')
    data={}
    clouds=deque(maxlen=20)
    rospy.Subscriber('/uav1/fastlio/odom',Odometry,lambda m:data.update(odom=m),queue_size=1)
    rospy.Subscriber('/uav1/exploration/status',String,lambda m:data.update(status=json.loads(m.data)),queue_size=1)
    rospy.Subscriber('/uav1/fastlio/cloud_map',PointCloud2,clouds.append,queue_size=1,buff_size=16000000)
    def position():
        p=data['odom'].pose.pose.position
        return p.x,p.y
    toggle=rospy.ServiceProxy('/uav1/exploration/set_enabled',SetBool)
    rospy.wait_for_service('/uav1/exploration/set_enabled',timeout=15)
    result={}
    try:
        toggle(False)
        time.sleep(3)
        begin=position()
        state=data['status']['state']
        reached=data['status']['goals_reached']
        time.sleep(3)
        result.update(pause_state=state,pause_drift_xy_m=math.dist(begin,position()),
                      goals_unchanged=reached==data['status']['goals_reached'])
        saved=rospy.ServiceProxy('/demo/save_map',Trigger)()
        if not saved.success:
            raise RuntimeError(saved.message)
        path=Path(saved.message.split(' (')[0])
        metadata=json.loads(path.with_suffix('.json').read_text())
        payload=path.read_bytes()
        points=np.frombuffer(payload.split(b'DATA binary\n')[1],dtype='<f4').reshape(-1,3)
        cloud=next((m for m in list(clouds) if abs(m.header.stamp.to_sec()-metadata['stamp'])<1e-6),None)
        result.update(export_points=len(points),metadata_points=metadata['points'],
            sha256_matches=hashlib.sha256(payload).hexdigest()==metadata['sha256'],
            exact_topic_match=bool(cloud is not None and np.array_equal(xyz(cloud),points)))
        snapshot=Path(args.output).with_suffix('.pcd')
        shutil.copyfile(path,snapshot)
        result['snapshot']=str(snapshot)
    finally:
        toggle(True)
    time.sleep(2)
    result['resume_state']=data['status']['state']
    result['passed']=bool(result['pause_state']=='PAUSED' and result['pause_drift_xy_m']<.15 and
                          result['goals_unchanged'] and result['exact_topic_match'] and result['sha256_matches'] and
                          result['resume_state']!='PAUSED')
    Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2))
    if not result['passed']:
        raise RuntimeError('Controls/export regression failed')


if __name__=='__main__':
    main()
