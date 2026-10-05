#!/usr/bin/env python3
"""Isolated real Explorer process, synthetic room and kinematic waypoint follower.

Tests normal frontier-exhaustion -> inspection -> return -> save contract, not
PX4 dynamics or full warehouse coverage. No topics on the user's ROS master.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    folder=Path(args.output)
    folder.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11349'
    import numpy as np
    import rospy
    import rosgraph
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header,String
    from std_srvs.srv import Trigger,TriggerResponse
    if rosgraph.is_master_online():
        raise RuntimeError('Isolated master already occupied')
    processes=[]
    logs=[]
    def start(name,cmd):
        stream=(folder/(name+'.log')).open('w')
        logs.append(stream)
        processes.append(subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        start('core',['roscore','-p','11349'])
        until=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>until: raise RuntimeError('master timeout')
            time.sleep(.1)
        rospy.init_node('mission_fixture_driver',disable_signals=True)
        for key,value in dict(bounds=[0.,0.,8.,8.],no_frontier_seconds=1.,settle_seconds=.5,
                              initial_window_radius=6.,growth_margin=0.,
                              require_takeoff_confirmation=False,
                              surface_min_views=2,convergence_seconds=3.,inspection_revisit_seconds=.5,
                              home_settle_seconds=1.,max_mission_seconds=100.,return_timeout=30.,
                              planning_hz=5.,report=str(folder/'status.json')).items():
            rospy.set_param('/mission_fixture/'+key,value)
        state={}
        phases=set()
        saves=[]
        def receive(msg):
            state['status']=json.loads(msg.data)
            phases.add(state['status']['state'])
        def save(req):
            saves.append(time.monotonic())
            return TriggerResponse(True,'Synthetic save acknowledgement only; not a real PCD export')
        rospy.Service('/demo/save_map',Trigger,save)
        rospy.Subscriber('/uav1/exploration/status',String,receive,queue_size=1)
        rospy.Subscriber('/uav1/planner/waypoint',PoseStamped,lambda m:state.update(waypoint=m),queue_size=1)
        odom_pub=rospy.Publisher('/uav1/fastlio/odom',Odometry,queue_size=1)
        pose_pub=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1)
        pubs={key:rospy.Publisher(topic,PointCloud2,queue_size=1) for key,topic in (
            ('map','/uav1/fastlio/cloud_map'),('scan','/uav1/fastlio/registered_scan'),('free','/uav1/local_free_space'))}
        start('explorer',['rosrun','fastlio_bridge','warehouse_explorer.py','__name:=mission_fixture'])
        free=[(x,y,1.2) for x in np.arange(.1,8,.2) for y in np.arange(.1,8,.2)]
        walls=[p for p in free if p[0]<.3 or p[0]>7.7 or p[1]<.3 or p[1]>7.7]
        position=np.array([1.5,1.5])
        began=time.monotonic()
        while time.monotonic()-began<130:
            if any(p.poll() is not None for p in processes): raise RuntimeError('fixture process exited')
            stamp=rospy.Time.now()
            z=1.2  # synthetic airborne fixture; not an initialization/takeoff test
            target=state.get('waypoint')
            if target is not None:
                delta=np.array([target.pose.position.x,target.pose.position.y])-position
                position+=delta*min(1.,.12/max(float(np.linalg.norm(delta)),1e-6))
            odom=Odometry(header=Header(stamp=stamp,frame_id='map'))
            odom.pose.pose.position.x,odom.pose.pose.position.y=position
            odom.pose.pose.position.z=z
            odom.pose.pose.orientation.w=1.
            pose=PoseStamped(header=odom.header,pose=odom.pose.pose)
            odom_pub.publish(odom)
            pose_pub.publish(pose)
            for key,pub in pubs.items():
                pub.publish(create_cloud_xyz32(odom.header,free if key=='free' else walls))
            status=state.get('status',{})
            if status.get('state')=='COMPLETE':
                # This closed accessible room must CONVERGE, not merely stop
                # with a partial map. Partial exhaustion is tested separately.
                passed=bool('INSPECTING' in phases and 'RETURNING' in phases and len(saves)==1 and
                            status.get('finish_reason')=='observed_reachable_converged' and
                            status.get('exploration_complete') and status.get('home_confirmed'))
                result=dict(passed=passed,phases=sorted(phases),save_calls=len(saves),mission=status,
                            note='Synthetic kinematic integration only, not warehouse completeness or PX4 safety')
                (folder/'result.json').write_text(json.dumps(result,indent=2))
                print(json.dumps(result,indent=2))
                return 0 if passed else 1
            time.sleep(.1)
        raise RuntimeError('normal mission did not finish within fixture bound')
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait()
        for stream in logs: stream.close()


if __name__=='__main__':
    raise SystemExit(main())
