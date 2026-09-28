#!/usr/bin/env python3
"""Opt-in Gazebo cangku regression; drives the same topic as the keyboard.

Run after the report, or --during-flight from before takeoff for a crossing
while the UAV navigates. Saves failures as well as successes; no truth map is
sent to the UAV. Position association is only an evaluation proxy, not MOS GT.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import shutil
import sys
import time
import numpy as np
import rospy
from fastlio_bridge.msg import DynamicObjectArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Trigger
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_dynamic_demo_bag import rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--during-flight', action='store_true')
    args = parser.parse_args()
    rospy.init_node('warehouse_regression')
    data = dict(states=None, odom=None, phase='waiting', risk=0, blocked=False)
    samples = []
    clouds = deque(maxlen=12)
    rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m: data.update(states=m), queue_size=1)
    rospy.Subscriber('/uav1/fastlio/odom', Odometry, lambda m: data.update(odom=m), queue_size=1)
    rospy.Subscriber('/uav1/fastlio/cloud_map', PointCloud2, clouds.append, queue_size=1)
    rospy.Subscriber('/uav1/collision_risk_level', UInt8, lambda m: data.update(risk=m.data), queue_size=1)
    rospy.Subscriber('/demo/pedestrian/blocked', Bool, lambda m: data.update(blocked=m.data), queue_size=1)
    def pose(name):
        states = data['states']
        return states.pose[states.name.index(name)]
    def transform():
        body_world, body_map = pose('iris_mid360'), data['odom'].pose.pose
        r = rotation(body_map.orientation).dot(rotation(body_world.orientation).T)
        w, m = body_world.position, body_map.position
        return r, np.array([m.x,m.y,m.z])-r.dot([w.x,w.y,w.z])
    def objects(msg):
        if data['states'] is None or data['odom'] is None:
            return
        p, u = pose('0').position, pose('iris_mid360').position
        r, t = transform()
        mp = r.dot([p.x,p.y,p.z])+t
        found = [o for o in msg.objects if math.hypot(o.pose.position.x-mp[0],o.pose.position.y-mp[1])<.9]
        samples.append(dict(phase=data['phase'], detected=bool(found),
            observed=any(not o.predicted for o in found),
            age=(rospy.Time.now()-msg.header.stamp).to_sec(),
            distance=math.hypot(p.x-u.x,p.y-u.y), risk=data['risk'],
            stamp=msg.header.stamp.to_sec(), wall=time.monotonic()))
    rospy.Subscriber('/uav1/dynamic_objects', DynamicObjectArray, objects, queue_size=1)
    pub = rospy.Publisher('/demo/pedestrian/0/cmd_vel', Twist, queue_size=1)
    ready = rospy.wait_for_message('/demo/pedestrian/ready', Bool, timeout=180)
    if not ready.data:
        raise RuntimeError('Warehouse pedestrian not ready')
    time.sleep(1)
    if 'Untitled' not in data['states'].name or '1' in data['states'].name:
        raise RuntimeError('Expected cangku world with exactly one pedestrian')
    if args.during_flight:
        deadline = time.monotonic()+120
        while pose('iris_mid360').position.y < .5 and time.monotonic()<deadline:
            time.sleep(.1)
        if pose('iris_mid360').position.y < .5:
            failure = dict(status='failed', reason='UAV did not reach crossing trigger before timeout',
                           uav_world_y=pose('iris_mid360').position.y)
            Path(args.output).write_text(json.dumps(failure, indent=2), encoding='utf8')
            raise RuntimeError(failure['reason'])
        if pose('iris_mid360').position.y > 2:
            raise RuntimeError('Started too late for the controlled in-flight crossing')
    def send(vx,vy):
        m = Twist()
        m.linear.x, m.linear.y = vx,vy
        pub.publish(m)
    def go(x,y,phase):
        data['phase'] = phase
        deadline = time.monotonic()+50
        while time.monotonic()<deadline:
            p = pose('0').position
            dx,dy = x-p.x,y-p.y
            distance = math.hypot(dx,dy)
            if distance<.10:
                break
            speed = min(.6,distance*1.5)
            send(speed*dx/distance,speed*dy/distance)
            time.sleep(.05)
        else:
            raise RuntimeError('Failed to reach pedestrian target '+phase)
        send(0,0)
        print(phase,pose('0').position,flush=True)
    try:
        start_wall, start_sim = time.monotonic(),rospy.Time.now().to_sec()
        go(-4.2,3,'cross_first')
        data['phase']='pause'
        time.sleep(5)
        go(-8,3,'cross_repeat')
        go(-8,6,'leave_initial')
        data['phase']='wall_guard'
        deadline=time.monotonic()+7
        ever_blocked=False
        while time.monotonic()<deadline:
            send(-1,0)
            ever_blocked |= data['blocked']
            time.sleep(.05)
        send(0,0)
        guard_x=pose('0').position.x
        go(-8,6,'return_from_wall')
        send(0,.6)
        time.sleep(1)
        p=pose('0').position
        stopped=(p.x,p.y)
        time.sleep(1)
        p=pose('0').position
        drift=math.hypot(p.x-stopped[0],p.y-stopped[1])
        data['phase']='final_observation'
        deadline=time.monotonic()+120
        while math.hypot(pose('iris_mid360').position.x+6,pose('iris_mid360').position.y-6)>.4 and time.monotonic()<deadline:
            time.sleep(.2)
        time.sleep(5)
        result=rospy.ServiceProxy('/demo/save_map',Trigger)()
        if not result.success:
            raise RuntimeError(result.message)
        path=Path(result.message.split(' (')[0])
        meta=json.loads(path.with_suffix('.json').read_text())
        snapshot=Path(args.output).with_name(Path(args.output).stem+'_map.pcd')
        shutil.copyfile(path,snapshot)
        shutil.copyfile(path.with_suffix('.json'),snapshot.with_suffix('.json'))
        xyz=np.frombuffer(path.read_bytes().split(b'DATA binary\n')[1],dtype='<f4').reshape(-1,3)
        matching=next((m for m in list(clouds) if abs(m.header.stamp.to_sec()-meta['stamp'])<1e-6),None)
        exact=False
        if matching is not None:
            source=np.asarray(list(point_cloud2.read_points(matching,field_names=('x','y','z'),skip_nans=True)),dtype='<f4').reshape(-1,3)
            exact=bool(np.array_equal(xyz,source[np.isfinite(source).all(axis=1)]))
        # Compare map points in WORLD coordinates; nonzero spawn is crucial.
        r,t=transform()
        world_xyz=(xyz-t).dot(r)
        roi=(np.hypot(world_xyz[:,0]+4.2,world_xyz[:,1]-3)<.4)&(world_xyz[:,2]>.4)&(world_xyz[:,2]<1.8)
        initial=(np.hypot(world_xyz[:,0]+8,world_xyz[:,1]-3)<.4)&(world_xyz[:,2]>.4)&(world_xyz[:,2]<1.8)
        phases={}
        for phase in sorted({s['phase'] for s in samples}):
            part=[s for s in samples if s['phase']==phase]
            phases[phase]=dict(messages=len(part),detected=sum(s['detected'] for s in part),
                observed=sum(s['observed'] for s in part),dynamic_risk_frames=sum(s['risk']>0 for s in part),
                age_p95_sim_s=float(np.percentile([s['age'] for s in part],95)),
                min_center_xy_m=min(s['distance'] for s in part))
        report=dict(phases=phases,wall_guard_stopped=ever_blocked,wall_guard_x=guard_x,
            deadman_drift_m=drift,export_equals_topic=exact,tested_map=str(snapshot),points=len(xyz),
            vacated_pause_roi_points=int(roi.sum()),vacated_initial_roi_points=int(initial.sum()),
            real_time_factor=(rospy.Time.now().to_sec()-start_sim)/(time.monotonic()-start_wall),
            uav_goal_error_world_xy_m=math.hypot(pose('iris_mid360').position.x+6,pose('iris_mid360').position.y-6))
        Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps(report,indent=2),flush=True)
        assert exact and drift<.02 and ever_blocked and guard_x>-9.5
        assert all(phases[p]['observed']>0 for p in ('cross_first','cross_repeat')), 'Crossing not detected'
        assert report['uav_goal_error_world_xy_m']<.4, 'UAV did not reach the goal'
    finally:
        send(0,0)


if __name__=='__main__':
    main()
