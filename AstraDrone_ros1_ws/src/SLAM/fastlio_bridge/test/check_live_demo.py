#!/usr/bin/env python3
"""Opt-in live SIMULATION test after demo report; moves actor 0 via teleop API.

Requires example.world; never runs automatically on a real vehicle.
Records actual detections, timestamps, deadman and exported XYZ equality.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
import rospy
from fastlio_bridge.msg import DynamicObjectArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rospy.init_node('live_demo_regression')
    data = {'states': None, 'phase': 'waiting'}
    detections = []
    clouds = deque(maxlen=12)
    rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m: data.update(states=m), queue_size=1)
    rospy.Subscriber('/uav1/fastlio/cloud_map', PointCloud2, clouds.append, queue_size=1)
    def objects(msg):
        if data['states'] is None:
            return
        states = data['states']
        p = states.pose[states.name.index('0')].position
        matched = [o for o in msg.objects if math.hypot(o.pose.position.x-p.x, o.pose.position.y-p.y) < .9]
        detections.append(dict(phase=data['phase'], stamp=msg.header.stamp.to_sec(),
                               age=(rospy.Time.now()-msg.header.stamp).to_sec(),
                               detected=bool(matched), predicted=all(o.predicted for o in matched),
                               nearest_error=min([math.hypot(o.pose.position.x-p.x, o.pose.position.y-p.y)
                                                  for o in matched], default=None)))
    rospy.Subscriber('/uav1/dynamic_objects', DynamicObjectArray, objects, queue_size=1)
    pub = rospy.Publisher('/demo/pedestrian/0/cmd_vel', Twist, queue_size=1)
    ready = rospy.wait_for_message('/demo/pedestrian/ready', Bool, timeout=15)
    if not ready.data:
        raise RuntimeError('Wait for the scripted crossing to finish before this test')
    time.sleep(1)
    if data['states'] is None or 'iris_mid360' not in data['states'].name:
        raise RuntimeError('Not the expected Gazebo scene')
    def position(actor='0'):
        states = data['states']
        return states.pose[states.name.index(actor)].position
    def go(x, y, phase):
        data['phase'] = phase
        deadline = time.monotonic()+45
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            p = position()
            dx, dy = x-p.x, y-p.y
            dist = math.hypot(dx, dy)
            if dist < .10:
                break
            cmd = Twist()
            speed = min(.6, dist*1.5)
            cmd.linear.x, cmd.linear.y = speed*dx/dist, speed*dy/dist
            pub.publish(cmd)
            time.sleep(.05)
        else:
            raise RuntimeError('Actor did not reach ' + phase)
        pub.publish(Twist())
        print(phase, (position().x, position().y), flush=True)
    start_other = (position('1').x, position('1').y)
    try:
        go(1.8, 2.5, 'approach_x')
        go(1.8, 9.5, 'approach_y')
        data['phase'] = 'pause_before_repeat'
        time.sleep(5)
        go(-1.8, 9.5, 'cross_first')
        data['phase'] = 'pause'
        time.sleep(5)
        go(1.8, 9.5, 'cross_repeat')
        # Explicitly test deadman: one nonzero command then no client traffic.
        cmd = Twist()
        cmd.linear.y = -.6
        pub.publish(cmd)
        time.sleep(1)
        stopped = (position().x, position().y)
        time.sleep(1)
        deadman_distance = math.hypot(position().x-stopped[0], position().y-stopped[1])
        go(1.8, 7., 'leave')
        data['phase'] = 'final_observation'
        time.sleep(5)
        result = rospy.ServiceProxy('/demo/save_map', Trigger)()
        if not result.success:
            raise RuntimeError(result.message)
        path = Path(result.message.split(' (')[0])
        meta = json.loads(path.with_suffix('.json').read_text())
        # The demo overwrites final_map again on exit. Preserve THIS tested
        # snapshot so the evidence does not silently refer to a different map.
        snapshot = Path(args.output).with_name(Path(args.output).stem + '_map.pcd')
        shutil.copyfile(path, snapshot)
        shutil.copyfile(path.with_suffix('.json'), snapshot.with_suffix('.json'))
        xyz = np.frombuffer(path.read_bytes().split(b'DATA binary\n')[1], dtype='<f4').reshape(-1, 3)
        matching = next((m for m in list(clouds) if abs(m.header.stamp.to_sec()-meta['stamp']) < 1e-6), None)
        exact = False
        if matching is not None:
            source = np.asarray(list(point_cloud2.read_points(matching, field_names=('x','y','z'), skip_nans=True)), dtype='<f4').reshape(-1,3)
            exact = bool(np.array_equal(xyz, source[np.isfinite(source).all(axis=1)]))
        roi = (np.hypot(xyz[:,0]+1.8, xyz[:,1]-9.5)<.4) & (xyz[:,2]>.4) & (xyz[:,2]<1.8)
        corridor = (np.abs(xyz[:,0])<2.2) & (np.abs(xyz[:,1]-9.5)<.4) & (xyz[:,2]>.4) & (xyz[:,2]<1.8)
        summary = {}
        for phase in sorted({d['phase'] for d in detections}):
            samples = [d for d in detections if d['phase']==phase]
            errors = [d['nearest_error'] for d in samples if d['nearest_error'] is not None]
            summary[phase] = dict(messages=len(samples), detected=sum(d['detected'] for d in samples),
                observed=sum(d['detected'] and not d['predicted'] for d in samples),
                age_p95_s=float(np.percentile([d['age'] for d in samples],95)),
                center_error_median=None if not errors else float(np.median(errors)))
        report = dict(phases=summary, deadman_drift_m=deadman_distance,
                      other_actor_drift_m=math.hypot(position('1').x-start_other[0],position('1').y-start_other[1]),
                      exported_map=str(path), export_equals_topic=exact,
                      tested_snapshot=str(snapshot), export_stamp=meta['stamp'],
                      export_sha256=meta['sha256'],
                      vacated_pause_roi_points=int(roi.sum()), points=len(xyz))
        report['vacated_crossing_corridor_points'] = int(corridor.sum())
        Path(args.output).write_text(json.dumps(report, indent=2), encoding='utf8')
        print(json.dumps(report, indent=2), flush=True)
        assert exact and deadman_distance<.02, 'Export or deadman regression'
        assert report['other_actor_drift_m'] < .02, 'Unselected actor did not hold its final pose'
        assert all(summary[p]['observed']>0 for p in ('cross_first','cross_repeat')), 'Repeated crossing was not detected'
    finally:
        pub.publish(Twist())


if __name__ == '__main__':
    main()
