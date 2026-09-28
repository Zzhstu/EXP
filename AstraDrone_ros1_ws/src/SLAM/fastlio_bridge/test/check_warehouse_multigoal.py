#!/usr/bin/env python3
"""Opt-in SITL test: publishes map goals; do NOT run on a real vehicle.

Run alongside warehouse_dynamic_demo.sh, without another goal publisher.
Uses user-reported first two goals, then returns to the initial demo target.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import UInt8, Float32


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=360,
                        help='wall seconds per goal, including long shelf detours')
    parser.add_argument('--start-at', choices=('initial', 'user_goal_1', 'user_goal_2', 'return'),
                        default='initial', help='explicit partial recovery test; recorded output remains partial')
    parser.add_argument('--return-goal', nargs=2, type=float,
                        help='explicit map XY if the original latched auto-goal publisher was restarted')
    args = parser.parse_args()
    rospy.init_node('warehouse_multigoal_test', disable_signals=True)
    state = {}
    def received(key):
        def callback(msg):
            state[key] = msg
            state[key+'_received'] = time.monotonic()
        return callback
    subscribers = [rospy.Subscriber(t, cls, received(key), queue_size=1) for t, cls, key in [
        ('/uav1/fastlio/odom', Odometry, 'odom'), ('/uav1/planner/local_path', RosPath, 'path'),
        ('/uav1/fused_collision_risk_level', UInt8, 'risk'),
        ('/uav1/nearest_static_surface_distance', Float32, 'distance')]]
    if args.return_goal:
        home = tuple(args.return_goal)
    else:
        auto = rospy.wait_for_message('/move_base_simple/goal', PoseStamped, timeout=90)
        home = (auto.pose.position.x, auto.pose.position.y)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    results = []
    targets = [('initial', home),
               ('user_goal_1', (5.60, 10.67)), ('user_goal_2', (14.00, -.15)),
               ('return', home)]
    targets = targets[[label for label, _ in targets].index(args.start_at):]
    try:
        for label, target in targets:
            if label != 'initial':
                deadline = time.monotonic()+10
                while goal_pub.get_num_connections() == 0:
                    if time.monotonic() > deadline:
                        raise RuntimeError('no planner goal subscriber')
                    time.sleep(.1)
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x, msg.pose.position.y = target
                msg.pose.orientation.w = 1
                # z=0 exercises actual RViz 2D semantics, not a fixed 3D goal.
                goal_pub.publish(msg)
            started = time.monotonic()
            stable = None
            item = dict(label=label, goal=list(target), arrived=False,
                        min_surface_distance_m=None, samples=[])
            results.append(item)
            while time.monotonic()-started < args.timeout:
                if 'odom' not in state or time.monotonic()-state['odom_received'] > 1.0:
                    stable = None
                    time.sleep(.1)
                    continue
                p = state['odom'].pose.pose.position
                error = math.hypot(p.x-target[0], p.y-target[1])
                path = state.get('path')
                d = state['distance'].data if 'distance' in state else float('inf')
                if math.isfinite(d):
                    item['min_surface_distance_m'] = min(d, item['min_surface_distance_m'] or d)
                sample = dict(sim_time=rospy.Time.now().to_sec(), xyz=[p.x,p.y,p.z], error=error,
                              path_poses=len(path.poses) if path else 0,
                              path_z=path.poses[-1].pose.position.z if path and path.poses else None,
                              risk=state['risk'].data if 'risk' in state else None)
                item['samples'].append(sample)
                if error < .30:
                    stable = stable or time.monotonic()
                    if time.monotonic()-stable > 2:
                        item.update(arrived=True, error=error, wall_seconds=time.monotonic()-started)
                        print(json.dumps({k:v for k,v in item.items() if k != 'samples'}), flush=True)
                        break
                else:
                    stable = None
                time.sleep(.5)
            if not item['arrived']:
                print('FAILED', label, item['samples'][-1] if item['samples'] else 'no fresh odometry', flush=True)
                break
            if label == 'initial':
                # Let the demo finish its 3s arrival + 8s observation window
                # before a second goal changes the finite report's goal topic.
                time.sleep(12)
    finally:
        Path(args.output).write_text(json.dumps(results, indent=2))
    if not all(r['arrived'] for r in results) or len(results) != len(targets):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
