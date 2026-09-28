#!/usr/bin/env python3
"""Replay a blocked warehouse snapshot on isolated ROS; never commands a UAV."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--goal', type=float, nargs=3, default=[14., -.15, 1.64])
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    import rosbag
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import PointCloud2
    package = Path(__file__).resolve().parents[1]
    messages = {}
    with rosbag.Bag(args.bag) as bag:
        for topic, msg, stamp in bag.read_messages():
            messages[topic] = msg
    nodes, streams = [], []
    with tempfile.TemporaryDirectory(prefix='planner-snapshot-') as folder:
        def start(command):
            stream = open(Path(folder)/('%d.log' % len(nodes)), 'w')
            streams.append(stream)
            nodes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                          start_new_session=True))
        try:
            start(['roscore', '-p', str(port)])
            deadline = time.monotonic()+15
            while not rosgraph.is_master_online():
                if time.monotonic() > deadline:
                    raise RuntimeError('master startup timed out')
                time.sleep(.1)
            rospy.init_node('snapshot_regression', disable_signals=True)
            pubs = {t: rospy.Publisher(t, cls, queue_size=1, latch=True) for t, cls in [
                ('/uav1/fastlio/odom', Odometry), ('/uav1/stable_static_map', PointCloud2),
                ('/uav1/local_static_map', PointCloud2), ('/uav1/local_free_space', PointCloud2)]}
            goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
            result, subs = {}, []
            for margin in (4, 8, 12, 0):
                name = 'snapshot_planner_%d' % margin
                config = yaml.safe_load((package/'config/path_planner.yaml').read_text())
                config.update(yaml.safe_load((package/'config/warehouse_navigation.yaml').read_text())['static_path_planner'])
                config.update(planning_margin=margin or 4, max_planning_margin=margin or 12,
                              path_topic='/snapshot/%d/global' % margin,
                              local_path_topic='/snapshot/%d/local' % margin,
                              waypoint_topic='/snapshot/%d/waypoint' % margin)
                rospy.set_param('/'+name, config)
                start(['rosrun', 'fastlio_bridge', 'static_path_planner_node', '__name:='+name])
                subs.append(rospy.Subscriber(config['local_path_topic'], RosPath,
                            lambda m, key=margin: result.__setitem__(key, m), queue_size=1))
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                for t, pub in pubs.items():
                    msg = messages[t]
                    msg.header.stamp = rospy.Time.now()
                    pub.publish(msg)
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = args.goal
                goal.pose.orientation.w = 1
                goal_pub.publish(goal)
                time.sleep(.25)
            print(json.dumps({k: [[p.pose.position.x,p.pose.position.y] for p in m.poses]
                              for k,m in result.items()}, indent=2))
            if len(result) != 4:
                raise RuntimeError('missing planner output')
            assert len(result[4].poses) == 1, 'original fixed bounds should reproduce blockage'
            assert len(result[8].poses) > 1 and len(result[12].poses) > 1
            assert len(result[0].poses) > 1, 'adaptive bounds must recover this snapshot'
        finally:
            for p in reversed(nodes):
                if p.poll() is None:
                    os.killpg(p.pid, signal.SIGINT)
                    try:
                        p.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait()
            for stream in streams:
                stream.close()


if __name__ == '__main__':
    main()
