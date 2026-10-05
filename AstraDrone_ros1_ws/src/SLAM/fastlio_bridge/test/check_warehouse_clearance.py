#!/usr/bin/env python3
"""Isolated real ROS-node regression; no PX4, no flight commands, no Gazebo.

Run after sourcing ROS and this workspace. Uses a separate master on a free
port. Synthetic walls test parameters/algorithms, not real vehicle safety.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import yaml
import numpy as np


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, UInt8
    package = Path(__file__).resolve().parents[1]
    config = package / 'config'
    profile = yaml.safe_load((config/'warehouse_navigation.yaml').read_text())
    nodes, streams, received = [], [], {}
    with tempfile.TemporaryDirectory(prefix='warehouse-clearance-') as logdir:
        def start(args):
            stream = open(Path(logdir)/('%d.log' % len(nodes)), 'w')
            streams.append(stream)
            process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            nodes.append(process)
        try:
            start(['roscore', '-p', str(port)])
            deadline = time.monotonic()+15
            while not rosgraph.is_master_online():
                if time.monotonic() > deadline:
                    raise RuntimeError('test master startup timed out')
                time.sleep(.1)
            rospy.init_node('warehouse_clearance_regression', disable_signals=True)
            pubs = {k: rospy.Publisher('/clearance_test/'+k, t, queue_size=1)
                    for k, t in [('odom', Odometry), ('goal', PoseStamped),
                                 ('map', PointCloud2), ('scan', PointCloud2),
                                 ('free', PointCloud2), ('local_map', PointCloud2)]}
            subs = []
            for name in ('standard', 'warehouse', 'warehouse_old_band'):
                planner = yaml.safe_load((config/'path_planner.yaml').read_text())
                risk = yaml.safe_load((config/'static_risk.yaml').read_text())
                if name.startswith('warehouse'):
                    planner.update(profile['static_path_planner'])
                    risk.update(profile['static_collision_risk'])
                if name == 'warehouse_old_band':
                    # Isolate height-band causality with identical XY margins.
                    planner.update(obstacle_min_relative_z=-.80, obstacle_max_relative_z=.80,
                                   route_switch_improvement=0.0)
                planner.update(map_topic='/clearance_test/map', local_map_topic='/clearance_test/local_map',
                               local_free_space_topic='/clearance_test/free', odom_topic='/clearance_test/odom',
                               goal_topic='/clearance_test/goal', path_topic='/clearance_test/'+name+'/path',
                               local_path_topic='/clearance_test/'+name+'/local',
                               waypoint_topic='/clearance_test/'+name+'/waypoint')
                risk.update(cloud_topic='/clearance_test/scan', odom_topic='/clearance_test/odom',
                            dynamic_objects_topic='/unused/objects')
                for kind, params, executable in [('planner', planner, 'static_path_planner_node'),
                                                  ('risk', risk, 'static_collision_risk_node')]:
                    nodename = 'clearance_'+name+'_'+kind
                    rospy.set_param('/'+nodename, params)
                    remaps = ['/uav1/static_collision_risk_level:=/clearance_test/'+name+'/risk'] if kind == 'risk' else []
                    start(['rosrun', 'fastlio_bridge', executable, '__name:='+nodename]+remaps)
                for kind, msgtype in [('path', RosPath), ('risk', UInt8)]:
                    key = name+'/'+kind
                    subs.append(rospy.Subscriber('/clearance_test/'+key, msgtype,
                        lambda m, k=key: received.__setitem__(k, m), queue_size=1))

            def publish_for(seconds, surface_distance, width=1.9, cloud=None,
                            start=(0., 0., 1.2), target=(0., 5., 1.2), free=(), overlay=()):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    if any(p.poll() is not None for p in nodes):
                        raise RuntimeError('test node exited')
                    stamp = rospy.Time.now()
                    h = Header(stamp=stamp, frame_id='map')
                    odom = Odometry(header=h)
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = start
                    odom.pose.pose.orientation.w = 1
                    goal = PoseStamped(header=h)
                    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target
                    goal.pose.orientation.w = 1
                    # A 1.90m corridor with walls extending beyond the planning
                    # bounds: bypass outside the synthetic corridor is impossible.
                    points = [(x, i*.05, 1.2) for x in (-width/2, width/2) for i in range(-200, 301)]
                    if cloud is not None:
                        points = cloud
                    scan = [(surface_distance, i*.02, 1.2+j*.02)
                            for i in range(-6, 7) for j in range(-2, 3)]
                    pubs['odom'].publish(odom)
                    pubs['goal'].publish(goal)
                    pubs['map'].publish(point_cloud2.create_cloud_xyz32(h, points))
                    pubs['free'].publish(point_cloud2.create_cloud_xyz32(h, free))
                    pubs['local_map'].publish(point_cloud2.create_cloud_xyz32(h, overlay))
                    pubs['scan'].publish(point_cloud2.create_cloud_xyz32(h, scan))
                    time.sleep(.05)

            publish_for(4, .85)
            old = len(received['standard/path'].poses)
            new = len(received['warehouse/path'].poses)
            assert old == 1, ('standard should hold', old)
            assert new >= 2, ('warehouse should plan', new)
            assert abs(received['warehouse/path'].poses[-1].pose.position.y-5) < .01
            assert received['standard/risk'].data == 1
            assert received['warehouse/risk'].data == 0
            checks = {'corridor_width_m': 1.9, 'standard_path_poses': old,
                      'warehouse_path_poses': new, 'risk_at_0.85m': [1, 0]}
            for distance, expected in [(.68, 1), (.72, 1), (.78, 0), (.60, 2)]:
                publish_for(1.0, distance)
                value = received['warehouse/risk'].data
                assert value == expected, (distance, value, expected)
                checks['warehouse_risk_at_%.2fm' % distance] = value
            # Preserve the negative boundary case: reducing safety margins is
            # not permission to force a route through every narrow opening.
            publish_for(1, .85, width=1.8)
            assert len(received['warehouse/path'].poses) == 1
            checks['warehouse_1.8m_boundary_case'] = 'blocked at this grid alignment'
            # Repeated RViz-style Z=0 goals must preserve the previous target
            # height, not ratchet each positive tracking error into a new goal.
            for actual_z in (1.27, 1.34, 1.41):
                publish_for(1, .85, start=(0., 0., actual_z), target=(0., 5., 0.))
                assert abs(received['warehouse/path'].poses[-1].pose.position.z-1.2) < 1e-6
            checks['repeated_2d_goal_altitude'] = 'held at 1.2m despite changing measured altitude'
            # Optional exact user map regression; never change the original PCD.
            usermap = package.parents[3]/'dynamic_demo_runs/ours_cangku_20260927_131318_final_map.pcd'
            if usermap.exists():
                xyz = np.frombuffer(usermap.read_bytes().split(b'DATA binary\n', 1)[1], dtype='<f4').reshape(-1, 3)
                publish_for(4, .85, cloud=xyz.tolist(), start=(5.75, 10.31, 1.83), target=(4.86, 6.78, 1.83))
                assert len(received['standard/path'].poses) == 1
                assert len(received['warehouse_old_band/path'].poses) == 1
                assert len(received['warehouse/path'].poses) > 1
                checks['user_final_map_replay'] = 'same XY settings: old band holds; new band finds route'
            else:
                checks['user_final_map_replay'] = 'SKIPPED: original PCD unavailable'
            wall = [(i*.05, 4., 1.2) for i in range(-60, 61)]
            publish_for(2, .85, cloud=wall, start=(-.25, 0., 1.2), target=(0., 8., 1.2))
            assert received['warehouse/path'].poses[1].pose.position.x < 0
            # Read the LOCAL route: global A* is allowed to propose alternatives.
            local_result = {}
            route_subs = [rospy.Subscriber('/clearance_test/'+name+'/local', RosPath,
                          lambda m, key=name: local_result.__setitem__(key,m), queue_size=1)
                          for name in ('warehouse','warehouse_old_band')]
            publish_for(2, .85, cloud=wall, start=(.25, 0., 1.2), target=(0., 8., 1.2))
            assert local_result['warehouse_old_band'].poses[1].pose.position.x > 0
            assert local_result['warehouse'].poses[1].pose.position.x < 0
            checks['route_hysteresis'] = 'small position change does not reverse valid detour'
            wall += [(-3.7, i*.05, 1.2) for i in range(20, 141)]
            publish_for(2, .85, cloud=wall, start=(.25, 0., 1.2), target=(0., 8., 1.2))
            assert local_result['warehouse'].poses[1].pose.position.x > 0
            checks['blocked_retained_route'] = 'new obstacle invalidates old route; switches immediately'
            wall = [(i*.05, 4., 1.2) for i in range(-60, 61)]
            nearby_free = [(i*.05, y, 1.2) for i in range(-20,21) for y in (3.2,4.8)]
            publish_for(2, .85, cloud=wall, target=(0., 8., 1.2), free=nearby_free)
            assert any(abs(p.pose.position.x) > 3 for p in local_result['warehouse'].poses)
            checks['free_halo'] = 'nearby free rays cannot erase a real wall or its inflation'
            above_free = [(i*.05, 4., 1.6) for i in range(-22,23)]
            publish_for(2, .85, cloud=wall, target=(0., 8., 1.2), free=above_free)
            assert any(abs(p.pose.position.x) > 3 for p in local_result['warehouse'].poses)
            checks['free_height'] = 'free space above a wall cannot erase lower occupancy'
            cleared = [(i*.05, 4., 1.2) for i in range(-22,23)]
            publish_for(2, .85, cloud=wall, target=(0., 8., 1.2), free=cleared)
            assert len(local_result['warehouse'].poses) == 2
            checks['matching_free_voxels'] = 'observed vanished wall reopens shortcut'
            publish_for(2, .85, cloud=wall, target=(0., 8., 1.2), free=cleared, overlay=wall)
            assert any(abs(p.pose.position.x) > 3 for p in local_result['warehouse'].poses)
            checks['current_returns_win'] = 'current occupied returns override free evidence'
            controller = yaml.safe_load((config/'astra_mavros_controller.yaml').read_text())
            controller.update(enable_control=False, auto_arm=False, auto_goal=False,
                              takeoff_height=0., prestream_seconds=0.,
                              state_topic='/controller_test/state',
                              mavros_pose_topic='/controller_test/pose',
                              odom_topic='/controller_test/odom',
                              final_goal_topic='/controller_test/final',
                              planner_waypoint_topic='/controller_test/waypoint',
                              risk_topic='/controller_test/risk',
                              avoidance_topic='/controller_test/avoid')
            rospy.set_param('/clearance_controller', controller)
            start(['rosrun','fastlio_bridge','mavros_avoidance_controller_node','__name:=clearance_controller'])
            cp = {k: rospy.Publisher('/controller_test/'+k,t,queue_size=1) for k,t in [
                ('state',State),('pose',PoseStamped),('odom',Odometry),('final',PoseStamped),
                ('waypoint',PoseStamped),('risk',UInt8),('avoid',TwistStamped)]}
            previews = []
            preview_sub = rospy.Subscriber('/clearance_controller/command_preview',TwistStamped,
                                            lambda m: previews.append(m),queue_size=1)
            for final_x in (5., .1):
                # Includes the new ground stability + takeoff dwell guard;
                # preview mode still does not publish any real FCU command.
                deadline = time.monotonic()+6
                while time.monotonic()<deadline:
                    h=Header(stamp=rospy.Time.now(),frame_id='map')
                    pose=PoseStamped(header=h);pose.pose.position.z=1.2;pose.pose.orientation.w=1
                    odom=Odometry(header=h);odom.pose.pose=pose.pose
                    wp=PoseStamped(header=h);wp.pose.position.x=.1;wp.pose.position.z=1.2
                    goal=PoseStamped(header=h);goal.pose.position.x=final_x;goal.pose.position.z=1.2
                    cp['state'].publish(State(connected=True,armed=True,mode='OFFBOARD'))
                    cp['pose'].publish(pose);cp['odom'].publish(odom)
                    cp['final'].publish(goal);cp['waypoint'].publish(wp)
                    cp['risk'].publish(UInt8(0));cp['avoid'].publish(TwistStamped(header=h))
                    time.sleep(.05)
                assert previews, 'missing controller preview'
                vx=previews[-1].twist.linear.x
                assert vx > .02 if final_x==5. else abs(vx)<1e-6, (final_x,vx)
            checks['corner_vs_final'] = '10cm intermediate corner still commanded; 10cm final goal hovers'
            print(json.dumps(checks, indent=2))
        except Exception:
            for path in Path(logdir).glob('*.log'):
                print(path.name, path.read_text(errors='replace')[-2000:])
            raise
        finally:
            for process in reversed(nodes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
            for stream in streams:
                stream.close()


if __name__ == '__main__':
    main()
