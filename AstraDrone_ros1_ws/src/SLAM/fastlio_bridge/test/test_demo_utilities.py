#!/usr/bin/env python3
"""Offline regression: PCD exactness and actor ownership/deadman. No ROS master."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from pedsim_msgs.msg import AgentState, AgentStates

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from save_navigation_map import write_snapshot
from pedestrian_control import PedestrianRelay


class DemoUtilities(unittest.TestCase):
    @patch('pedestrian_control.rospy.Timer')
    @patch('pedestrian_control.rospy.Subscriber')
    @patch('pedestrian_control.rospy.Publisher')
    @patch('pedestrian_control.rospy.Time.now')
    def test_completed_crossing_freezes_both_actors(self, clock, pub, sub, timer):
        relay = PedestrianRelay()
        msg = AgentStates()
        for i in (0, 1):
            a = AgentState(id=i)
            a.pose.position.x = 3.8
            a.pose.position.y = 2.5
            msg.agent_states.append(a)
        for t in (1, 2, 3, 4, 5, 6):
            clock.return_value = rospy.Time(t)
            relay.receive(msg)
        self.assertTrue(relay.ready)
        self.assertEqual(set(relay.manual), {0, 1})
        msg.agent_states[1].pose.position.x = 4.4
        relay.receive(msg)
        self.assertEqual(relay.manual[1].pose.position.x, 3.8)

    def test_binary_pcd_exact_and_empty_guard(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'map.pcd'
            header = Header(stamp=rospy.Time(12, 345), frame_id='map')
            cloud = point_cloud2.create_cloud_xyz32(header, [(1, 2, 3), (-1, 0, .5), (float('nan'), 0, 0)])
            meta = write_snapshot(cloud, path, '/uav1/fastlio/cloud_map')
            data = path.read_bytes()
            xyz = np.frombuffer(data.split(b'DATA binary\n')[1], dtype='<f4').reshape(-1, 3)
            np.testing.assert_array_equal(xyz, [[1, 2, 3], [-1, 0, .5]])
            self.assertEqual(meta['sha256'], hashlib.sha256(data).hexdigest())
            self.assertEqual(meta, json.loads(path.with_suffix('.json').read_text()))
            with self.assertRaises(ValueError):
                write_snapshot(point_cloud2.create_cloud_xyz32(header, []), path, 'test')
            self.assertEqual(path.read_bytes(), data)

    @patch('pedestrian_control.time.monotonic', return_value=10.)
    @patch('pedestrian_control.rospy.Time.now', return_value=rospy.Time.from_sec(5.04))
    def test_actor_takeover_deadman_and_no_teleport(self, clock, wall):
        relay = PedestrianRelay.__new__(PedestrianRelay)
        relay.lock = threading.RLock()
        relay.pub = Mock()
        relay.ready = True
        relay.source = AgentStates()
        for i in (0, 1):
            a = AgentState(id=i)
            a.pose.position.x = 3.8
            a.pose.position.y = 2.5
            a.pose.orientation.w = 1
            relay.source.agent_states.append(a)
        relay.manual, relay.commands = {}, {}
        relay.source_time, relay.last_time = 10., 5.
        cmd = Twist()
        cmd.linear.y = .6
        relay.command(cmd, 0)
        relay.tick(None)
        out = relay.pub.publish.call_args[0][0]
        self.assertAlmostEqual(out.agent_states[0].pose.position.y, 2.524)
        self.assertEqual(out.agent_states[1].pose.position.y, 2.5)
        wall.return_value = 11.
        clock.return_value = rospy.Time.from_sec(5.08)
        relay.tick(None)
        out = relay.pub.publish.call_args[0][0]
        self.assertEqual(out.agent_states[0].twist.linear.y, 0)
        self.assertAlmostEqual(out.agent_states[0].pose.position.y, 2.524)
        self.assertEqual(relay.source.agent_states[0].pose.position.y, 2.5)


if __name__ == '__main__':
    unittest.main()
