#!/usr/bin/env python3
"""Mission and inspection regressions; no ROS master or simulator required."""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import Explorer
from exploration_grid import ExplorationGrid


class MissionTest(unittest.TestCase):
    def node(self):
        n=Explorer.__new__(Explorer)
        n.p=lambda k,d:d
        n.grid=ExplorationGrid([0,0,12,10],.2,.8,.4)
        n.grid.known[:]=True
        n.grid.update_obstacles([])
        n.home_xy=(2.1,2.1)
        n.home_confirmed=False
        n.inspected=np.zeros(n.grid.shape,bool)
        n.publish=Mock()
        n.save=Mock(return_value=True)
        n.begin_return('test_partial',10.)
        return n

    def test_return_requires_continuous_dwell(self):
        n=self.node()
        start=n.grid.cell(n.home_xy)
        d,p=n.grid.search(start)
        n.return_tick(11,n.home_xy,start,d,p)
        self.assertFalse(n.home_confirmed)
        self.assertEqual(n.publish.call_args[0][0],'HOME_SETTLE')
        n.return_tick(14,(3.,2.1),n.grid.cell((3.,2.1)),d,p)
        self.assertIsNone(n.home_since)
        n.return_tick(15,n.home_xy,start,d,p)
        n.return_tick(19,n.home_xy,start,d,p)
        n.save.assert_not_called()
        n.return_tick(20,n.home_xy,start,d,p)
        self.assertTrue(n.home_confirmed)
        self.assertEqual(n.phase,'COMPLETE')

    def test_failed_save_cannot_complete(self):
        n=self.node()
        n.save.return_value=False
        start=n.grid.cell(n.home_xy)
        d,p=n.grid.search(start)
        n.return_tick(11,n.home_xy,start,d,p)
        n.return_tick(20,n.home_xy,start,d,p)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.publish.call_args[0][0],'HOME_SAVE_FAILED')

    def test_return_never_plans_through_wall(self):
        n=self.node()
        n.grid.update_obstacles([(5,y) for y in np.arange(0,10,.1)])
        position=(8.,2.1)
        start=n.grid.cell(position)
        d,p=n.grid.search(start)
        n.return_tick(11,position,start,d,p)
        self.assertEqual(n.publish.call_args[0][0],'RETURN_BLOCKED')
        self.assertFalse(n.home_confirmed)

    def test_return_timeout_is_failure(self):
        n=self.node()
        pos=(8.,2.1)
        start=n.grid.cell(pos)
        d,p=n.grid.search(start)
        n.return_tick(1211,pos,start,d,p)
        self.assertEqual(n.phase,'RETURN_FAILED')
        n.save.assert_not_called()

    def test_return_resets_old_exploration_goal(self):
        n=self.node()
        n.target=(30,30)
        n.settle_until=1000.
        n.begin_return('operator_request',15.)
        self.assertIsNone(n.target)
        self.assertEqual(n.settle_until,0.)
        self.assertEqual(n.finish_reason,'operator_request')

    def test_inspection_does_not_cross_shelf(self):
        n=self.node()
        n.grid.update_obstacles([(5,y) for y in np.arange(0,10,.1)])
        n.mark_inspected((4.,5.))
        self.assertTrue(n.inspected[n.grid.cell((3.8,5.))])
        self.assertFalse(n.inspected[n.grid.cell((6.,5.))])

    @patch('warehouse_explorer.rospy.Service')
    @patch('warehouse_explorer.rospy.Subscriber')
    @patch('warehouse_explorer.rospy.Publisher')
    @patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d)
    @patch('warehouse_explorer.time.monotonic',return_value=20.)
    def test_normal_mission_exhaustion_returns_without_goal_limit(self,*mocks):
        n=Explorer()
        n.grid=ExplorationGrid([0,0,12,10],.2,.8,.4)
        n.inspected=np.zeros(n.grid.shape,bool)
        n.grid.known[:]=True
        n.home=0.
        n.altitude=1.2
        odom=Odometry()
        odom.pose.pose.position.x=odom.pose.pose.position.y=2.1
        odom.pose.pose.position.z=1.2
        n.inputs={k:(PointCloud2(),20.) for k in ('map','free','scan')}
        n.inputs.update(odom=(odom,20.),mavros=(PoseStamped(),20.))
        n.cloud=Mock(return_value=np.empty((0,3)))
        n.publish=Mock()
        n.save=Mock(return_value=True)
        n.p=lambda k,d: 1. if k=='no_frontier_seconds' else d
        for stamp,state in ((10,'WAIT_FRONTIER'),(12,'START_INSPECTION'),
                            (13,'WAIT_FRONTIER'),(15,'RETURNING'),
                            (16,'HOME_SETTLE'),(22,'COMPLETE')):
            for msg,_ in n.inputs.values():
                msg.header.stamp=rospy.Time(stamp)
            with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(stamp)):
                n.tick()
            self.assertEqual(n.publish.call_args[0][0],state)
        self.assertTrue(n.home_confirmed)
        self.assertEqual(n.finish_reason,'reachable_viewpoints_exhausted')


if __name__=='__main__':
    unittest.main()
