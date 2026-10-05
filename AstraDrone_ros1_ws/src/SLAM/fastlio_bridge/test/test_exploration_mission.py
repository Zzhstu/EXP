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
    def ready(self):
        with patch('warehouse_explorer.rospy.Service'),patch('warehouse_explorer.rospy.Subscriber'),\
                patch('warehouse_explorer.rospy.Publisher'),patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d):
            n=Explorer()
        n.grid=ExplorationGrid([-6,-6,6,6])
        n.grid.known[:]=True;n.grid.update_obstacles([])
        n.inspected=np.zeros(n.grid.shape,bool)
        n.altitude=1.2;n.home_xy=(2.1,2.1)
        odom=Odometry();odom.pose.pose.position.x=odom.pose.pose.position.y=2.1
        odom.pose.pose.position.z=1.2
        n.inputs={k:(PointCloud2(),20.) for k in ('map','free','scan')}
        n.inputs.update(odom=(odom,20.),mavros=(PoseStamped(),20.))
        for msg,_ in n.inputs.values(): msg.header.stamp=rospy.Time(100)
        n.cloud=Mock(return_value=np.empty((0,3)));n.publish=Mock()
        n.p=lambda k,d: False if k=='require_takeoff_confirmation' else 0 if k=='growth_margin' else d
        return n

    def tick_at_100(self,n):
        with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)),\
                patch('warehouse_explorer.time.monotonic',return_value=20.):
            n.tick()

    def test_low_gain_moves_to_inspection_not_fake_full_coverage(self):
        n=self.ready();n.goals_reached=3
        n.progress.update(0,int(n.grid.known.sum()),0,0)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'INSPECTING')
        self.assertFalse(n.exploration_complete)

    def test_convergence_completion_requires_surface_quality_and_no_failures(self):
        for failed,qualified,expected in ((False,1.,True),(True,1.,False),(False,.5,False)):
            n=self.ready();n.phase='INSPECTING'
            n.progress.update(0,int(n.grid.known.sum()),0,0)
            n.surface.deficits=Mock(return_value=(np.empty((0,3)),dict(observed_surface_quality_ratio=qualified)))
            if failed: n.failed_visits[(20,20)]=2
            self.tick_at_100(n)
            self.assertEqual(n.exploration_complete,expected)
            if expected: self.assertEqual(n.finish_reason,'observed_reachable_converged')

    def test_wall_budget_returns_partial_not_complete(self):
        n=self.ready();n.active_wall=0.;n.active_sim=0.
        old=n.p;n.p=lambda k,d: 10. if k=='max_mission_wall_seconds' else old(k,d)
        self.tick_at_100(n)
        self.assertEqual(n.finish_reason,'wall_budget_partial')
        self.assertFalse(n.exploration_complete)

    def test_inspection_resumes_when_new_frontier_is_visible(self):
        n=self.ready();n.phase='INSPECTING'
        n.grid.known[:,50:]=False
        n.progress.update(0,int(n.grid.known.sum()),0,0)
        n.surface.inspection_candidates=Mock(return_value=(np.zeros(n.grid.shape,bool),np.zeros(n.grid.shape)))
        self.tick_at_100(n)
        self.assertGreater(n.useful_frontier_count,0)
        self.assertEqual(n.phase,'EXPLORING')
        self.assertEqual(n.publish.call_args[0][0],'RESUME_EXPLORATION')
        self.assertFalse(n.progress.samples)
        self.assertFalse(n.exploration_complete)

    def test_unqualified_inspection_view_is_not_permanently_hidden(self):
        n=self.ready();n.phase='INSPECTING';cell=(22,24)
        mask=np.zeros(n.grid.shape,bool);mask[cell]=True
        gain=np.zeros(n.grid.shape);gain[cell]=1.
        n.surface.inspection_candidates=Mock(return_value=(mask,gain))
        n.surface.deficits=Mock(return_value=(np.array([[1.,1.,1.]]),dict(observed_surface_quality_ratio=.5)))
        n.inspection_visited=[(cell,70.)]
        self.tick_at_100(n)
        self.assertEqual(n.inspection_visited,[])
        self.assertEqual(n.target,cell)
        self.assertFalse(n.exploration_complete)

    def test_grid_capacity_returns_on_old_grid_not_unknown(self):
        n=self.ready();n.update_observed_window=Mock(return_value=False)
        with patch('warehouse_explorer.rospy.logerr_throttle'):
            self.tick_at_100(n)
        self.assertTrue(n.grid_capacity_limited)
        self.assertEqual(n.finish_reason,'map_capacity_partial')
        self.assertFalse(n.exploration_complete)

    def test_unsafe_start_without_progress_fails_explicitly(self):
        n=self.ready()
        n.unsafe_since=50.
        n.unsafe_position=(2.1,2.1)
        n.grid.update_obstacles=Mock(side_effect=lambda points: n.grid.safe.fill(False))
        n.grid.reconnect=Mock(return_value=None)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'STALLED_UNSAFE')
        self.assertEqual(n.finish_reason,'unsafe_start_no_progress')
        self.assertEqual(n.publish.call_args[0][0],'STALLED_UNSAFE')
        self.assertFalse(n.exploration_complete)

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

    def test_window_growth_shifts_index_based_mission_memory(self):
        n=self.node()
        old_cell=(20,30)
        n.target=old_cell
        n.visited=[(old_cell,1.)]
        n.blacklist=[(old_cell,2.)]
        n.failed_visits={old_cell:1}
        n.inspected[old_cell]=True
        pads=n.grid.grow_to_include([(-10.,-10.)],margin=2.,chunk=2.)
        self.assertIsNotNone(pads)
        n.shift_grid_indices(pads)
        shifted=(old_cell[0]+pads[0],old_cell[1]+pads[2])
        self.assertEqual(n.target,shifted)
        self.assertEqual(n.visited[0][0],shifted)
        self.assertEqual(n.blacklist[0][0],shifted)
        self.assertEqual(n.failed_visits[shifted],1)
        self.assertTrue(n.inspected[shifted])

    def test_explorer_window_growth_uses_pose_and_fresh_free_points(self):
        n=self.node()
        old_cell=n.grid.cell((2.,5.))
        old_xy=n.grid.xy(old_cell)
        n.target=old_cell
        n.visited=[]
        n.blacklist=[]
        n.failed_visits={}
        observed=np.array([[-1.,5.,1.]])
        self.assertTrue(n.update_observed_window((-1.,5.),observed))
        np.testing.assert_allclose(n.grid.xy(n.target),old_xy,atol=1e-12)
        self.assertGreater(n.target[1],old_cell[1])
        self.assertTrue(n.grid.known[n.grid.cell((-1.,5.))])
        self.assertEqual(n.target,n.grid.cell(old_xy))

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
        n.grid=ExplorationGrid([-6,-6,6,6],.2,.8,.4)
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
        n.p=lambda k,d: (0. if k=='growth_margin' else
                         1. if k=='no_frontier_seconds' else
                         False if k=='require_takeoff_confirmation' else d)
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
