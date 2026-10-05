#!/usr/bin/env python3
import sys
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from mavros_msgs.msg import ExtendedState
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from startup_checks import GroundPoseWindow,ground_state_fresh
from warehouse_explorer import Explorer
import run_warehouse_exploration as runner


class StartupTest(unittest.TestCase):
    def test_wall_guard_bounds_mission_without_sim_clock(self):
        self.assertEqual(runner.mission_deadline(100.,0.,4800.),(4900.,'wall_guard_timeout_partial'))
        self.assertEqual(runner.mission_deadline(100.,30.,4800.),(130.,'explicit_test_timeout_partial'))
        self.assertEqual(runner.mission_deadline(100.,5000.,4800.),(4900.,'wall_guard_timeout_partial'))
        for test,budget in ((float('nan'),4800.),(0.,float('inf')),(0.,0.),(-1.,100.)):
            with self.assertRaises(ValueError): runner.mission_deadline(100.,test,budget)

    def test_runner_chooses_sim_clock_before_initializing_ros(self):
        # The runner starts before Gazebo, so setting use_sim_time later in
        # Gazebo's launch cannot change its already-initialized rospy clock.
        with tempfile.TemporaryDirectory() as folder, \
             patch.dict(os.environ,ASTRA_ROOT=folder,PX4_DIR=folder,PEDSIM_WS=folder), \
             patch.object(sys,'argv',['runner','--headless']), \
             patch.object(runner.rosgraph,'is_master_online',side_effect=[False,True]), \
             patch.object(runner,'process_running',return_value=False), \
             patch.object(runner,'CaseRunner'), \
             patch.object(runner.rospy,'set_param') as set_param, \
             patch.object(runner.signal,'signal'):
            def initialized(*args,**kwargs):
                set_param.assert_called_once_with('/use_sim_time',True)
                raise KeyboardInterrupt # stop before Gazebo or flight startup
            with patch.object(runner.rospy,'init_node',side_effect=initialized):
                self.assertEqual(runner.main(),0)

    def test_slow_sim_landed_status_and_true_timeouts(self):
        msg=ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        msg.header.stamp=rospy.Time(10)
        self.assertTrue(ground_state_fresh((msg,20),22.5,11.2))
        self.assertFalse(ground_state_fresh((msg,20),26,11.2))
        self.assertFalse(ground_state_fresh((msg,20),22,13))
        self.assertFalse(ground_state_fresh((msg,20),22,9))
    def test_transient_does_not_lock_old_ground(self):
        w=GroundPoseWindow()
        for i in range(20):
            self.assertFalse(w.update(1+i*.1,(0,0,-1.87+i*.1)))
        for i in range(45):
            ready=w.update(3+i*.1,(0,0,0))
        self.assertTrue(ready)

    def test_negative_stable_coordinate_is_legal(self):
        w=GroundPoseWindow()
        for i in range(45): ready=w.update(1+i*.1,(0,0,-2))
        self.assertTrue(ready)

    def test_duplicate_samples_do_not_count_or_erase_window(self):
        w=GroundPoseWindow()
        for _ in range(100): self.assertFalse(w.update(1.,(0,0,0)))
        for i in range(1,45):
            ready=w.update(1+i*.1,(0,0,0))
            self.assertFalse(w.update(1+i*.1,(0,0,0)))
        self.assertTrue(ready)

    def test_gap_reset_nan_and_airborne_cannot_be_ground(self):
        for bad in ('gap','reset','nan','air'):
            w=GroundPoseWindow()
            for i in range(45): w.update(1+i*.1,(0,0,0))
            if bad=='gap': ready=w.update(9,(0,0,0))
            elif bad=='reset': ready=w.update(1,(0,0,0))
            elif bad=='nan': ready=w.update(5.5,(0,0,float('nan')))
            else: ready=w.update(5.5,(0,0,0),False)
            self.assertFalse(ready,bad)

    @patch('warehouse_explorer.rospy.Service')
    @patch('warehouse_explorer.rospy.Subscriber')
    @patch('warehouse_explorer.rospy.Publisher')
    @patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d)
    @patch('warehouse_explorer.time.monotonic',return_value=20.)
    @patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(10))
    def test_explorer_requires_fresh_true_controller_confirmation(self,*mocks):
        n=Explorer()
        odom=Odometry(); odom.header.stamp=rospy.Time(10)
        odom.pose.pose.position.z=1.2
        pose=PoseStamped(); pose.header.stamp=rospy.Time(10)
        n.inputs={k:(PointCloud2(),20.) for k in ('map','free','scan')}
        n.inputs.update(odom=(odom,20.),mavros=(pose,20.))
        for m,_ in n.inputs.values(): m.header.stamp=rospy.Time(10)
        n.home=-1.87 # regression: old apparent height must NOT release flight
        n.publish=Mock()
        for state in (None,(False,20.),(True,16.)):
            n.takeoff_state=state
            n.tick()
            self.assertIsNone(n.altitude)
            n.publish.assert_called_with('WAIT_TAKEOFF_CONFIRMATION')
        n.receive_takeoff(Bool(data=True))
        n.tick()
        self.assertEqual(n.altitude,1.2)
        self.assertEqual(n.publish.call_args[0][0],'OBSERVING')


if __name__=='__main__': unittest.main()
