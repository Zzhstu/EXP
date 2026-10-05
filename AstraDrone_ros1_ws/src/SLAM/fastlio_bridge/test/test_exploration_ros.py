#!/usr/bin/env python3
"""Message layout and fail-closed tests, no ROS master needed."""
from pathlib import Path
import struct
import sys
import threading
import unittest
from unittest.mock import Mock,patch
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import xyz,Explorer
from exploration_coverage import ProgressWindow


class MessageTest(unittest.TestCase):
    def test_organized_padded_big_endian(self):
        msg=PointCloud2(height=2,width=1,is_bigendian=True,point_step=16,row_step=20)
        msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(('x','y','z'))]
        msg.data=struct.pack('>fff',1,2,3)+bytes(8)+struct.pack('>fff',4,5,6)+bytes(8)
        np.testing.assert_array_equal(xyz(msg),[[1,2,3],[4,5,6]])

    def test_invalid_fields_rejected(self):
        with self.assertRaises(ValueError):
            xyz(PointCloud2())

    @patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(10))
    def test_missing_data_publishes_no_waypoint(self,clock):
        node=Explorer.__new__(Explorer)
        node.progress=ProgressWindow()
        node.inputs={}
        node.home=None
        node.lock=threading.Lock()
        node.p=lambda k,d:d
        node.publish=Mock()
        node.tick()
        node.publish.assert_called_once_with('WAIT_FRESH_DATA')

    @patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(10))
    @patch('warehouse_explorer.time.monotonic',return_value=20.)
    def test_stale_scan_publishes_no_waypoint(self,wall,clock):
        node=Explorer.__new__(Explorer)
        node.progress=ProgressWindow()
        node.inputs={name:(Mock(),19.) for name in ('odom','map','scan','free','mavros')}
        node.inputs['scan']=(Mock(),10.)
        node.home=0.
        node.lock=threading.Lock()
        node.p=lambda k,d:d
        node.publish=Mock()
        node.tick()
        node.publish.assert_called_once_with('WAIT_FRESH_DATA')

    @patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(10))
    @patch('warehouse_explorer.time.monotonic',return_value=20.)
    def test_republished_old_map_cannot_finish_home_dwell(self,wall,clock):
        node=Explorer.__new__(Explorer)
        node.progress=ProgressWindow()
        node.inputs={name:(PointCloud2(),19.) for name in ('map','scan','free')}
        node.inputs.update(odom=(Odometry(),19.),mavros=(PoseStamped(),19.))
        for msg,_ in node.inputs.values():
            msg.header.stamp=rospy.Time(10)
        node.inputs['map'][0].header.stamp=rospy.Time(5)
        node.home=0.
        node.home_since=1.
        node.lock=threading.Lock()
        node.p=lambda k,d:d
        node.publish=Mock()
        node.tick()
        node.publish.assert_called_once_with('WAIT_FRESH_DATA')
        self.assertIsNone(node.home_since)


if __name__=='__main__':
    unittest.main()
