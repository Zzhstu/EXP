#!/usr/bin/env python3
"""Deterministic room + moving obstacle for ROS pipeline smoke testing."""

import math
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


class SyntheticScene:
    def __init__(self):
        self.cloud_pub = rospy.Publisher(
            "/uav1/fastlio/cloud_map", PointCloud2, queue_size=2)
        self.odom_pub = rospy.Publisher(
            "/uav1/fastlio/odom", Odometry, queue_size=10)
        self.start = rospy.Time.now()
        self.rate = rospy.Rate(20)
        self.static_points = self.make_room()

    @staticmethod
    def make_room():
        points = []
        # Floor is outside dynamic detector's relative-z gate at flight z=1 m.
        for ix in range(-20, 21):
            for iy in range(-20, 21):
                if ix % 2 == 0 and iy % 2 == 0:
                    points.append((0.2 * ix, 0.2 * iy, 0.0))
        # Two walls and a static pillar.
        for i in range(-20, 21):
            for k in range(0, 11):
                points.append((4.0, 0.2 * i, 0.2 * k))
                points.append((0.2 * i, 4.0, 0.2 * k))
        for ix in range(-2, 3):
            for iy in range(-2, 3):
                for iz in range(0, 8):
                    points.append((2.0 + 0.08 * ix,
                                   1.5 + 0.08 * iy,
                                   0.4 + 0.12 * iz))
        return points

    @staticmethod
    def obstacle(center_x, center_y):
        points = []
        for ix in range(-3, 4):
            for iy in range(-3, 4):
                for iz in range(-5, 6):
                    if (ix + iy + iz) % 3 == 0:
                        points.append((center_x + 0.07 * ix,
                                       center_y + 0.07 * iy,
                                       1.0 + 0.10 * iz))
        return points

    def publish(self):
        elapsed = (rospy.Time.now() - self.start).to_sec()
        # Keep the first two seconds fully static. The detector config used by
        # synthetic_test.launch shortens background warm-up to 20 frames.
        points = list(self.static_points)
        if elapsed > 2.0:
            center_x = -1.5 + 0.45 * (elapsed - 2.0)
            center_y = 0.5 + 0.25 * math.sin(0.7 * elapsed)
            points.extend(self.obstacle(center_x, center_y))

        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "uav1/base_link"
        odom.pose.pose.position.z = 1.0
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.odom_pub.publish(odom)

        header = Header(stamp=now, frame_id="map")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def spin(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("synthetic_dynamic_scene")
    SyntheticScene().spin()
