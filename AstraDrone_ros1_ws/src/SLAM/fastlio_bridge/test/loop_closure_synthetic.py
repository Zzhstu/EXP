#!/usr/bin/env python3
"""Deterministic smoke test input for loop_closure_backend_node.

Publishes a square revisit with deliberate odometry drift. The registered cloud
is generated exactly as a drifting LIO front end would publish it: true local
measurements transformed by the drifting odometry pose.
"""

import math
import random

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


def square_trajectory(samples_per_side=12, side=8.0):
    poses = []
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    for segment in range(4):
        x0, y0 = corners[segment]
        x1, y1 = corners[segment + 1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        for step in range(samples_per_side):
            ratio = float(step) / samples_per_side
            poses.append((x0 + ratio * (x1 - x0),
                          y0 + ratio * (y1 - y0), yaw))
    poses.append((0.0, 0.0, 0.0))
    return poses


def environment_points():
    random.seed(7)
    points = []
    # Asymmetric walls, pillars and elevated points avoid rotational ambiguity.
    for _ in range(4500):
        wall = random.randrange(4)
        along = random.uniform(-3.0, 11.0)
        height = random.uniform(-1.0, 3.0)
        if wall == 0:
            points.append((along, -3.0, height))
        elif wall == 1:
            points.append((11.0, along, height))
        elif wall == 2:
            points.append((along, 11.0, height))
        else:
            points.append((-3.0, along, height))
    for center_x, center_y in [(2.0, 1.5), (6.5, 2.5), (7.0, 7.0), (1.5, 6.0)]:
        for _ in range(450):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(0.0, 0.35)
            points.append((center_x + radius * math.cos(angle),
                           center_y + radius * math.sin(angle),
                           random.uniform(-0.8, 2.8)))
    return points


def main():
    rospy.init_node("loop_closure_synthetic")
    odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=5)
    cloud_pub = rospy.Publisher("/test/cloud", PointCloud2, queue_size=2)
    poses = square_trajectory()
    landmarks = environment_points()
    rate = rospy.Rate(8.0)
    rospy.sleep(1.0)
    initial_stamp = rospy.Time.now()

    for index, (true_x, true_y, true_yaw) in enumerate(poses):
        if rospy.is_shutdown():
            return
        # Increasing translation/yaw bias creates a visible end-point drift.
        drift_x = 0.025 * index
        drift_y = -0.010 * index
        drift_yaw = 0.0025 * index
        raw_x = true_x + drift_x
        raw_y = true_y + drift_y
        raw_yaw = true_yaw + drift_yaw
        stamp = initial_stamp + rospy.Duration(float(index))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = raw_x
        odom.pose.pose.position.y = raw_y
        odom.pose.pose.position.z = 1.5
        quaternion = quaternion_from_euler(0.0, 0.0, raw_yaw)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        cosine_true = math.cos(true_yaw)
        sine_true = math.sin(true_yaw)
        cosine_raw = math.cos(raw_yaw)
        sine_raw = math.sin(raw_yaw)
        cloud = []
        for world_x, world_y, world_z in landmarks:
            dx = world_x - true_x
            dy = world_y - true_y
            if dx * dx + dy * dy > 14.0 * 14.0:
                continue
            # True world -> body measurement.
            local_x = cosine_true * dx + sine_true * dy
            local_y = -sine_true * dx + cosine_true * dy
            local_z = world_z - 1.5
            # Body measurement -> drifting LIO world registration.
            registered_x = raw_x + cosine_raw * local_x - sine_raw * local_y
            registered_y = raw_y + sine_raw * local_x + cosine_raw * local_y
            cloud.append((registered_x, registered_y, local_z + 1.5, 1.0))

        header = Header(stamp=stamp, frame_id="map")
        cloud_message = point_cloud2.create_cloud(header, [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ], cloud)
        odom_pub.publish(odom)
        cloud_pub.publish(cloud_message)
        rate.sleep()

    # Keep publishing the final revisit so timers finish ICP and map rebuild.
    for _ in range(32):
        odom.header.stamp = rospy.Time.now()
        cloud_message.header.stamp = odom.header.stamp
        odom_pub.publish(odom)
        cloud_pub.publish(cloud_message)
        rate.sleep()


if __name__ == "__main__":
    main()
