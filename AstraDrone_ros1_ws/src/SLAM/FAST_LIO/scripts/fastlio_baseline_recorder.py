#!/usr/bin/env python3
"""Record a start-aligned FAST-LIO trajectory against Gazebo model truth."""
import csv
import json
import math
import os

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import (concatenate_matrices, euler_from_quaternion,
                                inverse_matrix, quaternion_from_matrix,
                                quaternion_matrix, translation_matrix)


def pose_matrix(pose):
    """Return a homogeneous transform from a ROS Pose."""
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return concatenate_matrices(translation_matrix([pose.position.x, pose.position.y, pose.position.z]),
                                quaternion_matrix(q))


class BaselineRecorder:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris_mid360")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/fastlio_baselines"))
        self.tag = rospy.get_param("~tag", "long_corridor_baseline")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.truth_topic = rospy.get_param("~truth_topic", "/gazebo/model_states")
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = rospy.Time.now().to_sec()
        self.prefix = os.path.join(self.output_dir, "%s_%0.3f" % (self.tag, stamp))
        self.csv_file = open(self.prefix + ".csv", "w")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["stamp", "est_x", "est_y", "est_z", "gt_x", "gt_y", "gt_z",
                              "error_3d", "error_xy", "est_yaw", "gt_yaw", "yaw_error", "distance_m"])
        self.latest_truth = None
        self.alignment = None
        self.last_est = None
        self.distance = 0.0
        self.errors_3d = []
        self.errors_xy = []
        self.yaw_errors = []
        self.sub_truth = rospy.Subscriber(self.truth_topic, ModelStates, self.truth_callback, queue_size=10)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.on_shutdown(self.finish)
        rospy.loginfo("FAST-LIO baseline recorder: model=%s, CSV=%s.csv", self.model_name, self.prefix)

    def truth_callback(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(5.0, "Gazebo model '%s' not found; available: %s", self.model_name, ", ".join(msg.name))
            return
        self.latest_truth = (rospy.Time.now(), pose_matrix(msg.pose[index]))

    def odom_callback(self, msg):
        if self.latest_truth is None:
            return
        est = pose_matrix(msg.pose.pose)
        # FAST-LIO's camera_init origin is arbitrary.  Align only at the first
        # pair so following values are local drift, not a global-frame offset.
        if self.alignment is None:
            self.alignment = concatenate_matrices(est, inverse_matrix(self.latest_truth[1]))
            rospy.loginfo("Baseline alignment fixed at first odometry/truth pair.")
        gt = concatenate_matrices(self.alignment, self.latest_truth[1])
        delta = est[:3, 3] - gt[:3, 3]
        error_3d = float(math.sqrt(float(delta.dot(delta))))
        error_xy = float(math.hypot(delta[0], delta[1]))
        est_q = quaternion_from_matrix(est)
        gt_q = quaternion_from_matrix(gt)
        est_yaw = euler_from_quaternion(est_q)[2]
        gt_yaw = euler_from_quaternion(gt_q)[2]
        yaw_error = math.atan2(math.sin(est_yaw - gt_yaw), math.cos(est_yaw - gt_yaw))
        if self.last_est is not None:
            step = est[:3, 3] - self.last_est
            self.distance += float(math.sqrt(float(step.dot(step))))
        self.last_est = est[:3, 3].copy()
        self.errors_3d.append(error_3d)
        self.errors_xy.append(error_xy)
        self.yaw_errors.append(yaw_error)
        p_est, p_gt = est[:3, 3], gt[:3, 3]
        self.writer.writerow([msg.header.stamp.to_sec(), p_est[0], p_est[1], p_est[2], p_gt[0], p_gt[1], p_gt[2],
                              error_3d, error_xy, est_yaw, gt_yaw, yaw_error, self.distance])
        self.csv_file.flush()

    def finish(self):
        if not hasattr(self, "csv_file") or self.csv_file.closed:
            return
        self.csv_file.close()
        if not self.errors_3d:
            rospy.logwarn("No baseline samples written; check %s and %s.", self.odom_topic, self.truth_topic)
            return
        count = len(self.errors_3d)
        summary = {
            "model_name": self.model_name,
            "samples": count,
            "estimated_path_length_m": self.distance,
            "rmse_3d_m": math.sqrt(sum(e * e for e in self.errors_3d) / count),
            "rmse_xy_m": math.sqrt(sum(e * e for e in self.errors_xy) / count),
            "max_3d_m": max(self.errors_3d),
            "final_3d_m": self.errors_3d[-1],
            "final_xy_m": self.errors_xy[-1],
            "final_yaw_error_deg": math.degrees(self.yaw_errors[-1]),
        }
        with open(self.prefix + ".json", "w") as summary_file:
            json.dump(summary, summary_file, indent=2, sort_keys=True)
        rospy.loginfo("Baseline summary: %s", json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    rospy.init_node("fastlio_baseline_recorder")
    BaselineRecorder()
    rospy.spin()
