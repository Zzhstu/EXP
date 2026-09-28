#!/usr/bin/env python3
"""Score a dynamic-navigation demo using bagged Gazebo truth and ROS outputs."""

import argparse
import json
import math
import os
import sys

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def aligned_ate(times, estimates, truth_times, truth_positions):
    times = np.asarray(times)
    estimates = np.asarray(estimates)
    valid = (times >= truth_times[0]) & (times <= truth_times[-1])
    estimate = estimates[valid]
    times = times[valid]
    if len(estimate) < 3:
        return None, None, None
    truth = np.column_stack([np.interp(times, truth_times, truth_positions[:, axis])
                             for axis in range(3)])
    source_mean = estimate.mean(axis=0)
    truth_mean = truth.mean(axis=0)
    u, _, vt = np.linalg.svd((estimate - source_mean).T.dot(truth - truth_mean))
    rotation = vt.T.dot(u.T)
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T.dot(u.T)
    aligned = (estimate - source_mean).dot(rotation.T) + truth_mean
    error = np.linalg.norm(aligned - truth, axis=1)
    return float(np.sqrt(np.mean(error * error))), aligned, truth


def count_start_ghosts(cloud, initial_pedestrians, moved_pedestrians):
    if cloud is None or not initial_pedestrians or not moved_pedestrians:
        return None
    positions = [initial_pedestrians[name] for name in moved_pedestrians]
    count = 0
    for x, y, z in point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True):
        if any(.4 <= z-p[2] <= 2.1 and math.hypot(x-p[0], y-p[1]) <= .55 for p in positions):
            count += 1
    return count


def rotation(q):
    """Body-to-reference rotation; both Gazebo and ROS use xyzw quaternions."""
    values = np.asarray([q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid body quaternion: cannot align map and world ROIs')
    x, y, z, w = values/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def analyze(path, mode):
    gt_times, gt_positions = [], []
    gt_rotations, odom_rotations = [], []
    ped_tracks = {"0": [], "1": []}
    odom_times, odom_positions = [], []
    risk_times, risk_values = [], []
    goal = None
    map_cloud = None
    stable_cloud = None
    nav_path = None
    model_name = None
    with rosbag.Bag(path) as bag:
        for topic, msg, bag_time in bag.read_messages(topics=[
                "/gazebo/model_states", "/uav1/fastlio/odom",
                "/uav1/fused_collision_risk_level", "/move_base_simple/goal",
                "/uav1/fastlio/cloud_map", "/uav1/stable_static_map",
                "/uav1/planner/local_path"]):
            stamp = bag_time.to_sec()
            if topic == "/gazebo/model_states":
                if model_name not in msg.name:
                    matches = [name for name in msg.name if "iris_mid360" in name]
                    model_name = matches[0] if matches else None
                if model_name is None:
                    continue
                p = msg.pose[msg.name.index(model_name)].position
                gt_times.append(stamp)
                gt_positions.append((p.x, p.y, p.z))
                gt_rotations.append(msg.pose[msg.name.index(model_name)].orientation)
                for name in ped_tracks:
                    if name in msg.name:
                        person = msg.pose[msg.name.index(name)].position
                        ped_tracks[name].append((stamp, person.x, person.y, person.z))
            elif topic == "/uav1/fastlio/odom":
                odom_times.append(msg.header.stamp.to_sec() or stamp)
                p = msg.pose.pose.position
                odom_positions.append((p.x, p.y, p.z))
                odom_rotations.append(msg.pose.pose.orientation)
            elif topic == "/uav1/fused_collision_risk_level":
                risk_times.append(stamp)
                risk_values.append(int(msg.data))
            elif topic == "/move_base_simple/goal":
                goal = (stamp, msg.pose.position.x, msg.pose.position.y,
                        msg.pose.position.z)
            elif topic == "/uav1/fastlio/cloud_map":
                map_cloud = msg
            elif topic == "/uav1/stable_static_map":
                stable_cloud = msg
            elif topic == "/uav1/planner/local_path":
                nav_path = msg

    if len(gt_times) < 3 or len(odom_times) < 3:
        raise RuntimeError("缺少足够的 Gazebo 真值或 FAST-LIO 里程计")
    truth_times = np.asarray(gt_times)
    truth = np.asarray(gt_positions)
    # Gazebo publishes very often. One sample per 0.1s suffices for route
    # length and pedestrian clearance without giving faster runs more weight.
    keep = np.zeros(len(truth_times), dtype=bool)
    last_sample = -float("inf")
    for i, stamp in enumerate(truth_times):
        if stamp - last_sample >= 0.10:
            keep[i] = True
            last_sample = stamp
    sampled_times = truth_times[keep]
    sampled_truth = truth[keep]
    odom_times = np.asarray(odom_times)
    odom = np.asarray(odom_positions)
    ate, aligned, paired_truth = aligned_ate(
        odom_times, odom, truth_times, truth)

    moved = []
    initial = {}
    pedestrian_displacements = {}
    for name, track in ped_tracks.items():
        if not track:
            continue
        initial[name] = track[0][1:4]
        distance = math.hypot(track[-1][1] - track[0][1],
                              track[-1][2] - track[0][2])
        pedestrian_displacements[name] = round(distance, 3)
        if distance >= 1.5:
            moved.append(name)

    # With a nonzero spawn Gazebo world != navigation map. Use the first
    # associated BODY pose to transform pedestrian ROIs before counting map
    # points; comparing world coordinates directly would falsely report zero.
    ref = min(len(truth_times)-1, int(np.searchsorted(truth_times, odom_times[0])))
    map_from_world = rotation(odom_rotations[0]).dot(rotation(gt_rotations[ref]).T)
    ref_position = np.array([np.interp(odom_times[0], truth_times, truth[:, a]) for a in range(3)])
    translation = odom[0] - map_from_world.dot(ref_position)
    initial_map = {name: map_from_world.dot(p)+translation for name, p in initial.items()}

    minimum_center_xy = None
    if initial and goal is not None:
        # Interpolate each pedestrian on its own Gazebo timestamps. Report
        # center distance only; it is not a contact or mesh-clearance test.
        distances = []
        for name in initial:
            data = np.asarray(ped_tracks[name])
            valid = (sampled_times >= data[0, 0]) & (sampled_times <= data[-1, 0])
            px = np.interp(sampled_times[valid], data[:, 0], data[:, 1])
            py = np.interp(sampled_times[valid], data[:, 0], data[:, 2])
            after_goal = sampled_times[valid] >= goal[0]
            if np.any(after_goal):
                delta = sampled_truth[valid, :2][after_goal] - np.column_stack(
                    (px, py))[after_goal]
                distances.extend(np.linalg.norm(delta, axis=1).tolist())
        if distances:
            minimum_center_xy = float(min(distances))

    reached_time = None
    route_length = None
    end_goal_error = None
    if goal is not None:
        later = odom_times >= goal[0]
        goal_d = np.linalg.norm(odom[later, :2] - np.array(goal[1:3]), axis=1)
        later_times = odom_times[later]
        close_since = None
        for stamp, distance in zip(later_times, goal_d):
            if distance <= 0.30:
                close_since = stamp if close_since is None else close_since
                if stamp - close_since >= 3.0:
                    reached_time = float(close_since - goal[0])
                    break
            else:
                close_since = None
        # Exclude the 8s map-observation hover after successful arrival.
        route_end = goal[0] + reached_time + 3.0 if reached_time is not None \
            else sampled_times[-1]
        after = (sampled_times >= goal[0]) & (sampled_times <= route_end)
        segment = sampled_truth[after]
        if len(segment) > 1:
            route_length = float(np.linalg.norm(np.diff(segment[:, :2], axis=0),
                                                axis=1).sum())
        end_goal_error = float(np.linalg.norm(odom[-1, :2] - np.array(goal[1:3])))

    active_risk = None
    if len(risk_times) >= 2:
        stamps = np.asarray(risk_times)
        values = np.asarray(risk_values)
        if goal is not None:
            risk_end = goal[0] + reached_time + 3.0 if reached_time is not None \
                else stamps[-1]
            mask = (stamps >= goal[0]) & (stamps <= risk_end)
            stamps, values = stamps[mask], values[mask]
        if len(stamps) >= 2:
            dt = np.diff(stamps)
            active_risk = float(np.sum(dt[values[:-1] > 0]))

    metrics = {
        "mode": mode,
        "bag": os.path.abspath(path),
        "model": model_name,
        "duration_s": round(float(truth_times[-1] - truth_times[0]), 3),
        "goal_published": goal is not None,
        "goal_reached_odom": reached_time is not None,
        "time_to_goal_s": reached_time,
        "final_goal_error_odom_xy_m": end_goal_error,
        "gazebo_route_length_after_goal_xy_m": route_length,
        "min_uav_pedestrian_center_xy_m": minimum_center_xy,
        "risk_active_after_goal_s": active_risk,
        "fastlio_aligned_ate_3d_rmse_m": ate,
        "pedestrian_displacement_xy_m": pedestrian_displacements,
        "ghost_roi_map_from_world_rotation": map_from_world.tolist(),
        "ghost_roi_map_from_world_translation": translation.tolist(),
        "pedestrian_clearance_scope": "all present pedestrians, including stationary",
        "vacated_start_map_points": count_start_ghosts(map_cloud, initial_map, moved),
        "vacated_start_stable_map_points": count_start_ghosts(
            stable_cloud, initial_map, moved),
        "navigation_map_total_points": int(map_cloud.width * map_cloud.height)
        if map_cloud is not None else None,
        "stable_map_total_points": int(stable_cloud.width * stable_cloud.height)
        if stable_cloud is not None else None,
        "last_local_path_poses": len(nav_path.poses) if nav_path else None,
    }
    if not moved:
        metrics["warning"] = "行人没有离开初始位置超过1.5m；残影指标不可用"
    if goal is None:
        metrics["warning"] = "未录到目标，无法判定到达与航线指标"
    return metrics, (sampled_truth, aligned, paired_truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--mode", choices=("badcase", "ours"), required=True)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    prefix = args.output or os.path.splitext(args.bag)[0]
    metrics, traces = analyze(args.bag, args.mode)
    with open(prefix + "_metrics.json", "w", encoding="utf-8") as stream:
        json.dump(metrics, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    lines = ["Dynamic mapping and avoidance comparison: " + args.mode,
             "bag: " + metrics["bag"]]
    for key, value in metrics.items():
        if key not in ("mode", "bag"):
            lines.append("{}: {}".format(key, value))
    with open(prefix + "_report.txt", "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        truth, aligned, paired_truth = traces
        plt.figure(figsize=(8, 6))
        plt.plot(truth[:, 0], truth[:, 1], "k--", label="Gazebo UAV")
        if aligned is not None:
            plt.plot(aligned[:, 0], aligned[:, 1], color="#1686c7",
                     label="FAST-LIO aligned")
        plt.axis("equal")
        plt.grid(alpha=0.25)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(args.mode + " trajectory")
        plt.legend()
        plt.tight_layout()
        plt.savefig(prefix + "_trajectory.png", dpi=160)
    except ImportError:
        print("matplotlib 不可用，跳过轨迹图")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rosbag.ROSBagException) as exc:
        print("分析失败：{}".format(exc), file=sys.stderr)
        sys.exit(2)
