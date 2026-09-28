#!/usr/bin/env python3
"""Offline MID360 BEV and LMNet-style range-residual visualization.

This is NOT the LMNet/SalsaNext network and produces no motion labels. It only
reprojects older raw scans with FAST-LIO odometry and displays range differences.
"""

import argparse
import json
from copy import copy
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from sensor_msgs import point_cloud2


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXTRINSIC = ROOT / "AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml"
LIDAR_TOPIC = "/livox/lidar"
ODOM_TOPIC = "/uav1/fastlio/odom"
GT_TOPIC = "/gazebo/model_states"


def quaternion_matrix(q):
    """ROS xyzw quaternion -> active 3x3 rotation matrix."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def slerp(q0, q1, fraction):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + fraction * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1 - fraction) * theta) * q0 +
            np.sin(fraction * theta) * q1) / np.sin(theta)


def pose_at(odometry, stamp):
    stamps = odometry["time"]
    right = int(np.searchsorted(stamps, stamp))
    if right == 0 or right == len(stamps):
        raise ValueError("scan %.3f outside odometry interval" % stamp)
    left = right - 1
    gap = stamps[right] - stamps[left]
    if gap <= 0 or gap > 0.35:
        raise ValueError("odometry gap %.3fs near scan %.3f" % (gap, stamp))
    alpha = (stamp - stamps[left]) / gap
    position = ((1 - alpha) * odometry["position"][left] +
                alpha * odometry["position"][right])
    rotation = quaternion_matrix(slerp(odometry["quaternion"][left],
                                       odometry["quaternion"][right], alpha))
    return position, rotation


def xyz_from_cloud(message):
    points = np.asarray(list(point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
    if points.size == 0:
        raise ValueError("selected raw scan has no finite XYZ points")
    return points.reshape(-1, 3)


def world_points(sensor_points, pose, rotation_bl, translation_bl):
    position, rotation_wb = pose
    return ((sensor_points @ rotation_bl.T + translation_bl) @
            rotation_wb.T + position)


def world_to_sensor(world, pose, rotation_bl, translation_bl):
    position, rotation_wb = pose
    return ((world - position) @ rotation_wb - translation_bl) @ rotation_bl


def range_image(points, height, width, elev_min, elev_max, max_range):
    """Spherical projection with the nearest return per angular pixel."""
    radius = np.linalg.norm(points, axis=1)
    azimuth = np.arctan2(points[:, 1], points[:, 0])
    elevation = np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1]))
    valid = (np.isfinite(radius) & (radius >= 0.5) & (radius <= max_range) &
             (elevation >= elev_min) & (elevation < elev_max))
    u = np.clip(np.floor((azimuth + np.pi) / (2 * np.pi) * width).astype(int),
                0, width - 1)
    v = np.clip(np.floor((elev_max - elevation) /
                         (elev_max - elev_min) * height).astype(int),
                0, height - 1)
    image = np.full(height * width, np.inf, dtype=np.float32)
    np.minimum.at(image, v[valid] * width + u[valid], radius[valid])
    image[~np.isfinite(image)] = np.nan
    return image.reshape(height, width), u, v, valid


def detect_event(bag, first_time):
    """Pick the closest moving pedestrian/UAV XY encounter after takeoff."""
    initial = {}
    best = (np.inf, None)
    last_sample = -np.inf
    for _, message, bag_stamp in bag.read_messages(topics=[GT_TOPIC]):
        stamp = bag_stamp.to_sec()
        if stamp - last_sample < 0.10:
            continue
        last_sample = stamp
        names = {name: i for i, name in enumerate(message.name)}
        if "iris_mid360" not in names:
            continue
        uav = message.pose[names["iris_mid360"]].position
        for pedestrian in ("0", "1"):
            if pedestrian not in names:
                continue
            person = message.pose[names[pedestrian]].position
            initial.setdefault(pedestrian, (person.x, person.y))
            moved = np.hypot(person.x - initial[pedestrian][0],
                             person.y - initial[pedestrian][1])
            if uav.z < 0.8 or moved < 0.3 or stamp - first_time < 10:
                continue
            distance = float(np.hypot(uav.x - person.x, uav.y - person.y))
            if distance < best[0]:
                best = (distance, stamp)
    if best[1] is None:
        raise ValueError("cannot find moving pedestrian encounter; use --event-seconds")
    return best


def load_samples(bag, targets):
    selected = [None] * len(targets)
    best_delta = [np.inf] * len(targets)
    times, positions, quaternions = [], [], []
    for topic, message, bag_stamp in bag.read_messages(topics=[LIDAR_TOPIC, ODOM_TOPIC]):
        stamp = message.header.stamp.to_sec() or bag_stamp.to_sec()
        if topic == LIDAR_TOPIC:
            for index, target in enumerate(targets):
                delta = abs(stamp - target)
                if delta < best_delta[index]:
                    selected[index], best_delta[index] = (stamp, message), delta
        else:
            pose = message.pose.pose
            times.append(stamp)
            positions.append((pose.position.x, pose.position.y, pose.position.z))
            quaternions.append((pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w))
    if any(item is None for item in selected):
        raise ValueError("no raw LiDAR scans found at the selected times")
    if any(delta > 0.20 for delta in best_delta):
        raise ValueError("a requested frame is more than 0.2s from a LiDAR scan")
    if len(set(stamp for stamp, _ in selected)) != len(selected):
        raise ValueError("selected timestamps collapsed onto the same scan")
    order = np.argsort(times)
    odometry = {
        "time": np.asarray(times)[order],
        "position": np.asarray(positions)[order],
        "quaternion": np.asarray(quaternions)[order],
    }
    return selected, odometry


def save_figures(output, frames, poses, world_clouds, ranges, residuals,
                 point_residuals, first_time, max_range, bev_half_width):
    center = poses[1][0][:2]
    bounds = [center[0] - bev_half_width, center[0] + bev_half_width,
              center[1] - bev_half_width, center[1] + bev_half_width]
    n = len(frames)
    fig, axes = plt.subplots(1, n, figsize=(4.7 * n, 4.5), sharex=True, sharey=True)
    for i, axis in enumerate(axes):
        cloud = world_clouds[i]
        mask = (cloud[:, 2] >= 0.3) & (cloud[:, 2] <= 3.0)
        axis.scatter(cloud[mask, 0], cloud[mask, 1], s=0.22, c=cloud[mask, 2],
                     cmap="viridis", vmin=0.3, vmax=3.0, rasterized=True)
        axis.scatter(*poses[i][0][:2], s=80, marker="x", color="red")
        axis.set(xlim=bounds[:2], ylim=bounds[2:], aspect="equal",
                 title="t = %.2f s" % (frames[i][0] - first_time),
                 xlabel="map x [m]")
        if i == 0:
            axis.set_ylabel("map y [m]")
    fig.suptitle("BEV of four raw MID360 scans (height color 0.3-3.0 m; red x = UAV)", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.89))
    fig.savefig(output / "bev_frames.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, n, figsize=(4.8 * n, 7.2), sharex=True, sharey=True)
    range_cmap = copy(plt.get_cmap("viridis"))
    residual_cmap = copy(plt.get_cmap("magma"))
    range_cmap.set_bad("#282830")
    residual_cmap.set_bad("#282830")
    for i in range(n):
        axes[0, i].imshow(ranges[i], cmap=range_cmap, vmin=0, vmax=max_range,
                          aspect="auto", origin="upper")
        axes[0, i].set_title("range at %.2f s" % (frames[i][0] - first_time))
        axes[1, i].imshow(residuals[i], cmap=residual_cmap, vmin=0, vmax=0.8,
                          aspect="auto", origin="upper")
        axes[1, i].set_title("residual to previous frame" if i else "no previous frame")
        axes[1, i].set_xlabel("azimuth bin (0-360 deg)")
    axes[0, 0].set_ylabel("elevation bin\nnearer = yellow")
    axes[1, 0].set_ylabel("elevation bin\n|r-now - r-warped| / r-now")
    fig.suptitle("LMNet-style spherical range and ego-motion-compensated residual\n"
                 "Range colors: 0-%.0f m; residual colors: 0-0.8 (clipped); "
                 "black = no valid overlap" % max_range, y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(output / "range_residuals.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, n - 1, figsize=(5.2 * (n - 1), 5.0),
                             sharex=True, sharey=True)
    for i, axis in enumerate(axes, start=1):
        cloud = world_clouds[i]
        band = (cloud[:, 2] >= 0.3) & (cloud[:, 2] <= 3.0)
        axis.scatter(cloud[band, 0], cloud[band, 1], s=0.2, color="#777777",
                     alpha=0.20, rasterized=True)
        values = point_residuals[i]
        visible = band & np.isfinite(values)
        axis.scatter(cloud[visible, 0], cloud[visible, 1], s=1.0,
                     c=np.clip(values[visible], 0, 0.8), cmap="magma",
                     vmin=0, vmax=0.8, rasterized=True)
        axis.scatter(*poses[i][0][:2], s=80, marker="x", color="cyan")
        axis.set(xlim=bounds[:2], ylim=bounds[2:], aspect="equal",
                 title="%.2f - %.2f s" % (frames[i][0] - first_time,
                                            frames[i - 1][0] - first_time),
                 xlabel="map x [m]")
        if i == 1:
            axis.set_ylabel("map y [m]")
    fig.suptitle("BEV of range residuals (0-0.8 clipped); bright = change cue, not MOS label",
                 y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(output / "bev_residuals.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="bag containing raw /livox/lidar and odometry")
    parser.add_argument("--output-dir", type=Path,
                        help="new directory; default analysis_outputs/<bag>_<timestamp>")
    parser.add_argument("--event-seconds", type=float,
                        help="event time relative to bag start; default closest moving-person encounter")
    parser.add_argument("--extrinsic-yaml", type=Path, default=DEFAULT_EXTRINSIC)
    parser.add_argument("--height", type=int, default=64)
    parser.add_argument("--width", type=int, default=360)
    parser.add_argument("--elev-min", type=float, default=-45.0, help="degrees")
    parser.add_argument("--elev-max", type=float, default=45.0, help="degrees")
    parser.add_argument("--max-range", type=float, default=30.0)
    parser.add_argument("--bev-half-width", type=float, default=4.0)
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error("bag does not exist: %s" % args.bag)
    if args.height < 4 or args.width < 16 or args.elev_min >= args.elev_max:
        parser.error("invalid projection dimensions or elevation bounds")
    if args.max_range <= 0.5 or args.bev_half_width <= 0:
        parser.error("--max-range must exceed 0.5m and --bev-half-width must be positive")
    config = yaml.safe_load(args.extrinsic_yaml.read_text(encoding="utf-8"))
    translation_bl = np.asarray(config["mapping"]["extrinsic_T"], dtype=float)
    rotation_bl = np.asarray(config["mapping"]["extrinsic_R"], dtype=float).reshape(3, 3)

    with rosbag.Bag(str(args.bag), "r") as bag:
        topics = bag.get_type_and_topic_info().topics
        missing = [topic for topic in (LIDAR_TOPIC, ODOM_TOPIC) if topic not in topics]
        if missing:
            parser.error("bag lacks %s; record the demo with --full-bag next time" % missing)
        first_time = bag.get_start_time()
        if args.event_seconds is None:
            if GT_TOPIC not in topics:
                parser.error("no Gazebo truth; specify --event-seconds")
            nearest_distance, event = detect_event(bag, first_time)
        else:
            event = first_time + args.event_seconds
            nearest_distance = None
        targets = event + np.array([-0.75, -0.25, 0.25, 0.75])
        if targets[0] < first_time or targets[-1] > bag.get_end_time():
            parser.error("event too close to a bag boundary for four frames")
        frames, odometry = load_samples(bag, targets)

    poses = [pose_at(odometry, stamp) for stamp, _ in frames]
    sensor_clouds = [xyz_from_cloud(message) for _, message in frames]
    world_clouds = [world_points(cloud, pose, rotation_bl, translation_bl)
                    for cloud, pose in zip(sensor_clouds, poses)]
    low, high = np.radians([args.elev_min, args.elev_max])
    ranges, residuals, point_residuals = [], [], []
    statistics = []
    for i, (stamp, _) in enumerate(frames):
        current, u, v, valid = range_image(sensor_clouds[i], args.height,
                                            args.width, low, high, args.max_range)
        ranges.append(current)
        residual = np.full_like(current, np.nan)
        point_values = np.full(len(sensor_clouds[i]), np.nan)
        common_count = 0
        if i:
            warped = world_to_sensor(world_clouds[i - 1], poses[i],
                                     rotation_bl, translation_bl)
            previous, _, _, _ = range_image(warped, args.height, args.width,
                                              low, high, args.max_range)
            common = np.isfinite(current) & np.isfinite(previous)
            residual[common] = (
                np.abs(current[common] - previous[common]) /
                np.maximum(current[common], 1e-3))
            common_count = int(np.count_nonzero(common))
            # Only the nearest return actually represented by the range pixel
            # receives its value when residuals are displayed back in BEV.
            point_radius = np.linalg.norm(sensor_clouds[i], axis=1)
            point_valid = np.zeros(len(point_radius), dtype=bool)
            candidates = np.flatnonzero(valid & common[v, u])
            point_valid[candidates] = (
                np.abs(point_radius[candidates] - current[v[candidates], u[candidates]])
                < 0.01)
            point_values[point_valid] = residual[v[point_valid], u[point_valid]]
        residuals.append(residual)
        point_residuals.append(point_values)
        statistics.append({
            "bag_time_s": stamp, "relative_time_s": stamp - first_time,
            "point_count": int(len(sensor_clouds[i])),
            "valid_range_pixels": int(np.count_nonzero(np.isfinite(current))),
            "common_pixels_with_previous": common_count,
            "common_fraction_of_current": (common_count /
                max(1, np.count_nonzero(np.isfinite(current)))) if i else None,
            "residual_median": float(np.nanmedian(residual)) if common_count else None,
            "residual_p95": float(np.nanpercentile(residual, 95)) if common_count else None,
        })

    output = args.output_dir or (ROOT / "analysis_outputs" /
        ("lmnet_%s_%s" % (args.bag.stem, datetime.now().strftime("%Y%m%d_%H%M%S"))))
    if output.exists():
        parser.error("output directory already exists; choose a new --output-dir: %s" % output)
    output.mkdir(parents=True)

    save_figures(output, frames, poses, world_clouds, ranges, residuals,
                 point_residuals, first_time, args.max_range, args.bev_half_width)
    archive = {}
    for i in range(len(frames)):
        archive["xyz_lidar_%d" % i] = sensor_clouds[i].astype(np.float32)
        archive["xyz_map_%d" % i] = world_clouds[i].astype(np.float32)
        archive["range_%d" % i] = ranges[i]
        archive["residual_%d" % i] = residuals[i]
    np.savez_compressed(output / "sampled_frames.npz", **archive)
    metadata = {
        "source_bag": str(args.bag.resolve()),
        "source_topics": [topic for topic in (LIDAR_TOPIC, ODOM_TOPIC, GT_TOPIC)
                          if topic in topics],
        "event_relative_time_s": event - first_time,
        "nearest_gt_center_xy_distance_m": nearest_distance,
        "projection": {"height": args.height, "width": args.width,
                       "elevation_degrees": [args.elev_min, args.elev_max],
                       "max_range_m": args.max_range},
        "extrinsic_yaml": str(args.extrinsic_yaml.resolve()),
        "residual_definition": "abs(current_range - warped_previous_range) / current_range",
        "warning": "Not LMNet inference or dynamic-point ground truth; MID360 sampling and occlusion cause residuals.",
        "frames": statistics,
    }
    (output / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print("Images and sampled scans: %s" % output)


if __name__ == "__main__":
    main()
