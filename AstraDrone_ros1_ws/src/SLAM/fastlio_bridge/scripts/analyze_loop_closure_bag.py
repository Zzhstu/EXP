#!/usr/bin/env python3
"""Evaluate raw/optimized loop trajectories against Gazebo ground truth.

The report uses timestamp interpolation and a rigid SE(3) alignment for ATE,
so a harmless difference between the SLAM and Gazebo frame origins is not
counted as drift. It also reports the unaligned value and end-to-start closure
drift to expose initialization or alignment mistakes.
"""

import argparse
import math
import os
import sys

import numpy as np

try:
    import rosbag
except ImportError as exc:
    print("ERROR: source /opt/ros/noetic/setup.bash before running this script: %s" % exc,
          file=sys.stderr)
    sys.exit(2)


GT_TOPIC = "/gazebo/model_states"
RAW_TOPIC = "/uav1/fastlio/odom"
OPT_TOPIC = "/uav1/loop_closure/odom"
DIAG_TOPIC = "/uav1/loop_closure/diagnostics"


def message_time(msg, bag_time):
    if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0:
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def pose_xyz(msg):
    p = msg.pose.pose.position
    return [p.x, p.y, p.z]


def rigid_align(estimate, truth):
    """Return estimate rotated/translated onto truth (no scale)."""
    est_mean = estimate.mean(axis=0)
    truth_mean = truth.mean(axis=0)
    covariance = (estimate - est_mean).T.dot(truth - truth_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T.dot(u.T)
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T.dot(u.T)
    translation = truth_mean - rotation.dot(est_mean)
    return estimate.dot(rotation.T) + translation, rotation, translation


def interpolate_truth(gt_t, gt_p, sample_t):
    valid = (sample_t >= gt_t[0]) & (sample_t <= gt_t[-1])
    times = sample_t[valid]
    values = np.column_stack([np.interp(times, gt_t, gt_p[:, axis]) for axis in range(3)])
    return valid, values


def trajectory_metrics(times, positions, gt_t, gt_p):
    valid, truth = interpolate_truth(gt_t, gt_p, times)
    estimate = positions[valid]
    sample_t = times[valid]
    if len(estimate) < 3:
        return None
    aligned, rotation, _ = rigid_align(estimate, truth)
    aligned_error = np.linalg.norm(aligned - truth, axis=1)
    raw_error = np.linalg.norm(estimate - truth, axis=1)

    # Translational RPE over approximately five seconds.
    rpe = []
    for i, stamp in enumerate(sample_t):
        j = int(np.searchsorted(sample_t, stamp + 5.0))
        if j >= len(sample_t):
            break
        est_delta = aligned[j] - aligned[i]
        truth_delta = truth[j] - truth[i]
        rpe.append(np.linalg.norm(est_delta - truth_delta))

    est_displacement = rotation.dot(estimate[-1] - estimate[0])
    truth_displacement = truth[-1] - truth[0]
    closure_drift = np.linalg.norm(est_displacement - truth_displacement)
    travelled = float(np.linalg.norm(np.diff(truth, axis=0), axis=1).sum())
    return {
        "samples": len(estimate),
        "duration": float(sample_t[-1] - sample_t[0]),
        "truth_distance": travelled,
        "ate_rmse": float(np.sqrt(np.mean(aligned_error ** 2))),
        "ate_mean": float(np.mean(aligned_error)),
        "ate_max": float(np.max(aligned_error)),
        "unaligned_rmse": float(np.sqrt(np.mean(raw_error ** 2))),
        "rpe_5s_rmse": float(np.sqrt(np.mean(np.square(rpe)))) if rpe else math.nan,
        "closure_drift": float(closure_drift),
        "times": sample_t,
        "aligned": aligned,
        "truth": truth,
    }


def format_metrics(name, metrics):
    if metrics is None:
        return ["%s: insufficient samples" % name]
    return [
        "%s samples: %d (%.1f s, truth path %.2f m)" %
        (name, metrics["samples"], metrics["duration"], metrics["truth_distance"]),
        "%s ATE aligned RMSE/mean/max: %.4f / %.4f / %.4f m" %
        (name, metrics["ate_rmse"], metrics["ate_mean"], metrics["ate_max"]),
        "%s ATE without frame alignment RMSE: %.4f m" % (name, metrics["unaligned_rmse"]),
        "%s translational RPE (5 s) RMSE: %.4f m" % (name, metrics["rpe_5s_rmse"]),
        "%s end-to-start displacement error: %.4f m" % (name, metrics["closure_drift"]),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="benchmark rosbag")
    parser.add_argument("--model", default="iris_mid360",
                        help="Gazebo model name (default: iris_mid360; substring fallback is allowed)")
    parser.add_argument("--output", default="", help="output prefix; defaults to bag path without .bag")
    args = parser.parse_args()
    output = args.output or os.path.splitext(args.bag)[0]

    gt_t, gt_p, raw_t, raw_p, opt_t, opt_p = [], [], [], [], [], []
    diagnostics = {}
    selected_model = None
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, bag_time in bag.read_messages(
                topics=[GT_TOPIC, RAW_TOPIC, OPT_TOPIC, DIAG_TOPIC]):
            if topic == GT_TOPIC:
                if selected_model not in msg.name:
                    selected_model = args.model if args.model in msg.name else None
                    if selected_model is None:
                        matches = [name for name in msg.name if args.model in name or "iris" in name]
                        selected_model = matches[0] if matches else None
                if selected_model is not None:
                    index = msg.name.index(selected_model)
                    p = msg.pose[index].position
                    gt_t.append(bag_time.to_sec())
                    gt_p.append([p.x, p.y, p.z])
            elif topic == RAW_TOPIC:
                raw_t.append(message_time(msg, bag_time))
                raw_p.append(pose_xyz(msg))
            elif topic == OPT_TOPIC:
                opt_t.append(message_time(msg, bag_time))
                opt_p.append(pose_xyz(msg))
            elif topic == DIAG_TOPIC:
                for status in msg.status:
                    for item in status.values:
                        diagnostics[item.key] = item.value

    if len(gt_t) < 3:
        raise RuntimeError("No usable Gazebo ground truth for model '%s'" % args.model)
    gt_t, gt_p = np.asarray(gt_t), np.asarray(gt_p)
    raw = trajectory_metrics(np.asarray(raw_t), np.asarray(raw_p), gt_t, gt_p) if raw_t else None
    optimized = trajectory_metrics(np.asarray(opt_t), np.asarray(opt_p), gt_t, gt_p) if opt_t else None

    lines = [
        "FAST-LIO2 loop-closure evaluation",
        "bag: %s" % os.path.abspath(args.bag),
        "ground-truth model: %s" % selected_model,
        "",
    ]
    lines.extend(format_metrics("RAW", raw))
    lines.append("")
    lines.extend(format_metrics("OPTIMIZED", optimized))
    lines.append("")
    if raw and optimized:
        lines.append("ATE RMSE change: %.4f m (%+.1f%%; negative is improvement)" % (
            optimized["ate_rmse"] - raw["ate_rmse"],
            100.0 * (optimized["ate_rmse"] / max(raw["ate_rmse"], 1e-9) - 1.0)))
        lines.append("Closure displacement change: %.4f m" %
                     (optimized["closure_drift"] - raw["closure_drift"]))
    lines.append("accepted loop constraints: %s" % diagnostics.get("accepted_loops", "not recorded"))
    for key in ("keyframes", "graph_edges", "no_spatial_candidates", "rejected_scan_context",
                "icp_attempts", "rejected_icp", "optimizer_failures", "last_stage",
                "last_spatial_distance_m", "last_scan_context_score",
                "last_sector_shift_deg", "last_icp_fitness", "last_icp_overlap",
                "last_processing_ms"):
        if key in diagnostics:
            lines.append("diagnostic %s: %s" % (key, diagnostics[key]))

    report_path = output + "_report.txt"
    with open(report_path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("report: %s" % report_path)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 7))
        if raw:
            plt.plot(raw["truth"][:, 0], raw["truth"][:, 1], "k--", label="Gazebo truth")
            plt.plot(raw["aligned"][:, 0], raw["aligned"][:, 1], color="#f5a623", label="Raw FAST-LIO")
        if optimized:
            plt.plot(optimized["aligned"][:, 0], optimized["aligned"][:, 1], color="#20c85a", label="Optimized")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Loop-closure trajectory comparison")
        plt.legend()
        plt.tight_layout()
        plot_path = output + "_trajectory.png"
        plt.savefig(plot_path, dpi=160)
        print("plot: %s" % plot_path)
    except ImportError:
        print("matplotlib unavailable: skipped trajectory PNG")


if __name__ == "__main__":
    main()
