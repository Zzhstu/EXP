#!/usr/bin/env python3
"""Print a concise, same-definition badcase/ours table from two JSON reports."""

import argparse
import json
from pathlib import Path


def gazebo_route(bag_path):
    import rosbag
    points = []
    goal_time = None
    last_sample = -float("inf")
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(
                topics=["/gazebo/model_states", "/move_base_simple/goal"]):
            if topic == "/move_base_simple/goal":
                goal_time = stamp.to_sec()
            elif goal_time is not None and stamp.to_sec() - last_sample >= 0.15:
                models = [name for name in msg.name if "iris_mid360" in name]
                if not models:
                    continue
                pose = msg.pose[msg.name.index(models[0])].position
                points.append((pose.x, pose.y))
                last_sample = stamp.to_sec()
    return points


def save_comparison_plot(bad, ours, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    bad_path = gazebo_route(bad["bag"])
    ours_path = gazebo_route(ours["bag"])
    if not bad_path or not ours_path:
        return False
    plt.figure(figsize=(7, 7))
    for points, color, label in ((bad_path, "#d97706", "badcase"),
                                 (ours_path, "#16a34a", "ours")):
        x, y = zip(*points)
        plt.plot(x, y, color=color, linewidth=2, label=label)
        plt.scatter((x[0],), (y[0],), color=color, marker="o", s=24)
        plt.scatter((x[-1],), (y[-1],), color=color, marker="x", s=48)
    plt.axis("equal")
    plt.grid(alpha=0.25)
    plt.xlabel("Gazebo x [m]")
    plt.ylabel("Gazebo y [m]")
    plt.title("Actual UAV route from goal to finish")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=170)
    return True


FIELDS = [
    ("goal_reached_odom", "到达并稳定悬停", "true"),
    ("time_to_goal_s", "到达用时 [s]", "lower"),
    ("final_goal_error_odom_xy_m", "终点平面误差 [m]", "lower"),
    ("gazebo_route_length_after_goal_xy_m", "实际飞行平面路程 [m]", "lower"),
    ("min_uav_pedestrian_center_xy_m", "最小人机中心平面距离 [m]", "higher"),
    ("risk_active_after_goal_s", "风险触发累计时间 [s]", "context"),
    ("vacated_start_stable_map_points", "人离开后旧位置地图点数", "lower"),
    ("stable_map_total_points", "最终稳定地图总点数", "context"),
    ("fastlio_aligned_ate_3d_rmse_m", "FAST-LIO 3D ATE RMSE [m]", "lower"),
]


def printable(value):
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, float):
        return "{:.3f}".format(value)
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("badcase_json")
    parser.add_argument("ours_json")
    parser.add_argument("--plot", default="",
                        help="combined Gazebo route PNG; defaults beside metrics")
    args = parser.parse_args()
    with open(args.badcase_json, encoding="utf-8") as stream:
        bad = json.load(stream)
    with open(args.ours_json, encoding="utf-8") as stream:
        ours = json.load(stream)
    if bad.get("mode") != "badcase" or ours.get("mode") != "ours":
        parser.error("参数顺序必须是 badcase 指标 JSON 然后 ours 指标 JSON")
    print("| 指标 | badcase | ours | 判读 |")
    print("|---|---:|---:|---|")
    for key, label, direction in FIELDS:
        a, b = bad.get(key), ours.get(key)
        if direction == "context":
            interpretation = "结合安全距离与目标进度判断"
        elif a is None or b is None:
            interpretation = "数据不足"
        elif direction == "true":
            interpretation = "两者均到达" if a and b else (
                "仅 ours 到达" if b else "ours 未到达")
        elif key == "fastlio_aligned_ate_3d_rmse_m":
            interpretation = "不同路径，仅监控定位质量"
        elif key == "min_uav_pedestrian_center_xy_m":
            interpretation = "单次差异需重复验证"
        else:
            improved = b < a if direction == "lower" else b > a
            interpretation = "ours 更优" if improved else "未体现优势"
        print("| {} | {} | {} | {} |".format(
            label, printable(a), printable(b), interpretation))
    print("\n注意：中心距离不是模型表面净空，也不能据此宣称零碰撞。")
    print("不同飞行路径的ATE仅用于监控定位质量，不能归因于地图清理。")
    output = args.plot or str(Path(args.ours_json).parent / "badcase_vs_ours.png")
    try:
        if save_comparison_plot(bad, ours, output):
            print("同坐标航线图：{}".format(output))
        else:
            print("bag缺少目标或Gazebo真值，未生成合并轨迹图")
    except ImportError:
        print("缺少rosbag或matplotlib，未生成合并轨迹图")


if __name__ == "__main__":
    main()
