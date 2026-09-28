#!/usr/bin/env python3
"""Run one isolated PX4/Gazebo comparison case, record a bag, and report it."""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rosgraph
import roslib.packages
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


TOPICS = [
    "/gazebo/model_states", "/mavros/state", "/mavros/local_position/pose",
    "/uav1/fastlio/odom", "/uav1/fastlio/path",
    "/uav1/fastlio/cloud_map", "/uav1/stable_static_map",
    "/uav1/local_static_map", "/uav1/local_free_space",
    "/uav1/dynamic_points", "/uav1/semantic_dynamic_objects",
    "/uav1/fused_collision_risk_level", "/uav1/planner/path",
    "/uav1/planner/local_path", "/uav1/planner/waypoint",
    "/move_base_simple/goal", "/mavros/setpoint_velocity/cmd_vel",
    "/uav1/dynamic_objects", "/uav1/dynamic_markers", "/demo/pedestrian/ready",
]


def process_running(name):
    result = subprocess.run(["pgrep", "-x", name], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, check=False)
    return result.returncode == 0


def cleanup_interrupt(signum, frame):
    # Repeated Ctrl-C must not unwind finally and leave the ROS master alive.
    print('正在清理本次演示，请等待；重复 Ctrl-C 不会中断清理。', flush=True)


def finish_recording(process, bag, timeout=25):
    """Wait for the native writer, then validate its completed index."""
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=timeout)
    if process.returncode != 0:
        raise RuntimeError('录包进程退出码 {}'.format(process.returncode))
    if not bag.exists() or Path(str(bag)+'.active').exists():
        raise RuntimeError('录包未完成写盘，保留文件供检查')
    with rosbag.Bag(str(bag), 'r') as recorded:
        if recorded.get_message_count() == 0:
            raise RuntimeError('bag为空')


def wait_topic(topic, message_type, timeout, process=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError("关键进程在等待 {} 时退出，检查日志".format(topic))
        try:
            return rospy.wait_for_message(topic, message_type, timeout=2.0)
        except rospy.ROSException:
            pass
    raise RuntimeError("等待话题 {} 超时 ({:.0f}s)".format(topic, timeout))


class CaseRunner:
    def __init__(self, root, prefix, env):
        self.root = root
        self.prefix = prefix
        self.env = env
        self.processes = []
        self.logs = []

    def start(self, name, command):
        path = str(self.prefix) + "_" + name + ".log"
        stream = open(path, "w", encoding="utf-8")
        self.logs.append(stream)
        process = subprocess.Popen(command, cwd=str(self.root), env=self.env,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes.append((name, process))
        print("启动 {} (PID {}), 日志 {}".format(name, process.pid, path), flush=True)
        return process

    def stop(self):
        # Keep ROS master available while native nodes unregister and Gazebo
        # unloads plugins. Killing it with all children produced XMLRPC errors
        # and a Gazebo shutdown crash in the warehouse GUI test.
        # Only process groups created by this runner are touched.
        for group in ('children', 'core'):
            selected = [(name, process) for name, process in reversed(self.processes)
                        if (name == 'core') == (group == 'core')]
            for _, process in selected:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for name, process in selected:
                if process.poll() is None:
                    try:
                        # roslaunch has its own ~15 s native-node SIGINT
                        # grace period; do not terminate its monitor at 8 s.
                        process.wait(timeout=30 if name == 'gazebo' else 20)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGTERM)
                            process.wait(timeout=4)
                        except ProcessLookupError:
                            pass
                        except subprocess.TimeoutExpired:
                            try:
                                os.killpg(process.pid, signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                            process.wait(timeout=3)
        for stream in self.logs:
            stream.close()

        # roslaunch gives some native nodes their own process session. An
        # orphan FAST-LIO can attach to the next ROS master and corrupt the
        # comparison. Preflight established that none existed before this run.
        result = subprocess.run(["pgrep", "-x", "fastlio_mapping"],
                                capture_output=True, text=True, check=False)
        for token in result.stdout.split():
            pid = int(token)
            command = Path("/proc/{}/cmdline".format(pid))
            try:
                command_line = command.read_bytes().replace(b"\0", b" ")
            except (FileNotFoundError, PermissionError):
                continue
            expected = str(self.root / "AstraDrone_ros1_ws/devel/lib/fast_lio/fastlio_mapping").encode()
            if expected not in command_line:
                continue
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                continue
            for _ in range(20):
                if not command.exists():
                    break
                time.sleep(0.1)
            if command.exists():
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("badcase", "ours"))
    parser.add_argument('--scene', choices=('example', 'cangku'), default='example')
    parser.add_argument('--navigation-profile', choices=('auto', 'standard', 'warehouse'),
                        default='auto', help='auto: cangku uses the low-speed warehouse clearance profile')
    parser.add_argument("--headless", action="store_true",
                        help="Gazebo server only and no RViz; useful for CI")
    parser.add_argument("--full-bag", action="store_true",
                        help="also record raw Livox packets; much larger bag")
    parser.add_argument("--max-flight-seconds", type=float, default=160.0,
                        help="maximum recording/navigation observation window; does not stop the live demo")
    parser.add_argument("--auto-stop", action="store_true",
                        help="exit and close the simulator after the report (for unattended tests)")
    args = parser.parse_args()

    root = Path(os.environ["ASTRA_ROOT"])
    px4 = Path(os.environ["PX4_DIR"])
    ped = Path(os.environ["PEDSIM_WS"])
    output = root / "dynamic_demo_runs"
    output.mkdir(exist_ok=True)
    label = args.mode if args.scene == 'example' else args.mode + '_cangku'
    prefix = output / "{}_{}".format(label, datetime.now().strftime("%Y%m%d_%H%M%S"))
    if rosgraph.is_master_online():
        parser.error("已有ROS master；请先结束旧仿真后再运行独立对照")
    leftovers = [name for name in ("gzserver", "px4", "fastlio_mapping")
                 if process_running(name)]
    if leftovers:
        parser.error("已有仿真进程：{}".format(", ".join(leftovers)))
    if args.max_flight_seconds <= 10:
        parser.error("--max-flight-seconds 必须大于10")
    if not args.headless and not os.environ.get("DISPLAY"):
        parser.error("没有图形DISPLAY；请加 --headless")

    env = os.environ.copy()
    simlib = str(root / "simulation/sim_workspace/devel/lib")
    pedlib = str(ped / "devel/lib")
    env["GAZEBO_PLUGIN_PATH"] = ":".join(filter(None, [pedlib, simlib,
                                               env.get("GAZEBO_PLUGIN_PATH", "")]))
    env["LD_LIBRARY_PATH"] = ":".join(filter(None, [pedlib, simlib,
                                            env.get("LD_LIBRARY_PATH", "")]))
    env["GAZEBO_MODEL_PATH"] = ":".join(filter(None, [
        str(root / "simulation/astra_gazebo_models"),
        str(ped / "src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models"),
        env.get("GAZEBO_MODEL_PATH", "")]))

    run = CaseRunner(root, prefix, env)
    bag_process = None
    map_saver_started = False
    run_error = None
    analysis_returncode = None
    bag = Path(str(prefix) + ".bag")
    analyzer = root / "AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/analyze_dynamic_demo_bag.py"
    try:
        core = run.start("core", ["roscore"])
        for _ in range(60):
            if rosgraph.is_master_online():
                break
            if core.poll() is not None:
                raise RuntimeError("roscore 启动失败")
            time.sleep(0.5)
        else:
            raise RuntimeError("roscore 30秒内未就绪")
        rospy.init_node("dynamic_comparison_runner", anonymous=True,
                        disable_signals=True)

        gazebo_launch = str(root / "simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch")
        setup = str(px4 / "Tools/simulation/gazebo-classic/setup_gazebo.bash")
        world = str(root / "simulation/astra_gazebo_worlds/example.world")
        spawn_args = ''
        geometry = None
        if args.scene == 'cangku':
            from warehouse_scene import prepare
            world, geometry = prepare(root, prefix)
            spawn_args = ' x:=-6 y:=-4'
        gazebo_cmd = "source '{}' '{}' '{}'; exec roslaunch '{}' world:='{}' gui:={} interactive:=false".format(
            setup, px4, px4 / "build/px4_sitl_default", gazebo_launch,
            world, "false" if args.headless else "true")
        gazebo_cmd += spawn_args
        gazebo = run.start("gazebo", ["bash", "-lc", gazebo_cmd])
        wait_topic("/livox/lidar", PointCloud2, 120, gazebo)

        lio = run.start("fastlio", ["roslaunch", "fast_lio", "mapping_mid360.launch",
                                     "rviz:=false", "sim_lidar:=true",
                                     "dynamic_filter:=false"])
        wait_topic("/Odometry", Odometry, 90, lio)
        # Recording begins before the controller starts, so initial map and
        # takeoff are not silently omitted from the comparison.
        topics = TOPICS + (["/livox/lidar", "/livox/imu"]
                           if args.full_bag else [])
        # Track the actual C++ writer, not rosbag's Python launcher (which can
        # exit on SIGINT before its child finishes the index/rename).
        record_nodes = roslib.packages.find_node('rosbag', 'record')
        if not record_nodes:
            raise RuntimeError('找不到rosbag原生record可执行文件')
        bag_process = run.start("bag", [record_nodes[0], "--lz4", "-O",
                                        str(prefix)] + topics)
        time.sleep(2.0)
        if bag_process.poll() is not None:
            raise RuntimeError("rosbag record 启动失败")

        enabled = "true" if args.mode == "ours" else "false"
        warehouse_narrow = args.navigation_profile == 'warehouse' or (
            args.navigation_profile == 'auto' and args.scene == 'cangku')
        print('导航参数：' + ('warehouse (低速/适度缩小静态距离)' if warehouse_narrow
                            else 'standard (原距离)'), flush=True)
        bridge = run.start("navigation", [
            "roslaunch", "fastlio_bridge", "astra_dynamic_avoidance.launch",
            "enable_loop_closure:=false", "enable_control:=true",
            "auto_arm:=true", "auto_goal:=true",
            'warehouse_narrow:=' + ('true' if warehouse_narrow else 'false'),
            'relative_goal_y:=' + ('10.0' if args.scene == 'cangku' else '12.0'),
            # cangku's ceiling is only 2.5 m. Use lower takeoff plus the
            # warehouse profile's body-height slice; do not erase roof points.
            'takeoff_height:=' + ('1.2' if args.scene == 'cangku' else '1.5'),
            "enable_dynamic_map_clearing:=" + enabled,
            "enable_free_space_clearing:=" + enabled,
            "enable_local_replanning:=" + enabled,
            "rviz:=" + ("false" if args.headless else "true")])
        wait_topic("/mavros_avoidance_controller/navigation_ready", Bool, 90, bridge)
        if args.mode == 'ours':
            run.start('map_saver', ['rosrun', 'fastlio_bridge', 'save_navigation_map.py',
                                   '_output:=' + str(prefix) + '_final_map.pcd'])
            rospy.wait_for_service('/demo/save_map', timeout=15)
            map_saver_started = True
        goal = wait_topic("/move_base_simple/goal", PoseStamped, 15, bridge)
        print("目标 map=(%.2f, %.2f, %.2f)" % (
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z), flush=True)

        ped_launch = str(root / "AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/astra_pedestrians.launch")
        # Sourcing PedSim replaces CMAKE_PREFIX_PATH/PYTHONPATH. Preserve BOTH
        # overlays so roslaunch can locate our installed Python relay as well
        # as PedSim messages; ROS_PACKAGE_PATH alone is not enough for libexec.
        ped_cmd = ("source '{}'; export CMAKE_PREFIX_PATH='{}':\"$CMAKE_PREFIX_PATH\"; "
                   "export PYTHONPATH='{}':\"$PYTHONPATH\"; "
                   "export ROS_PACKAGE_PATH='{}'; exec roslaunch '{}'").format(
            ped / "devel/setup.bash", root / 'AstraDrone_ros1_ws/devel',
            root / 'AstraDrone_ros1_ws/devel/lib/python3/dist-packages',
            env["ROS_PACKAGE_PATH"], ped_launch)
        if args.scene == 'cangku':
            pedestrians = run.start('pedestrians', [sys.executable,
                str(root/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/warehouse_pedestrian.py'),
                '_scene_file:=' + str(geometry)])
            print('仓库仅0号行人，可立即用键盘控制；无自动横穿。', flush=True)
        else:
            pedestrians = run.start("pedestrians", ["bash", "-lc", ped_cmd])
        # Do not accept a successful flight with an absent pedestrian relay.
        wait_topic('/demo/pedestrian/ready', Bool, 15, pedestrians)

        # Goal arrival is judged using the same map-frame odometry that the
        # controller uses. Offline analysis additionally checks Gazebo truth.
        reached_since = None
        deadline = time.monotonic() + args.max_flight_seconds
        while time.monotonic() < deadline:
            if bridge.poll() is not None or pedestrians.poll() is not None:
                raise RuntimeError("导航或PedSim进程提前退出，检查日志")
            try:
                odom = rospy.wait_for_message("/uav1/fastlio/odom", Odometry,
                                              timeout=2.0)
            except rospy.ROSException:
                continue
            dx = odom.pose.pose.position.x - goal.pose.position.x
            dy = odom.pose.pose.position.y - goal.pose.position.y
            if (dx * dx + dy * dy) ** 0.5 <= 0.30:
                reached_since = reached_since or time.monotonic()
                if time.monotonic() - reached_since >= 3.0:
                    print("目标到达并悬停3秒，继续观察地图8秒。", flush=True)
                    time.sleep(8.0)
                    break
            else:
                reached_since = None
        else:
            print("达到录包观察时限；本段按未到达样本保存，导航仍会继续。", flush=True)

        # Finalize the finite experiment before entering the open-ended
        # presentation phase. Otherwise a hovering drone can grow the bag
        # indefinitely and the report is unavailable until Ctrl-C.
        try:
            finish_recording(bag_process, bag)
            analysis_returncode = subprocess.run(
                [sys.executable, str(analyzer), str(bag),
                 "--mode", args.mode, "--output", str(prefix)],
                env=env, check=False).returncode
        except Exception as exc:
            # Reporting is not flight-critical. Keep navigation alive and
            # report failure honestly instead of shutting down the simulator.
            analysis_returncode = 1
            print('录包/评测失败，仿真继续：{}；检查 *_bag.log'.format(exc), flush=True)
        print("数据目录：{}".format(output), flush=True)
        if map_saver_started:
            try:
                result = rospy.ServiceProxy('/demo/save_map', Trigger)()
                print('地图保存：{} {}'.format(result.success, result.message), flush=True)
                if not result.success:
                    run_error = run_error or result.message
            except Exception as exc:
                run_error = run_error or '阶段地图保存失败: ' + str(exc)
                print(run_error + '；导航继续，退出前会再次尝试保存。', flush=True)
        print(('仓库0号行人可直接控制。' if args.scene == 'cangku' else '行人横穿结束后，') +
              '另开终端运行：rosrun fastlio_bridge pedestrian_control.py --keyboard', flush=True)

        if not args.auto_stop:
            print("录包评测阶段结束；Gazebo/RViz 和无人机继续运行（到达目标后悬停）。按 Ctrl-C 结束本次演示。",
                  flush=True)
            while True:
                # Do not silently leave orphaned simulator components if one
                # of the supervised launch processes exits unexpectedly.
                for name, process in run.processes:
                    if name != "bag" and process.poll() is not None:
                        raise RuntimeError("{} 提前退出，检查对应日志".format(name))
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("收到 Ctrl-C，正在关闭本次演示。", flush=True)
    except Exception as exc:
        run_error = str(exc)
        print("演示中断：{}".format(exc), file=sys.stderr, flush=True)
    finally:
        signal.signal(signal.SIGINT, cleanup_interrupt)
        # Save BEFORE teardown; includes manual walking after finite bag recording.
        if map_saver_started and rosgraph.is_master_online():
            try:
                result = rospy.ServiceProxy('/demo/save_map', Trigger)()
                print('退出前最终地图：{} {}'.format(result.success, result.message), flush=True)
                if not result.success:
                    run_error = run_error or result.message
            except Exception as exc:
                run_error = run_error or '最终地图未保存: ' + str(exc)
                print(run_error, file=sys.stderr, flush=True)
        # rosbag receives SIGINT first and gets time to close its index.
        if bag_process is not None and bag_process.poll() is None:
            try:
                os.killpg(bag_process.pid, signal.SIGINT)
                bag_process.wait(timeout=25)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        run.stop()

    if not bag.exists():
        print("未生成有效bag；检查 *_bag.log 和 *_gazebo.log", file=sys.stderr)
        return 1
    if analysis_returncode is None:
        analysis_returncode = subprocess.run(
            [sys.executable, str(analyzer), str(bag),
             "--mode", args.mode, "--output", str(prefix)],
            env=env, check=False).returncode
        print("数据目录：{}".format(output), flush=True)
    return analysis_returncode if analysis_returncode else (2 if run_error else 0)


if __name__ == "__main__":
    sys.exit(main())
