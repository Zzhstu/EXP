#!/usr/bin/env bash
# One-command, repeatable loop-closure benchmark:
# PX4/Gazebo -> FAST-LIO2 -> loop backend/RViz -> bag recorder -> square flight.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOOP_BAG_DIR="$ASTRA_ROOT/loop_closure_runs"
LOOP_BAG_PREFIX="$LOOP_BAG_DIR/loop_closure_${RUN_STAMP}"
export ASTRA_ROOT PX4_DIR LOOP_BAG_DIR LOOP_BAG_PREFIX

source /opt/ros/noetic/setup.bash
if [[ -f "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash" ]]; then
  source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
fi
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH:-}"

if [[ ! -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ||
      ! -d "$PX4_DIR/build/px4_sitl_default" ]]; then
  echo "错误：PX4 SITL 未就绪：$PX4_DIR"
  exit 1
fi
if [[ ! -x "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/lib/fastlio_bridge/loop_closure_backend_node" ||
      ! -x "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/lib/offboard/loop_closure_demo_flight" ]]; then
  echo "错误：新节点尚未编译。请先执行："
  echo "  cd $ASTRA_ROOT/AstraDrone_ros1_ws && catkin_make --pkg fastlio_bridge offboard -j2 -l2"
  exit 1
fi
if rosnode list >/dev/null 2>&1; then
  echo "错误：检测到已有 ROS master。请先结束旧仿真，避免多个 PX4/控制器冲突。"
  exit 1
fi
# A crashed/closed roscore can leave native nodes alive. They may reconnect to
# the new master and silently create duplicate publishers even though
# `rosnode list` above reports no master.
if pgrep -x gzserver >/dev/null 2>&1 ||
   pgrep -x px4 >/dev/null 2>&1 ||
   pgrep -x fastlio_mapping >/dev/null 2>&1 ||
   pgrep -f '/loop_closure_backend_node( |$)' >/dev/null 2>&1; then
  echo "错误：检测到旧 Gazebo/PX4/FAST-LIO/回环进程。请先结束这些残留进程："
  pgrep -a -x gzserver || true
  pgrep -a -x px4 || true
  pgrep -a -x fastlio_mapping || true
  pgrep -af '/loop_closure_backend_node( |$)' || true
  exit 1
fi
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：需要 gnome-terminal 来分窗口显示各模块日志。"
  exit 1
fi
mkdir -p "$LOOP_BAG_DIR"

open_terminal() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="$title" -- bash -ic "$command; exec bash"
}

open_terminal "Loop Demo 1/6 - ROS Core" 'roscore'

open_terminal "Loop Demo 2/6 - PX4 Gazebo" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH}"
sleep 3
roslaunch "$ASTRA_ROOT/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" world:="$ASTRA_ROOT/simulation/astra_gazebo_worlds/fastlio_bench/loop_closure_arena.world"'

open_terminal "Loop Demo 3/6 - FAST-LIO2" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
for i in $(seq 1 60); do
  [[ "$(rostopic type /livox/lidar 2>/dev/null)" == "sensor_msgs/PointCloud2" ]] && break
  sleep 1
done
if [[ "$(rostopic type /livox/lidar 2>/dev/null)" != "sensor_msgs/PointCloud2" ]]; then
  echo "错误：60 秒内未出现 /livox/lidar。"
  exit 1
fi
# 回环后端需要完整注册点云，因此基准实验关闭前端动态点过滤。
roslaunch fast_lio mapping_mid360.launch rviz:=false sim_lidar:=true dynamic_filter:=false'

open_terminal "Loop Demo 4/6 - Backend and RViz" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
for i in $(seq 1 60); do
  [[ "$(rostopic type /Odometry 2>/dev/null)" == "nav_msgs/Odometry" ]] && break
  sleep 1
done
roslaunch fastlio_bridge loop_closure_demo.launch rviz:=true'

open_terminal "Loop Demo 5/6 - Bag and Report" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
for i in $(seq 1 90); do
  [[ "$(rostopic type /uav1/loop_closure/diagnostics 2>/dev/null)" == "diagnostic_msgs/DiagnosticArray" ]] && break
  sleep 1
done
echo "录包位置：${LOOP_BAG_PREFIX}.bag"
rosbag record --lz4 -O "$LOOP_BAG_PREFIX" \
  /gazebo/model_states \
  /uav1/fastlio/odom /uav1/fastlio/registered_scan \
  /uav1/loop_closure/odom /uav1/loop_closure/raw_path \
  /uav1/loop_closure/path /uav1/loop_closure/constraints \
  /uav1/loop_closure/diagnostics /uav1/loop_closure/map &
BAG_PID=$!
trap '\''kill -INT "$BAG_PID" 2>/dev/null; wait "$BAG_PID" 2>/dev/null'\'' EXIT INT TERM
if timeout 240 rostopic echo /loop_closure_demo/finished | grep -m1 -q "data: True"; then
  echo "航线完成，正在安全结束 rosbag……"
else
  echo "警告：240 秒内没有完成信号，仍会保存当前数据。"
fi
kill -INT "$BAG_PID" 2>/dev/null
wait "$BAG_PID" 2>/dev/null
trap - EXIT INT TERM
python3 "$ASTRA_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/analyze_loop_closure_bag.py" "${LOOP_BAG_PREFIX}.bag" --output "$LOOP_BAG_PREFIX"
echo "报告：${LOOP_BAG_PREFIX}_report.txt"'

open_terminal "Loop Demo 6/6 - Automatic Flight" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
for i in $(seq 1 120); do
  ODOM_TYPE="$(rostopic type /uav1/fastlio/odom 2>/dev/null)"
  CLOUD_TYPE="$(rostopic type /uav1/fastlio/registered_scan 2>/dev/null)"
  DIAG_TYPE="$(rostopic type /uav1/loop_closure/diagnostics 2>/dev/null)"
  [[ "$ODOM_TYPE" == "nav_msgs/Odometry" && "$CLOUD_TYPE" == "sensor_msgs/PointCloud2" && "$DIAG_TYPE" == "diagnostic_msgs/DiagnosticArray" ]] && break
  sleep 1
done
if [[ "$ODOM_TYPE" != "nav_msgs/Odometry" || "$CLOUD_TYPE" != "sensor_msgs/PointCloud2" || "$DIAG_TYPE" != "diagnostic_msgs/DiagnosticArray" ]]; then
  echo "错误：SLAM/回环话题未就绪，不执行自动飞行。"
  exit 1
fi
roslaunch offboard loop_closure_demo_flight.launch'

echo "完整回环实验已经启动。"
echo "场景：simulation/astra_gazebo_worlds/fastlio_bench/loop_closure_arena.world"
echo "航线：10 m 方形闭环；黄色=原始关键帧轨迹，绿色=优化轨迹，约束线=已接受回环。"
echo "飞行结束后会自动停止录包并生成：${LOOP_BAG_PREFIX}_report.txt"
