#!/usr/bin/env bash
# One-command PX4 SITL + FAST-LIO + EXP dynamic-avoidance demonstration.
# Pedestrians are intentionally started after the static background is ready.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PEDSIM_WS="${PEDSIM_WS:-$HOME/pedsim_ws}"
# true: use the YAML automatic target; false: take off, hover and wait for an
# RViz 2D Nav Goal or a /move_base_simple/goal message.
AUTO_GOAL="${AUTO_GOAL:-true}"
export ASTRA_ROOT PX4_DIR PEDSIM_WS AUTO_GOAL

source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"

if [[ ! -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ||
      ! -d "$PX4_DIR/build/px4_sitl_default" ]]; then
    echo "错误：PX4 SITL 未就绪：$PX4_DIR/build/px4_sitl_default"
    exit 1
fi
if [[ ! -f "$PEDSIM_WS/devel/setup.bash" ]]; then
    echo "错误：PedSim 工作区不存在：$PEDSIM_WS"
    exit 1
fi
if rosnode list >/dev/null 2>&1; then
    echo "检测到已有 ROS master，请先结束旧仿真，避免多个控制器同时发指令。"
    exit 1
fi

open_terminal() {
    local title="$1"
    local command="$2"
    gnome-terminal --title="$title" -- bash -ic "$command; exec bash"
}

open_terminal "Dynamic Avoidance - ROS Core" 'roscore'

open_terminal "Dynamic Avoidance - PX4 Gazebo" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
export GAZEBO_PLUGIN_PATH="$PEDSIM_WS/devel/lib:$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$PEDSIM_WS/devel/lib:$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:$PEDSIM_WS/src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models:${GAZEBO_MODEL_PATH}"
sleep 3
roslaunch "$ASTRA_ROOT/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch"'

open_terminal "Dynamic Avoidance - FAST-LIO" \
'sleep 10
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
for i in $(seq 1 45); do
    [[ "$(rostopic type /livox/lidar 2>/dev/null)" == "sensor_msgs/PointCloud2" ]] && break
    sleep 1
done
if [[ "$(rostopic type /livox/lidar 2>/dev/null)" != "sensor_msgs/PointCloud2" ]]; then
    echo "错误：/livox/lidar 未就绪。"
    exit 1
fi
# EXP 必须看到原始注册点云；这里关闭 FAST-LIO 内部动态过滤，避免动态点在检测前被删除。
roslaunch fast_lio mapping_mid360.launch rviz:=false sim_lidar:=true dynamic_filter:=false'

open_terminal "Dynamic Avoidance - Perception and Control" \
'sleep 14
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
roslaunch fastlio_bridge astra_dynamic_avoidance.launch enable_control:=true auto_arm:=true auto_goal:="$AUTO_GOAL" rviz:=true'

open_terminal "Dynamic Avoidance - Delayed Pedestrians" \
'sleep 16
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
source "$PEDSIM_WS/devel/setup.bash"
# 两个 overlay 的 setup.bash 会互相覆盖包路径；显式保留项目与 PedSim。
export ROS_PACKAGE_PATH="$ASTRA_ROOT/AstraDrone_ros1_ws/src:$PEDSIM_WS/src:${ROS_PACKAGE_PATH}"
echo "等待无人机完成起飞并开始导航……"
if ! timeout 75 rostopic echo -n1 /mavros_avoidance_controller/navigation_ready 2>/dev/null | grep -q "data: True"; then
    echo "错误：75 秒内未收到导航就绪信号，PedSim 不会启动。请检查 OFFBOARD/解锁状态。"
    exit 1
fi
echo "导航已就绪，开始移动行人。"
# 行人已预置在 example.world；这里只发布轨迹，避免 Gazebo 运行时插入模型的竞态崩溃。
roslaunch "$ASTRA_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/astra_pedestrians.launch"'

echo "动态避障演示已启动。无人机将自动起飞并沿 +Y 飞行 12 m，与 x 向行人轨迹交叉。"
echo "RViz 中紫色为全局 A* 路径，橙色为局部实时路径，青色为实际飞行路径。"
echo "绿色为清理后的全局地图，黄色为滚动局部地图，红色为确认动态点。"
echo "淡青色为当前激光确认的局部自由空间；它会立即修正橙色路径并逐步清理绿色旧障碍。"
echo "不要再启动 keyboard_control 或其他 Offboard 控制节点。"
