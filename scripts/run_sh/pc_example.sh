#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
export ASTRA_ROOT
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export PX4_DIR

# Make the script independent of a user-specific `astra` alias.
source /opt/ros/noetic/setup.bash

# PX4 is a ROS package tree only after these paths are exported. Without this,
# roslaunch cannot resolve px4/posix_sitl.launch or mavlink_sitl_gazebo.
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
if [[ -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" &&
      -d "$PX4_DIR/build/px4_sitl_default" ]]; then
    source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
        "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
else
    echo "错误：未找到 PX4 SITL 构建目录：$PX4_DIR/build/px4_sitl_default"
    echo "请先编译 PX4：cd $PX4_DIR && make px4_sitl gazebo"
    exit 1
fi

export EXAMPLE_WORLD="$ASTRA_ROOT/simulation/astra_gazebo_worlds/example.world"

# 防止重复启动：已有 ROS master 时直接退出。
if rosnode list >/dev/null 2>&1; then
    echo "检测到已有 ROS master 正在运行。"
    echo "请先停止旧仿真，再重新启动。"
    exit 1
fi

open_terminal() {
    local title="$1"
    local command="$2"

    gnome-terminal --title="$title" -- bash -ic "$command; exec bash"
}

# 1. ROS master
open_terminal "AstraDrone - ROS Core" \
'roscore'

# 2. PX4、Gazebo、Mid-360；必须在启动 Gazebo 前加载 PedSim 插件库。
open_terminal "AstraDrone - PX4 and Gazebo" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
export GAZEBO_PLUGIN_PATH="/home/a/pedsim_ws/devel/lib:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="/home/a/pedsim_ws/devel/lib:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:/home/a/pedsim_ws/src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models:${GAZEBO_MODEL_PATH}"
sleep 3
roslaunch "$ASTRA_ROOT/simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch" world:="$EXAMPLE_WORLD"'

# 3. PedSim：不启动第二个 Gazebo，只生成和更新行人。
open_terminal "AstraDrone - PedSim" \
'sleep 8
source /opt/ros/noetic/setup.bash
source /home/a/pedsim_ws/devel/setup.bash
roslaunch pedsim_gazebo_plugin astra_pedsim.launch'

# 4. FAST-LIO：等待 Gazebo 与行人生成完成。
open_terminal "AstraDrone - FAST-LIO" \
'sleep 12
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
echo "等待 Gazebo 发布 /livox/lidar ..."
for i in $(seq 1 60); do
    if [[ "$(rostopic type /livox/lidar 2>/dev/null)" == "sensor_msgs/PointCloud2" ]]; then
        break
    fi
    sleep 1
done
if [[ "$(rostopic type /livox/lidar 2>/dev/null)" != "sensor_msgs/PointCloud2" ]]; then
    echo "错误：未检测到 sensor_msgs/PointCloud2 类型的 /livox/lidar。请检查 Gazebo 插件路径和模型。"
    exit 1
fi

# example.world only contains ActorPosesPlugin; the moving people are spawned
# by the separate PedSim process. Do not silently start FAST-LIO without them.
echo "等待 PedSim 发布 /pedsim_simulator/simulated_agents ..."
for i in $(seq 1 30); do
    if [[ "$(rostopic type /pedsim_simulator/simulated_agents 2>/dev/null)" == "pedsim_msgs/AgentStates" ]]; then
        break
    fi
    sleep 1
done
if [[ "$(rostopic type /pedsim_simulator/simulated_agents 2>/dev/null)" != "pedsim_msgs/AgentStates" ]]; then
    echo "错误：未检测到 PedSim 行人状态。example.world 中没有内置行人，请检查 PedSim 终端。"
    exit 1
fi
if ! timeout 5 rostopic echo -n 1 /pedsim_simulator/simulated_agents >/dev/null 2>&1; then
    echo "错误：PedSim 话题存在但没有收到行人状态消息，行人可能没有生成。"
    exit 1
fi
roslaunch fast_lio mapping_mid360.launch rviz:=true dynamic_filter:=true sim_lidar:=true'

# 5. 键盘 / Offboard 控制
open_terminal "AstraDrone - Offboard Control" \
'sleep 16
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
roslaunch offboard keyboard_control.launch'

# 6. QGroundControl
open_terminal "AstraDrone - QGroundControl" \
'qgc'
