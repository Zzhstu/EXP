#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# ROBOCUP_WORLD="$ASTRA_ROOT/simulation/astra_gazebo_worlds/RoboCup_sim/RoboCup_sim.world"
ROBOCUP_WORLD="$ASTRA_ROOT/simulation/astra_gazebo_worlds/RoboCup_sim/RoboCup_sim_pedsim.world"
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

# 1. ROS master：保持前台运行，Ctrl+C 可以正常停止。
open_terminal "AstraDrone - ROS Core" \
    "roscore"

# 2. PX4 与 Gazebo。
open_terminal "AstraDrone - PX4 and Gazebo" \
    # "sleep 3; roslaunch px4 astra_example.launch world:=$ROBOCUP_WORLD"
export GAZEBO_PLUGIN_PATH="/home/a/pedsim_ws/devel/lib:${GAZEBO_PLUGIN_PATH}";
export LD_LIBRARY_PATH="/home/a/pedsim_ws/devel/lib:${LD_LIBRARY_PATH}";
export GAZEBO_MODEL_PATH="/home/a/pedsim_ws/src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models:${GAZEBO_MODEL_PATH}";
sleep 3;
roslaunch px4 astra_example.launch world:=$ROBOCUP_WORLD
# 3. FAST-LIO。
open_terminal "AstraDrone - FAST-LIO" \
    "sleep 6; astra && roslaunch fast_lio mapping_mid360.launch rviz:=true"

# 4. 键盘 / Offboard 控制。
open_terminal "AstraDrone - Offboard Control" \
    "sleep 10; astra && roslaunch offboard keyboard_control.launch"

# 5. QGroundControl。
open_terminal "AstraDrone - QGroundControl" \
    "qgc"
