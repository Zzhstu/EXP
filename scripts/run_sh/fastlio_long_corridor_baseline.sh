#!/usr/bin/env bash
# Launch the first FAST-LIO degeneration benchmark.  Unlike pc_example.sh,
# this deliberately does not start PedSim or the dynamic-point filter.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ASTRA_ROOT PX4_DIR

source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
if [[ ! -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" || ! -d "$PX4_DIR/build/px4_sitl_default" ]]; then
    echo "错误：PX4 SITL 未就绪：$PX4_DIR/build/px4_sitl_default"
    exit 1
fi

if rosnode list >/dev/null 2>&1; then
    echo "检测到已有 ROS master；请停止旧仿真后再启动基线场景。"
    exit 1
fi

open_terminal() {
    local title="$1"
    local command="$2"
    gnome-terminal --title="$title" -- bash -ic "$command; exec bash"
}

open_terminal "FAST-LIO Corridor - ROS Core" 'roscore'
open_terminal "FAST-LIO Corridor - PX4 and Gazebo" \
'source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH}"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH}"
sleep 3
roslaunch "$ASTRA_ROOT/simulation/px4_sim_files/px4_launch/astra_launch/astra_long_corridor.launch"'
open_terminal "FAST-LIO Corridor - Mapping" \
'sleep 12
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
roslaunch fast_lio mapping_mid360.launch rviz:=true sim_lidar:=true dynamic_filter:=false'
open_terminal "FAST-LIO Corridor - Baseline Recorder" \
'sleep 14
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
roslaunch fast_lio long_corridor_baseline.launch'
open_terminal "FAST-LIO Corridor - Automatic Route" \
'sleep 16
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
roslaunch offboard long_corridor_baseline_flight.launch'

echo "长走廊基线已启动。结束 recorder 后，结果位于 ~/.ros/fastlio_baselines/。"
