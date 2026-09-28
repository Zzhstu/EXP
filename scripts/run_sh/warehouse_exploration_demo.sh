#!/usr/bin/env bash
# Independent exploration entry; old dynamic comparison scripts unchanged.
set -euo pipefail
ASTRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PEDSIM_WS="${PEDSIM_WS:-$HOME/pedsim_ws}"
export ASTRA_ROOT PX4_DIR PEDSIM_WS
if [[ ! -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ||
      ! -d "$PX4_DIR/build/px4_sitl_default" ||
      ! -f "$PEDSIM_WS/devel/setup.bash" ]]; then
  echo "错误：PX4 Gazebo Classic 或 PedSim 模型环境不完整。" >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$ASTRA_ROOT/AstraDrone_ros1_ws/src:$PEDSIM_WS/src:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:${ROS_PACKAGE_PATH:-}"
exec /usr/bin/python3 "$ASTRA_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/run_warehouse_exploration.py" "$@"
