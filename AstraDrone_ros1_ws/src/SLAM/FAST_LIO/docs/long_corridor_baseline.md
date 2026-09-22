# FAST-LIO 长走廊退化基线

场景文件为 `simulation/astra_gazebo_worlds/fastlio_bench/long_corridor_degenerate.world`。走廊沿 Gazebo 的 +X 方向延伸 200 m，宽 5 m、高 4.05 m；起点在 x=0，端墙在 x=±100 m。因此按 0 到 +60 m 的测试轨迹飞行时，MID360 的 40 m 最大量程内主要只有两面平行墙、地面和天花板。

这不是普通避障场景：它特意去掉了柱、门、标牌、行人和重复的离散物体。对点到平面残差来说，沿走廊轴向平移以及部分偏航变化的约束较弱，能放大 FAST-LIO 的 IMU 漂移、外参/时间同步误差和退化情况下的状态估计行为。

## 启动

最简方式是直接运行：

```bash
bash /home/a/AstraDroneOpen/scripts/run_sh/fastlio_long_corridor_baseline.sh
```

脚本会启动 ROS、PX4/Gazebo、原始 FAST-LIO、记录器和自动航线，不启动 PedSim。自动航线以当前 MAVROS 局部位置为原点：起飞 1.5 m、沿 `+X` 飞 60 m、悬停 3 秒、原路返回并降落。若需要手动分终端启动，在已经 `source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash` 且 PX4 Gazebo 环境已配置的终端中，依次运行：

```bash
roslaunch /home/a/AstraDroneOpen/simulation/px4_sim_files/px4_launch/astra_launch/astra_long_corridor.launch
roslaunch fast_lio mapping_mid360.launch rviz:=true sim_lidar:=true dynamic_filter:=false
roslaunch fast_lio long_corridor_baseline.launch
roslaunch offboard long_corridor_baseline_flight.launch
```

`dynamic_filter:=false` 是有意设置：这次记录的是未加入动态点过滤的 FAST-LIO 基线。`fastlio_baseline_recorder.py` 将 `/Odometry` 与 `/gazebo/model_states` 的 `iris_mid360` 真值在第一对样本处对齐；随后输出的是局部漂移，而不是两个世界坐标原点不同造成的常量偏移。

## 航线参数与注意事项

不要同时启动 `keyboard_control.launch` 与自动航线，两者都会向 `/mavros/setpoint_position/local` 发布目标。默认航线参数可以覆盖，例如：

```bash
roslaunch offboard long_corridor_baseline_flight.launch corridor_distance:=40 cruise_speed:=0.6 auto_land:=false
```

`max_tracking_error` 默认为 3 m；超过该阈值会停止路径推进并保持当前位置，且不会自动降落，便于检查仿真或控制异常。这个脚本是为当前 PX4 SITL/Gazebo 场景编写的，真机使用前必须重新做安全审查和限幅。

## 等效人工飞行流程

1. 起飞到约 1.5 m，悬停 5 秒，等待 FAST-LIO 初始化稳定。
2. 保持朝 +X，低速匀速飞到 x≈+60 m；建议速度 0.8–1.0 m/s，尽量不做横移或转向。
3. 悬停 3 秒后沿原路返回起点，降落并结束 recorder。
4. 每套参数至少重复 3 次；比较同一路径长度下的结果，不能只看单次最终误差。

记录器会在 `~/.ros/fastlio_baselines/` 写出同名前缀的 CSV 和 JSON。JSON 包括：路径长度、3D/XY RMSE、最大 3D 误差、最终位置误差和最终偏航误差。CSV 可用于画“误差—距离”曲线。

注意：Gazebo `model_states` 没有消息时间戳，记录器使用到达时的仿真时间与 FAST-LIO 里程计配对。对正常实时仿真足够做基线；若要发表级评估，应录 rosbag 后采用带插值的离线真值关联。
