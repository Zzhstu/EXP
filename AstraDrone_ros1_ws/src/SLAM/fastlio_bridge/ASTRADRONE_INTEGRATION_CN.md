# AstraDrone 动态避障集成

该目录来自 `EXP-main`，保留其 FAST-LIO 动态感知主链，并增加了当前工程所需的原生 MAVROS 控制器。

## 一键启动

```bash
bash /home/a/AstraDroneOpen/scripts/run_sh/pc_dynamic_avoidance.sh
```

启动链路如下：

```text
/cloud_registered + /Odometry
  -> 坐标对齐与点云桥接
  -> 初始背景差分
  -> 聚类、轨迹关联、速度与动态确认
  -> 动态 CPA/TTC 风险 + 静态净空风险
  -> 风险融合
  -> MAVROS 速度控制
```

脚本先启动静态场景、学习 40 帧背景并完成自动起飞。PedSim 不依赖固定延时，而是等待控制器锁存发布 `~navigation_ready=true` 后才开始运动，因此不同机器上也会在无人机开始水平导航时形成交叉冲突。控制器自动起飞至 1.5 m，并沿相对 `+Y` 方向飞行 6 m；行人在 `y=2.5 m` 沿 X 方向单次横穿并停在场边。两个行人模型已经预置在 `example.world`，不要再运行 `spawn_pedsim_agents.py`；该脚本与 Gazebo 世界插件并发访问模型列表时可能导致 `gzserver` 段错误。

默认场景使用 `config/scenarios/astra_crossing_once.xml`。不要在同一局部通道中让行人无限往返：无人机第一次让行后进入通道，后续行人再次横穿属于需要全局时空规划的持续交通流问题，单次局部 stop-and-wait 无法保证安全。

动态检测默认只处理无人机周围 6 m，并把连续 20 帧稳定的新体素并入背景。这两个参数用于抑制无人机运动后新进入视野的树木、墙面被误判为动态物体；若行人速度低于约 0.2 m/s，应继续增大 `online_static_confirm_frames`，避免慢目标被背景吸收。

## 关键话题

```bash
rostopic hz /uav1/foreground_points
rostopic echo /uav1/dynamic_objects
rostopic echo /uav1/fused_collision_risk_level
rostopic echo /uav1/fused_avoidance_velocity
rostopic echo /mavros/setpoint_velocity/cmd_vel
rostopic echo /uav1/dynamic_system/diagnostics
```

RViz 可添加：`/uav1/fastlio/registered_scan`（当前帧）、`/uav1/fastlio/cloud_map`（局部观测持续更新的全局图）、`/uav1/local_static_map`、`/uav1/dynamic_points` 和 `/uav1/dynamic_markers`。固定坐标系使用 `map`。

## 安全提醒

- `astra_dynamic_avoidance.launch` 默认启用控制，仅面向 PX4 SITL。
- 真机首次验证必须传入 `enable_control:=false auto_arm:=false`，只观察私有话题 `~command_preview`。
- 不得同时运行键盘、长走廊自动航线或其他 `/mavros/setpoint_*` 发布节点。
- 感知、里程计或风险速度任一超时，控制器会把 XY 指令置零并保持高度。
