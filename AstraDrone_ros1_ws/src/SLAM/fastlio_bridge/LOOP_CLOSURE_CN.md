# AstraDrone FAST-LIO2 回环后端

## 设计来源与边界

后端结合两个参考仓库的核心思路，但没有直接复制其 GPLv2 源码：

- `FAST_LIO_SAM`：关键帧、相邻里程计约束、当前帧到历史子地图 ICP、回环后统一优化历史位姿；
- `better_fastlio2`：先用里程计空间距离产生候选，再用 Scan Context 排除外观不一致候选，最后才运行 ICP。

针对 MID360 非重复扫描和无人机高度变化，当前实现又加入了：连续 4 个关键帧组成描述子子地图、局部地面分位数高度归一化、全部有界空间候选的 Scan Context 比较、Scan Context 航向种子、多候选 ICP，以及仅在近距离启用的几何兜底。描述子不再是单帧“一票否决”，但最终仍必须通过 ICP、重叠率和最大修正硬门限。

当前机器没有 GTSAM。为保证 ROS Noetic 工程不增加系统依赖，包内实现稀疏 Gauss-Newton SE(3) 位姿图，固定第一帧并对回环残差使用 Huber 鲁棒核。这是工程后端，不应写成新的位姿图算法。

## 为什么不直接修改飞控里程计

FAST-LIO 原始 `/uav1/fastlio/odom` 连续、低延迟，适合 PX4 控制；回环会一次性修正历史漂移，直接反馈给飞控可能产生位置跳变。因此保留原话题，另行发布：

- `/uav1/loop_closure/odom`：应用最新全局修正的里程计；
- `/uav1/loop_closure/path`：优化关键帧轨迹；
- `/uav1/loop_closure/raw_path`：未优化关键帧轨迹，用于对比；
- `/uav1/loop_closure/map`：按优化关键帧重建的全局地图；
- `/uav1/loop_closure/registered_scan`：应用全局修正的当前帧；
- `/uav1/loop_closure/constraints`：回环节点和红色约束边；
- `/uav1/loop_closure/diagnostics`：关键帧及接受/拒绝计数；
- `/uav1/loop_closure/map_to_raw`：数值修正消息，不广播冲突 TF。

在闭环飞行测试完成前，不要把优化里程计 remap 到 MAVROS vision pose 或控制器输入。

## 一键完整演示（推荐）

先编译一次：

```bash
cd /home/a/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg fastlio_bridge offboard -j2 -l2
```

随后只运行：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/loop_closure_demo.sh
```

脚本会启动专用非对称场景 `loop_closure_arena.world`、PX4/MID360、FAST-LIO、回环后端和 RViz，然后自动执行 10 m 方形航线并返回起点。黄色是原始关键帧轨迹，绿色是优化轨迹，约束 Marker 中的红线是已接受回环。飞行完成后 rosbag 自动停止，并在 `loop_closure_runs/` 生成：

- `loop_closure_时间.bag`：含原始/优化里程计、当前注册点云、真值、路径、约束和诊断；
- `loop_closure_时间_report.txt`：ATE、5 秒 RPE、闭环位移误差和诊断统计；
- `loop_closure_时间_trajectory.png`：Gazebo 真值、原始和优化 XY 轨迹。

注意：脚本检测到已有 ROS master 或残留的 Gazebo/PX4/FAST-LIO/回环原生进程时会拒绝启动，避免重复发布者。不要并行启动 `keyboard_control`。

## 分步启动与观察

正常启动 `fastlio_bridge/perception.launch` 时后端默认启动，也可单独执行：

```bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
roslaunch fastlio_bridge loop_closure.launch
```

RViz Fixed Frame 使用 `map`，添加：

1. Path `/uav1/loop_closure/raw_path`（黄色）；
2. Path `/uav1/loop_closure/path`（绿色）；
3. MarkerArray `/uav1/loop_closure/constraints`；
4. PointCloud2 `/uav1/loop_closure/map`。

无人机必须离开起点并在至少 `min_loop_time` 秒后真正重访旧区域。原地悬停或从不返回时没有回环是正确行为。

## 误闭环保护

一条回环必须同时满足：关键帧间隔、时间间隔、三维空间半径、垂直分离、Scan Context（或严格受限的近距离几何兜底）、ICP fitness、ICP 重叠率、最大修正量和优化器数值检查；接受后还有关键帧冷却，避免同一次重访连续添加高度相关的冗余约束。`max_loop_vertical_separation: 1.0` 很重要：它阻止起飞前地面扫描与巡航高度返航扫描形成假约束。默认阈值偏保守，因为漏检只是不纠正，误闭环却可能扭曲整张地图。调参应先录包并逐条检查约束边，不要同时放宽全部门限。

诊断中的 `no_spatial_candidates`、`rejected_scan_context`、`icp_attempts`、`rejected_icp` 和 `optimizer_failures` 已分开计数；`last_*` 字段会给出最近候选的距离、SC、航向偏移、ICP、重叠率和处理耗时。

## 完成后的优点

- 重访区域可约束累计漂移，减轻起终点错位和重复墙面；
- 轨迹与重建地图使用同一组优化位姿，不会只改轨迹不改地图；
- Scan Context 与 ICP 双重验证比单纯距离+ICP更不易在重复走廊误匹配；
- 重叠率和最大修正硬门限阻止局部小平面造成的假收敛；
- 回环低频执行，关键帧和点数有上限，不替换 FAST-LIO 高频前端；
- 控制里程计与优化结果分离，便于安全对比和逐步接入规划。

## 已完成的运行验证与结果解释

2026-09-23 在专用场景完成了真实 PX4 SITL + Gazebo + MID360 全航线验证：43 个关键帧，接受 1 条 `3 <-> 40` 回环，空间距离 0.832 m，SC 0.275，ICP fitness 0.0165，重叠率 1.00，优化器失败 0。合成测试也接受 1 条回环，ICP fitness 0.0054、重叠率 1.00。

该仿真的 FAST-LIO 前端本身异常准确：原始对齐 ATE RMSE 约 1.23 cm。加入回环后为 1.40 cm，差值 1.7 mm；这不表示回环失效，而表示短距离、无噪声仿真中没有足够漂移可供后端纠正，ICP 的毫米级噪声反而略占上风。垂直门限加入前错误连接了地面第 0 帧，优化 ATE 曾升至 2.41 cm；修复后已消除该问题。论文实验应另外使用长航程、噪声/退化场景证明“有漂移时的改善”，并同时报告无漂移场景中的不劣化程度，不能只展示一条红色回环线。

离线复算已有 bag：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosrun fastlio_bridge analyze_loop_closure_bag.py your.bag --output /tmp/loop_result
```

## 后续长航程实验

一次短程真实仿真不能证明所有场景都绝无误闭环。后续应在长走廊、楼层高度变化和多圈航线中继续记录 ATE/RPE、闭环前后起终点误差、每条回环的 SC/ICP/重叠率、地图重影、CPU 峰值和处理延迟。关键帧地图来自注册点云，回环本身不会自动完成动态语义清图；动态行人场景还需检查历史人体是否重新进入重建地图。
