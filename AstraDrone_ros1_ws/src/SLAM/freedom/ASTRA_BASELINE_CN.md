# FreeDOM 在 AstraDrone 中的独立基线

来源：[LC-Robotics/FreeDOM](https://github.com/LC-Robotics/FreeDOM)，固定上游提交 `dcfd5690cafddf121a80bd8ea477807d9656748e`，MIT 许可（见同目录 `LICENSE`）。论文：Li 等，*FreeDOM: Online Dynamic Object Removal Framework for Static Map Construction Based on Conservative Free Space Estimation*，IEEE RA-L 2025，DOI 10.1109/LRA.2025.3560881。这里保留原作者前端扫描过滤和后端地图细化代码；适配只涉及ROS输入帧校验、队列、日志、参数及启动/导出。不是 ours 的新算法。

## 启动

首次编译：

```bash
cd /home/a/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg freedom -j2
```

另开终端运行一次完整仓库探索演示，同时生成两张地图：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_exploration_demo.sh --freedom-baseline
```

无GUI短时验收（**只生成部分地图**，不是完整探索）：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --headless --test-seconds 120 --freedom-baseline
```

默认探索仍由现有 static_map_builder / explorer / 控制器执行；FreeDOM **并行只读**，不发布到 `/uav1/fastlio/cloud_map`，不改变避障决策。不要再同时启动另一套带控制器的 demo。终端会打印本次 `exploration_demo_runs/warehouse_<时间>` 前缀。

## 输入、输出和坐标系

| 内容 | ours | FreeDOM |
|---|---|---|
| 扫描 | `/cloud_registered` 经 cloud_bridge 对齐后的 `/uav1/fastlio/registered_scan` | `/cloud_registered_body`（FAST-LIO 去畸变的 IMU/body 帧） |
| 位姿 | FAST-LIO `/Odometry` 经当前演示初始对齐 | 扫描时刻 `camera_init→body` TF |
| 在线地图 | `/uav1/fastlio/cloud_map`，`map` 帧 | `/uav1/freedom/static_map`，`camera_init` 帧；只有订阅者存在时原作者可视化才构造整图 |
| 落盘 | `<前缀>_final_map.pcd` | `<前缀>_freedom_static_map_point.pcd` 和 `<前缀>_freedom_static_map_voxel.pcd` |

`/cloud_registered` 已在 `camera_init` 帧，绝不能作为传感器局部点云再应用 TF。FreeDOM 保留 `camera_init` 坐标；两地图不能直接叠加或逐点差分。运行器只在评估侧额外保存 `*_alignment.json` 与 `*_freedom_alignment.json`（Gazebo world 分别到 map 和 camera_init 的初始估计），不会提供给探索器或规划器。双方的 `*_coverage.json` 使用同一个货架碰撞盒侧面、相同高度和0.25m容差审计；初始真值接收配对不是长期漂移校正，覆盖率并非动态清理/SLAM精度。

FreeDOM 的 `config/astra_mid360.yaml` 保守地关闭了射线增强：原仓库提供的是 Livox 80°、Velodyne 等数据集配置，项目里没有经标定的 MID360 非重复扫描 FOV 掩膜。使用那些掩膜或把 FAST-LIO 配置中的 `fov_degree: 360` 当作有效自由空间，会错误清除未观测障碍。这一配置保留 FreeDOM 的扫描过滤/自由射线/地图细化，但**不是论文完整参数复现**。`counts_to_free=6`、`counts_to_revert=20`、`sub_voxel_size=0.10m`、`voxel_depth=2`（自由体素0.40m）均须与 ours 的阈值/分辨率一同报告；不宜只凭最终点数排名。

手动保存正在运行的基线：

```bash
rostopic pub -1 /freedom/save_map std_msgs/Empty '{}'
```

演示运行器结束时也会自动触发保存。若想在 RViz 看 FreeDOM 地图，添加 PointCloud2 显示并将 Topic 设为 `/uav1/freedom/static_map`，Fixed Frame 选 `camera_init`；查看 ours 则选 `/uav1/fastlio/cloud_map` 与 `map`。不要以单张截图宣称哪种方法更优。

## 目前基线的验收边界

已经核验编译、ROS启动、同场景输入及两份地图导出；短时演示仍可能在探索中被显式时限截断。发表级对照仍需录制同一原始扫描/位姿，报告静止行人进入地图后离开旧位置的残影点数与清除延迟、静态货架保留/误删、可达通道恢复、处理时延P95、CPU/内存，并对时间预算、观察机会和位姿系保持一致。当前并行模式仅比较地图，不构成“换用FreeDOM规划后无人机到达率改善”的证据。

2026-09-30 的一组100墙钟秒短测（`warehouse_20260930_095121`）在各自初始world对齐下，参考货架侧面支持为 ours 7151/10192（70.16%）、FreeDOM 7881/10192（77.33%）。这只是该次部分航线的表面匹配比例，配置/分辨率不同，且未对动态旧位置和误删静态表面作标注，不能推论算法总体优劣。最终保存完成确认代码另经80墙钟秒短测（`warehouse_20260930_100425`）验证：FreeDOM 83792点PCD、保存回执与两侧覆盖JSON均生成；该轮仅约6.2%的参考面支持，说明时间预算截断/航线进度对覆盖数字影响极大，不宜跨不同运行直接比较。
