# 无回环版：SLAM、地图维护与动态避障对照演示

本演示使用仓库现有的 `example.world` 和已预置的两名行人；PedSim 让行人从左向右穿越无人机通往 `+Y` 的航线。它不下载新场景，也不启动回环后端。两次运行使用同一 MID360、FAST-LIO2、动态检测、风险融合、安全控制器、目标高度和速度；每次都重启 PX4/Gazebo，并重新建立地图。

| 模式 | 导航地图 | 规划 |
|---|---|---|
| `badcase` | 仍累计旧障碍；关闭动态轨迹清理和局部自由空间清理 | 只使用持续地图的全局 A* |
| `ours` | 动态轨迹清理、重复射线自由证据及滚动局部占用 | 全局 A* + 当前局部占用/自由证据重规划；旧全局地图封路时仍尝试局部恢复 |

这是本项目内部的组合消融，不代表“原始 FAST-LIO2 自带 A*”或与外部算法的公平性能对比。两模式都保留动态风险检测和紧急停/躲，因此 `badcase` 并非关掉安全保护让无人机撞人。FAST-LIO2 内部的 `/Laser_map` 与本演示的可变导航地图 `/uav1/fastlio/cloud_map` 是不同存储；当前对照比较后者，不声称擦除了前者的 ikd-Tree。

## 运行

首次编译：

```bash
cd /home/a/AstraDroneOpen/AstraDrone_ros1_ws
catkin_make --pkg fastlio_bridge -j2 -l2
```

依次运行，两次不能并发。默认情况下，到达目标并稳定悬停后继续观察地图 8 秒，随后结束录包、生成报告，但无人机保持悬停，Gazebo/RViz 和 ROS 节点**不会自行退出**。看完后在启动命令的终端按 `Ctrl-C`，等待本次进程关闭，再启动另一模式。若在观察时限内没有到达目标，录包仍会结束并将该段标为未到达样本，但仿真与导航继续运行；它不会自动关掉窗口。

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/dynamic_comparison_demo.sh badcase
bash scripts/run_sh/dynamic_comparison_demo.sh ours
```

没有显示器时加 `--headless`。可用 `--max-flight-seconds 200` 延长录包/到达判定窗口；它不控制仿真的退出。若做无人值守回归测试，可加 `--auto-stop` 恢复生成报告后自动结束的行为；现场展示不要加。超时样本应保留，不能只展示到达样本。脚本拒绝已有 ROS master、gzserver、PX4 或 FAST-LIO 进程，避免把两次试验混在同一个地图里。运行时不要再启动键盘控制或别的 Offboard 控制器。需要提前具备 PX4 Gazebo Classic 和 `~/pedsim_ws` 的编译环境；脚本会检查。
默认录包包含真值、地图、轨迹、目标和风险，不包含体积很大的原始 Livox 包；若要保留原始激光与 IMU 数据供后续算法重放，可在命令末尾加 `--full-bag`，并预留足够磁盘空间。报告反映录包截止前的状态；截止后仍可在 RViz 实时观察，但后续变化不会自动写进已生成的报告。

结果位于 `dynamic_demo_runs/`，每次生成独立 `*.bag`、`*_report.txt`、`*_metrics.json`、`*_trajectory.png` 和各模块日志。比较最新的两次：

```bash
cd /home/a/AstraDroneOpen
BAD=$(ls -t dynamic_demo_runs/badcase_*_metrics.json | head -1)
OURS=$(ls -t dynamic_demo_runs/ours_*_metrics.json | head -1)
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/compare_dynamic_demo.py "$BAD" "$OURS"
```

需要重新分析某个 bag 时：

```bash
source /opt/ros/noetic/setup.bash
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/analyze_dynamic_demo_bag.py \
  dynamic_demo_runs/某次运行.bag --mode ours
```

## 2026-09-23 的一次实测示例

两次均用 `--headless --max-flight-seconds 150` 运行，均到达并稳定悬停。badcase 用时 61.70 s、Gazebo 真值平面路程 20.30 m；ours 用时 37.38 s、路程 12.42 m，分别少 24.32 s 和 7.87 m（约 39.4% 和 38.8%）。行人离开初始位置后，该区域在最终稳定导航地图中的点数为 128 vs 0。两次最小人机中心平面距离为 0.979 vs 0.965 m；ours 的终点平面误差为 0.171 m，略高于 badcase 的 0.132 m，因此不能说所有指标都提高。

这只是各一次运行，而且两次 PedSim 行人的实际位移并不完全相同；这些百分比是这组样本的描述，不是统计性能结论。静态树木附近的稳定地图点数约 999 vs 705，也提醒我们：不同航线带来的观测覆盖差异与潜在过度清理还没有分开，不能只凭鬼影为零认定地图质量整体更好。

可以直接打开 [这次的合并航线图](../../../../dynamic_demo_runs/badcase_vs_ours.png)，或分别查看 `dynamic_demo_runs/badcase_20260923_201211_trajectory.png` 与 `dynamic_demo_runs/ours_20260923_200322_trajectory.png`。合并图使用同一个 Gazebo XY 坐标系：橙色 badcase 在行人横穿后明显绕向右侧，绿色 ours 更接近直达路径。汇报时把图、两份报告和 Gazebo/RViz 同屏录像一起展示。

## 现场如何讲

按“无人机起飞 → FAST-LIO 形成位姿和扫描地图 → 行人横穿 → 风险上升/悬停让行 → 行人离开后地图与路线更新 → 继续前往目标”的顺序展示。在 RViz 中固定坐标系 `map`，现有配置已显示：蓝色当前扫描、绿色 `/uav1/fastlio/cloud_map`、黄色 `/uav1/local_static_map`、浅青色局部自由空间、红色确认动态点、紫色全局 A* 路线、橙色局部路线、青色无人机实际轨迹。将 Gazebo 与 RViz 并排录屏，尤其拍下行人离开后的旧位置和路径变化。绿色地图显示项在两模式使用同一话题，但 badcase 已关闭清理，应按运行命令解释其含义。

可以如实说：“我们将 FAST-LIO2 作为激光惯性定位前端；在其输出之上维护可更新的导航地图。相比同一系统中关闭动态轨迹/自由空间清理和局部重规划的组合基线，完整系统利用后续可见性证据更新了行人离开后的旧占用，并重新规划了通往目标的路线。在这一次实测中，两组都到达目标；ours 的航线更短、用时更少，旧位置残留点也更少，但尚需重复实验和静态结构保留检查。”

随后只填本次两份 JSON 确实支持的数值：到达率/用时、Gazebo 实际飞行路程、目标误差、人机中心平面距离、旧位置稳定地图点数、风险持续时间和 FAST-LIO 对齐 ATE。若某项 `N/A` 或 ours 未胜出，就展示失败片段并说明原因，不能写“显著提高”“零碰撞”或“已解决所有鬼影”。人机中心距离只是两个模型中心的 XY 距离，不是机体表面净空；ATE 是定位质量监控指标，两次路径不同，不能把差异归因于地图清理。至少重复多次，记录失败与方差后再做论文性能声明。

现有已知边界：纯几何检测会受到 MID360 非重复采样、视角改变、树叶、遮挡和位姿误差影响；静止的人仍然是即时障碍；当前规划是固定高度带二维栅格 A*，不保证三维动力学轨迹可行性；历史点云在 FAST-LIO 内部地图里可能仍存在。此演示验证导航地图和控制链路，不展示回环精度。

## 2026-09-24：备份、持续聚类、键盘行人及最终地图

修改前完整备份在 `/home/a/AstraDroneOpen_backups/20260924_142901/`（含 `.git`、未提交/未跟踪文件、编译产物、所有已有 bag 和分析结果）。SHA256 与恢复步骤见该目录 `README.txt`。请解压到新的空目录，不要直接覆盖当前项目；外部 ROS/PX4/PedSim 不在归档内。

启动仍为：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/dynamic_comparison_demo.sh ours
```

演示不会自动结束。两名行人都在右侧终点区（距x=3.8,y=2.5小于0.95m）持续停留3秒后，脚本将其固定并解锁键盘。社会力在终点可能仍有轻微摆动，因此不要求速度严格归零。另开终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosrun fastlio_bridge pedestrian_control.py --keyboard
```

`0`/`1`选择行人，按住 `W/S` 沿世界 `+Y/-Y`，`A/D` 沿世界 `-X/+X` 移动，速度0.6m/s；空格停，`Q`退出键盘（不关闭仿真）。键盘终端必须获得焦点。命令断流0.4秒自动停车，换人或退出也停车，原先位置不会被PedSim抢回。初期横穿完成时两人都固定在终点，之后只有选中者响应命令，避免PedSim社会力让未选中者继续挪动。控制器不做行人碰撞路径规划，不要穿过树干/墙体；不支持瞬移到无人机旁。锁定状态可用 `rostopic echo /demo/pedestrian/ready` 查看。

无人机到终点后已距最初横穿处约9米，原行人超出6米检测范围是正常现象。可先沿X将行人带到树木内侧，再沿+Y走近无人机（树干约在x=3.9,y=10.6），然后横向往返、暂停再启动。动态球只代表确认运动目标，不应一直包住静止的人。红球是当前观测、橙球是短期预测；消失即DELETE，上游断流则0.5秒过期。位置平滑与运动确认历史现已分离，保留原确认门限，初次运动确认仍需要多帧。

诊断节点是否在工作，别只看有没有球：

```bash
rostopic hz /uav1/foreground_points
rostopic hz /uav1/dynamic_objects
rostopic hz /uav1/dynamic_markers
```

ours 在报告生成时保存 `dynamic_demo_runs/ours_时间_final_map.pcd` 和同名 `.json`，正常按主终端 Ctrl-C **关闭各节点之前再保存一次**，包括后来手动移动行人之后的更新。不想结束演示，随时执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosservice call /demo/save_map
```

只在返回 `success: True` 时视为保存成功。JSON记录来源 `/uav1/fastlio/cloud_map`、`map`坐标系、时间、点数、SHA256；PCD为二进制XYZ，不含RViz配色/Marker/路径，也不是FAST-LIO未清理的内部地图。地图或注册扫描断流超过3秒时拒绝覆盖有效文件。强制kill/断电不保证最后保存。报告结束后bag已停止，后续键盘活动只进入实时地图和退出快照，不自动加入原报告。

**清图边界：**导出逐点保持维护地图的有限XYZ，不偷偷按真值删除人或进行假清理。人离开后需要新的自由空间观测才能清掉旧位置；遥控后请留几秒观察地图，必要时让无人机回访。遮挡/出量程后未再观测的区域不能保证零鬼影；当前静止的人仍应作为真实障碍存在。若RViz还有残影，PCD也会忠实保存，不能把“文件保存成功”当成“全场景零残影认证”。

### 本轮验证证据

最终代码的无界面运行前缀为 `dynamic_demo_runs/ours_20260924_144552`：自动到达37.567s、实际XY路程12.410m、两人初始位置导航/稳定地图残留0点。到达后仍持续运行，再用手动控制接口执行靠近、横穿、停5秒、反向横穿及离开；两次横穿分别有50/42帧实测动态对象，非每帧检出，运动确认/遮挡仍会造成间歇。对象时间戳年龄P95约0.027s（仿真时钟，不是wall-clock运行耗时，也不是起动后首次确认延迟）。

`*_manual_test.json` 保存接口回归结果：命令断流后漂移约5.5e-7m，未选中行人漂移约1.8e-7m；`*_manual_test_map.pcd` 的190305个XYZ点与对应发布消息完全相同，校验和存于同名JSON。离开的暂停点(-1.8,9.5)、半径0.4m、z=0.4~1.8m范围残留0点。较宽横穿带仍有1点(2.126,9.130,1.772)，但横穿前地图已有距离0.013m的对应点，不能把这类背景支持点为了“全空”强删；尚无全图逐点真值来认证零鬼影。退出前另更新 `*_final_map.pcd`，所以最终文件点数可多于固定测试快照。

已通过fastlio_bridge编译、3个工具单测、Marker独立话题测试（轨迹ID、0.5s寿命、空数组DELETE）、真实终端连续键与W移动/断流停车、正常退出保存。仿真使用headless，本轮未完整目视检查Gazebo/RViz窗口，也未重跑badcase；旧对照数字属于旧版本，不能当作新参数下的两组比较。
