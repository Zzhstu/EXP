# 仓库自主探索与动态地图演示

这是独立的新 demo；原 warehouse_dynamic_demo.sh 和 example badcase/ours 入口不变。
仅用于当前 Gazebo/PX4 SITL 仿真，不能未经验证直接用于真机。

## 当前版本的验证边界（2026-09-28）

本次发布是当前工作快照，不是全仓库探索完成版。短程两目标仿真已经验证返航、
驻留与地图保存，但较长的八目标仓库测试最后仍为 `RETURN_BLOCKED`：当前安全
栅格中的返航连通性丢失，不能保证远距离探索后自动回到起点。系统会保守等待而
不是降低安全距离或穿越障碍；该问题尚未修复。最后保存的状态中起点误差约
18.72m，`home_confirmed=false`，不得当作成功返航。

静态保护和动态历史占用撤销的控制回归已通过，但货架完整三维覆盖、所有场景的
零鬼影均未证实。八目标旁路评价中，7815个已多次观测的货架参考体素有7770个
在维护地图的27邻域内得到支持（约99.42%）；这不是全仓库覆盖率。短程仿真还
出现过Gazebo卸载阶段原生崩溃，地图已保存不代表所有进程都无错误。

原导航/动态避障演示与新探索演示的入口分开。bag、运行日志、运行生成的PCD和
build/devel不随源码上传，地图仍保存在本地 `exploration_demo_runs/` 等运行目录。

## 启动与停止

先关闭旧仿真，在项目根目录执行：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_exploration_demo.sh
```

自动启动原 cangku 场景的运行时副本、一个行人、FAST-LIO、地图维护、风险控制、
自主探索和 RViz。自动起飞约 1.2m，相机/雷达与原仓库 demo 一致，不启用回环。
2026-09-27修订：默认按“探索 → 已知货架附近近距补扫 → 返航 → 起点范围内稳定5秒
→ 成功保存地图 → 关闭本次Gazebo/RViz”运行。不是起飞前恰好在起点就结束，也不是
探索途中偶然经过起点就结束。此改动只影响新探索demo，旧badcase/ours入口不变。
可达候选用尽才正常返航；3600仿真秒预算耗尽也会返航，但结果明确标为部分探索。
返航最多等待1200仿真秒，无法安全到达则保存部分结果并报告失败，不穿墙/缩小净空。
结束是停止SITL仿真进程，不是可直接用于真机的自动降落功能。
Ctrl-C 先保存地图再清理本次进程，重复 Ctrl-C 不打断清理。

```bash
# 无GUI；仍然运行真实Gazebo物理仿真
bash scripts/run_sh/warehouse_exploration_demo.sh --headless
# 行人完全由键盘控制，不安排自动行走
bash scripts/run_sh/warehouse_exploration_demo.sh --manual-pedestrian
# 返回后仍希望停留在RViz观察（显式选择，默认会自动结束）
bash scripts/run_sh/warehouse_exploration_demo.sh --keep-open
# 短流程回归：到达两个观测点就返航；不是全仓库扫描
bash scripts/run_sh/warehouse_exploration_demo.sh --headless --return-after-goals 2
# 仅用于无人值守测试，显式要求900墙钟秒后关闭，正常演示不要加
bash scripts/run_sh/warehouse_exploration_demo.sh --headless --test-seconds 900
```

环境需要原项目已编译的 ROS Noetic / PX4 / Gazebo Classic / person_standing 模型。
首次更新后编译：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C AstraDrone_ros1_ws -j2 -l2
```

## 控制与查看

另开终端，先执行 `source /opt/ros/noetic/setup.bash` 和
`source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash`。

```bash
# 暂停自主选点并悬停（风险保护仍优先）；true恢复
rosservice call /uav1/exploration/set_enabled "data: false"
rosservice call /uav1/exploration/set_enabled "data: true"
# 提前结束探索并安全返航；暂停时需先恢复，返航仍受障碍和数据新鲜度保护
rosservice call /uav1/exploration/return_home
# 状态、已知自由面积、到达视点数、航程、当前目标
rostopic echo /uav1/exploration/status
# 立即保存与RViz维护地图对应的PCD
rosservice call /demo/save_map
# 查看自动行人的阶段：finished_keyboard_ready后再开键盘
rostopic echo /demo/pedestrian/walk_status
# 原键盘工具还会导入PedSim消息，显式保留其Python路径（不改变旧脚本）
PYTHONPATH="/home/a/pedsim_ws/devel/lib/python3/dist-packages:$PYTHONPATH" \
  rosrun fastlio_bridge pedestrian_control.py --keyboard
```

新demo行人起点为world=(-8,-4)，位于起飞点旁；原demo的(-8,3)不变。
默认行人在无人机进入6m且保持8仿真秒后沿左侧通道走一轮，随后释放控制。
不要在自动行走阶段同时开键盘，避免两个速度发布者竞争；要从开始手动控制请加
`--manual-pedestrian`。行人是原站立网格的位移/转向，不是骨骼步行动画。
本 demo 的目标由探索器管理，不支持直接用 RViz 的 2D Nav Goal 接管；指定目标请用原导航 demo。

RViz：

- Maintained map：`/uav1/fastlio/cloud_map`，最终 PCD 的来源。
- UAV flown trajectory：实际里程计轨迹。
- Observed-free exploration route：当前探索路径（紫色）。
- Frontier viewpoints and state：候选视点和文字状态。
- Dynamic people：确认的动态物体标记；未检出标记不等于没有安全障碍。
- 可选显示探索栅格：未知/已观测自由/占用；它是平面规划视图，不是最终3D点云地图。

## 输出与评价

每次运行写入 `exploration_demo_runs/warehouse_时间_*`：

- `_final_map.pcd`：每60墙钟秒自动更新，Ctrl-C前再次保存；是最新快照，文件名不表示整仓库已经扫描完整。
- `_final_map.json`：来源话题、frame、stamp、点数及SHA256。
- `_exploration.json`：最新探索状态；不是全程历史。
- `_result.json`：结束原因、最终任务状态、是否返航成功；只有returned_home_and_saved
  表示自动返航驻留且地图已保存。coverage_complete=false表示没有证明全三维覆盖，
  而不是把返航成功当作完整建图。
- `_cangku.world` / `_scene.json`：运行时场景与行人碰撞约束。原world不改。
- 各节点日志。默认不无限录制bag，避免持续演示填满磁盘。

保存的是维护地图的有限 XYZ，不是原始FAST-LIO内部地图；没有导出时裁人或真值擦除。
若地图或注册扫描超过3墙钟秒不更新，保存器拒绝把旧数据称为新地图。
离开旧位置的人，需要雷达重新看到其背后，才有充分自由空间证据清除；被遮挡、未重新
观测的位置仍可能残留。当前静止人体仍是实际障碍，不应强行删除。

读数据的测试工具（不向导航发送真值）：

```bash
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_exploration_live.py \
  --seconds 600 --output /tmp/exploration_evidence.json
```

它记录已知平面面积、到达视点、航程、行人初始位置ROI前后点数、人机中心距离及点云
表面距离。应在起飞前启动才能得到非零“人原地不动”基线。ROI包含潜在背景，
中心距离/点云距离不是碰撞真值；不能把面积除以world面积就称为3D建图完整率。

## 方法与参数

`config/warehouse_exploration.yaml` 仅影响新 demo：

| 参数 | 含义 |
|---|---|
| bounds | map系任务边界；不是货架真值图，换出生点须核对 |
| resolution | 0.20m规划格；不改变0.15m维护点云体素 |
| clearance | 0.80m机心到障碍，已含约0.40m机体半径；另计栅格半对角 |
| unknown_margin | 0.40m机体到未知空间余量，不允许把未知默认作自由 |
| recovery_minimum_clearance | 仅用于已处在膨胀余量内时的短退让；默认0.70m，与原静态预警机心距离一致，正常规划仍用0.80m |
| height_below/above | 巡航高度下0.45m/上0.55m投影带 |
| progress_timeout | 25仿真秒没有0.25m位移，暂时屏蔽该目标换方向 |
| blacklist_seconds | 失败目标60仿真秒后允许重试 |
| no_frontier_seconds | 连续无候选30仿真秒进入补扫；补扫也无候选则返航 |
| inspection_radius | 2m近距访问范围，只标记能沿安全直线看到的已知空间，不跨货架标记另一侧 |
| inspection_surface_range | 实测障碍周围3m内筛选未近距访问过的安全补扫位置 |
| max_target_attempts | 同一目标最多2次失败；防止某些不可达目标无限重试 |
| home_tolerance / home_settle_seconds | 回到起点0.30m范围，连续驻留5仿真秒；离开范围重新计时 |
| max_mission_seconds / return_timeout | 3600/1200仿真秒；前者触发部分探索返航，后者报告返航失败 |

流程：近飞行高度自由射线累积 → 维护地图+当前扫描占用 → 机体膨胀/未知余量 →
可达自由空间搜索 → 边界视点按未知量和路径距离评分 → 保持目标、重检路径 →
发布短前视航点 → 原MAVROS控制器与静动态风险保护。
地图被清理后重新投影，不永久缓存人体占用；当前扫描优先，避免清图掩膜让真实人“隐身”。
暂停、新障碍、信息断流均应保持或触发原安全响应，不以缩小安全半径来强行推进。
当新观测或跟踪误差让当前位置不再满足规划膨胀时，先检查能否朝正常安全格退让：
整段都必须在已观测空间中，点云净空不低于起点且不低于0.70m，单次仅给0.10m前视点。
这不删除起点障碍、不放宽正常规划膨胀；条件不满足时继续保持并服从原风险控制。
`stalled_goals`还包含因新占用而不可达的候选，并不全是持续飞行卡死；需结合状态/轨迹判断。

相关思想参考作者 [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) 的frontier探索。
本实现是独立的轻量固定高度探索，不是FUEL复现，未使用其FIS、巡回优化或最小时间轨迹。
安全距离不允许进入的货架缝、遮挡区域和该飞行高度无法通行的区域可能无法完整扫描。
没有回环，长时间累计漂移仍存在；这是仿真工程扩展，不是完备探索或三维安全性的证明。

### 本次货架缺点修正

探索launch单独启用`/static_map_builder/evidence_only_clearing=true`。原动态目标轨迹圆柱
不再直接删图，也不阻止该范围实测点重新入图；动态检测和当前扫描的避障保护仍运行。
已有静止人体和新移动人体均可能短暂进入维护图，必须用重新观测到的空闲射线撤销，
不能用“识别人就删除附近一切”交换漂亮的地图。遮挡/未重访区域不能保证无残影。

- 射线来自真实返回点，不使用一个体素内多个点的均值虚构射线。
- 按扫描时间匹配里程计（最大差0.08s），使用FAST-LIO相同的LiDAR到IMU平移外参，
  经过姿态旋转得到原点。回调顺序不同会暂存最新一帧，缺少配对时不使用旧位姿强行清图。
- 当前帧回波及相邻体素优先保护；自由射线还必须靠近旧点0.06m以内才构成删除证据。
- 距离最后一次观测至少0.5s，普通点3次、稳定点6次有效自由观测；证据间隔超过1s
  重新计数。不是因为MID360这一帧没扫中就删除，也不是对静态点永久上锁。
- 新增近距补扫阶段避免单靠二维未知区域面积评价扫描充分程度。补扫仍不能穿越
  安全余量不允许的缝隙，也不对不可见表面插值填点，更不导入world网格伪造地图。

上述参数位于`launch/warehouse_exploration.launch`，任务参数在
`config/warehouse_exploration.yaml`。调小射线容差/加大稳定miss次数会更保守，但人体残影
消除可能更慢；调大容差可能重新损伤货架薄面。应成对验证静态保留和动态清理。

货架旁路评价（只读取数据；world仅用于测试ROI，不进入导航）：

```bash
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_rack_retention_live.py \
  --scene exploration_demo_runs/warehouse_时间_scene.json \
  --output exploration_demo_runs/warehouse_时间_rack_retention.json --seconds 1800
```

它累计真实注册扫描中至少3次采样命中的货架ROI体素，再检查维护图是否存在1格邻域支持。
15cm网格、27邻域容差、ROI盒体近似、初始world/map配准及早期估计误差都会影响数字；
这是“已观测点保留”，不是未扫描三维表面的完整率，更不能把噪点删除都视为货架破坏。
未被重新扫描的旧PCD缺口不会自动补出来，需要重新运行建图或重新观测对应货架。
Gazebo Classic原有退出卸载段错误在开发期间仍有重现（182446）；发生在关闭阶段，
不是已经修复的原生插件问题。启动器在卸载前保存PCD，旧日志保留；本轮不宣称无bug。

## 备份

开发前完整备份已覆盖到：
`/home/a/AstraDroneOpen_backups/20260924_142901/AstraDroneOpen.tar`。
SHA256：`51ecb1c31bda78dfe399e4b5852fb628fc17df008d3de4855725ab721fbbb782`。
包含开发前完整代码、未提交改动、build/devel和实验数据，不含本次探索新增代码。
恢复应解压到新的空目录核对，不能直接覆盖正在运行的项目。

本次修复前补充范围备份（包含新增探索代码）：
`/home/a/AstraDroneOpen_backups/exploration_fix_20260927_1954/bridge_scripts_records.tar.gz`。
覆盖bridge、run_sh、双记录及AGENTS，不替代上面的全项目备份；SHA256：
`fceef088ca69b7d9d2d9116c91ee37d408af382c352a9cdaeb84d8e41a135ba6`。

## 验证状态

编译与9项栅格、4项消息/断流单测通过；旧工具/清理/仓库场景7项通过。
首轮catkin中继模块导入失败已修正；182446后期膨胀起点死锁也已加入受约束退让，
失败日志和数据保留。最终183610使用修正版冷启动、开启Gazebo/RViz，约660墙钟秒测试：

- 自主选14个目标，已到达13个，第14个仍在执行时按显式测试时限收尾；不是全仓库任务完成。
- 里程计平面航程60.71m，已观测自由面积394.52m²，最终维护地图112095点。
- 人原静止位置ROI最多41点，离开后0点；不代表所有遮挡区域零残影。
- 654个采样中3段短退让后恢复推进，最长采样跨度约8.17墙钟秒；仅1个保持起点不安全样本，
  没有重现182446的持续死锁。不能据此保证任意场景不再卡住。
- 最近巡航点云水平距离约0.779m、人机中心XY约1.891m；没有测量真实接触或最小旋翼净空。
- 暂停漂移0.064m、恢复通过；同次42173点固定快照与维护话题逐点相同，最终PCD点数/长度/有限性/SHA均正确。
- 本轮正常清理、未发现Python异常或Gazebo卸载段错误；182446原生卸载异常仍是已知限制，不宣称根除。
- GUI仿真有效时段RTF约0.49，不是机载算法性能基准，没有做CPU/内存/延迟消融。

证据位于项目根目录 `exploration_demo_runs/warehouse_20260927_183610_*`，包括
`_evidence.json`、`_controls.json`、`_exploration.json`、`_final_map.pcd/.json`及节点日志。
最终地图SHA256为 `92d496170338f0bd44642f1f6171f5cb8efa68e38245eaf8208d8236b5e494c7`。
原world的SHA及原demo运行器、两个旧shell入口、地图维护/控制器源码与开发前备份比较一致。
整个仓库长时完整覆盖、不可达缝隙、静态表面保留率、动态MOS真值、长时定位漂移及真机适配仍待专项实验。
