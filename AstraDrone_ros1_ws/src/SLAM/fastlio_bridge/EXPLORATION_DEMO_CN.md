# 仓库自主探索与动态地图演示

这是独立的新 demo；原 warehouse_dynamic_demo.sh 和 example badcase/ours 入口不变。
仅用于当前 Gazebo/PX4 SITL 仿真，不能未经验证直接用于真机。

## 当前版本：提速、结束标准与完整性审计（2026-09-29晚）

先编译一次新增的无额外依赖C++搜索库：

```bash
cd /home/a/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg fastlio_bridge -j2 -l2
cd /home/a/AstraDroneOpen
# 推荐：保留RViz观察，关闭Gazebo GUI；不是关闭物理仿真
bash scripts/run_sh/warehouse_exploration_demo.sh --no-gazebo-gui
# 速度/稳定性实验：两种GUI都不启动，其他链路照常
# bash scripts/run_sh/warehouse_exploration_demo.sh --headless
```

只选一个主命令，不叠加第二个FAST-LIO或飞控。行人默认自动移动一轮；键盘终端
需等 `/demo/pedestrian/walk_status` 为 `finished_keyboard_ready` 再接管：

```bash
cd /home/a/AstraDroneOpen
source /opt/ros/noetic/setup.bash
source /home/a/pedsim_ws/devel/setup.bash
source AstraDrone_ros1_ws/devel/setup.bash
rosrun fastlio_bridge pedestrian_control.py --keyboard
```

0选择唯一行人，W/S沿world Y，A/D沿world X，空格停车、Q退出；0.4秒断流停车。
若从开场即用键盘，在主命令同时加 `--manual-pedestrian`，不要与自动行走竞争。

默认提速不改physics步长、雷达10Hz/每帧20000点、清图证据和风险停车。
本轮曾在生成副本试验2ms/500Hz：虽然本机PX4插件接受这个lockstep配对，
仓库实测却未通过地面位姿稳定门控，未起飞（193754场次保留失败日志）。因此已撤回
该选项，保持1ms/1000Hz原物理设置，不绕过安全检查或只调real_time_factor假造提速。
机器实际运行快慢仍以RTF、飞行时间及计算耗时实测为准。
新增原生Dijkstra
保持原八邻域/对角禁止切角规则，未编译时回退Python并在status显示后端。访问抑制只
操作局部小块，候选按米级代表点检查可见未知增益，不再把隔墙未知当作近处收益。
栅格/Marker/状态磁盘快照0.5Hz，航点和在线状态仍2Hz。探索原限速0.55m/s、前视1.2m、
驻留2秒，正常0.80m膨胀/0.40m机体/静动态风险不缩小。提高限速有跟踪/时延代价，
这里只用于SITL；不能按微测速度推断整机或真机提速。
可复测单独搜索耗时（建议关仿真后，同网格比较Python/C++，不代表飞行总时间）：

```bash
/usr/bin/python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/benchmark_exploration_search.py --runs 5
```

安全格边缘加1.5无量纲软代价，优先通道中部，减少贴架飞行触发慢速退让；硬安全格、
障碍膨胀和对角防切角不变，安全窄道不会因软代价被封死。加权搜索的distance是
等效长度代价，不是实际航程（航程仍由里程计累积）。退让前视从0.10m到0.20m，
比例控制约0.09m/s；整段真实点净空不低于起点且≥0.70m、机体处在已知区等检查保留。

探索栅格保持0.20m、200000格，仍计入原0.80m净空及格子半对角线。本轮试验
0.10m时，现有0.15m自由体素中心采样在细网格中留下未知小孔，真实仓库起飞后
HOLD_START_NOT_SAFE、未选择目标，已撤回（194424场次保留失败/部分结果）。
几何单测表明细网格可以减轻量化损失，但不能单独改分辨率：必须同时提供一致的
射线穿越/体素空间支持，再验证飞行，不用膨胀“自由点”冒充已观测未知空间。

结束分三层，不混用：

| 层级 | 标准和含义 |
|---|---|
| 在线探索进展 | 90仿真秒新增自由面积<1m²且新0.4m三维表面格<25；至少3个到达观察目标后低增益转补扫，不直接宣称完成 |
| 已测表面观测质量 | 当前维护图中，近6m范围至少3个不同采集时刻、至少2个方位扇区且观察基线≥0.75m的粗表面格比例；分母不含从未测到的表面 |
| 在线可达区域收敛 | 无剩余有用前沿/补扫候选、上述窗口低增益、已测表面质量≥95%、无失败目标记忆/容量不足，才 `exploration_complete=true`，原因 `observed_reachable_converged` |
| 任务收尾 | 返航到起点≤0.30m连续5仿真秒，保存成功才 `COMPLETE`，默认退出；并不自动变成三维完整 |

前沿连续30秒用尽转补扫；补扫用新实际扫描的三维表面证据生成另一角度观察位置，
当前维护图已清除的人体不会成为永久补扫目标。
补扫到达不等于取得足够证据：不合格表面的视点访问记忆20仿真秒后可重试，
用600仿真秒补扫预算限制重试，不用永久访问屏蔽悄悄跳过仍未扫描好的表面。
若补扫无剩余候选却发现新的有效前沿（如行人离开后打开通道），返回探索并重置收敛窗口，
不因曾进入补扫就放弃新区域。
无可达补扫位置但质量不足、目标失败，或600仿真秒补扫预算耗尽，都保存为部分结果，
不能美化成完整。3600仿真秒或3600墙钟秒
探索预算也只触发部分返航（探索器收到有效数据时执行）。运行器另有独立4800墙钟秒
任务硬保护，包括返航；即使/clock停止或数据一直不新鲜也只保存部分图退出，标记
`wall_guard_timeout_partial`、failed=true，不伪装返航成功。可用`--max-wall-seconds N`
调整（有限且≥10秒）。`--keep-open`仅在成功返航保存后允许持续展示。
断流/暂停/起点不安全会
重置进展窗口，不把停住造成的零增长认定为收敛。容量不足保持旧栅格并尝试安全返航，
不把扩窗失败当作空闲空间；返航1200仿真秒超时失败。`--keep-open`仍保留展示。

在线查看（已source终端）：

```bash
rostopic echo /uav1/exploration/status
# 手动结束探索，但仍规划安全返航，不直接杀节点
rosservice call /uav1/exploration/return_home "{}"
```

看 `useful_frontier_candidates`、`surface_quality`、`progress`、`compute_p95_ms`、
`estimated_rtf`、`exploration_complete`、`finish_reason`；`coverage_complete`仍false，
因为当前固定高度二维规划不能证明屋顶、底面、遮挡内部、不可达狭缝及未知外部全部完成。
0.5Hz磁盘快照可能滞后2秒，最终_result.json以在线最终任务状态为准。

退出后新增独立离线审计：`_alignment.json`、`_coverage.json`、`_coverage_missing.pcd`。
场景参考碰撞盒和初始world/map对齐只用于这个评估器，不传给无人机决策。对齐使用
ModelStates无时间戳消息与初期odom≤50ms墙钟接收配对，长期漂移没有事后校正；必须
结合SLAM误差看指标。缺少有效对齐时不生成虚假覆盖率，审计失败也不删除地图。

货架侧面验收建议：参考暴露碰撞盒侧面在world高度0.25–1.9m范围每0.30m采样，地图
在0.25m**欧氏**距离内支持的总比例≥95%，且每个货架≥90%。不可达/遮挡侧面仍在
分母并体现在缺口文件里，不能将它们悄悄排除来提高指标。它是**货架侧面**代理标准，
不是完整视觉mesh、所有内部货物/上下表面或全场景3D覆盖。0.25m容差不是SLAM精度。

若需发表或真正要求全三维完整，必须先声明目标静态表面、传感器可观测性和安全可达
空间，用准确mesh/传感器射线参考评估，扩展经过三维碰撞/视场检查的多高度/多角度
视点规划，并单列不可达面。当前低顶棚场景不能盲目升高；没有这种实现和实测前不保证
100%完整。动态残影指标还需成对检查旧人体ROI与静态保留；覆盖指标高并不能证明零鬼影。

查看本次真实地图：`pcl_viewer <本次前缀>_final_map.pcd`。
查看缺口参考：`pcl_viewer <本次前缀>_coverage_missing.pcd`，它不是实际建图点云。
手动重算（具体前缀替换为自己的运行场次）：

```bash
/usr/bin/python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/warehouse_map_audit.py \
  --scene exploration_demo_runs/warehouse_YYYYMMDD_HHMMSS_scene.json \
  --pcd exploration_demo_runs/warehouse_YYYYMMDD_HHMMSS_final_map.pcd \
  --alignment exploration_demo_runs/warehouse_YYYYMMDD_HHMMSS_alignment.json \
  --output exploration_demo_runs/warehouse_YYYYMMDD_HHMMSS_coverage.json
```

设计参考：[FUEL作者项目](https://github.com/HKUST-Aerial-Robotics/FUEL)的前沿与
视点规划方向；本实现不是FUEL复现。仿真时间/墙钟关系见
[Gazebo官方物理参数文档](https://classic.gazebosim.org/tutorials?tut=physics_params)。

### 本轮实际验证与限制（2026-09-29 20:01）

64项Python回归、catkin C++构建、Python/shell语法、包结构检查通过。最终C++/Python
同输入50×30m搜索各5次：无软代价中位数178.80/2.74ms，有1.5软代价210.83/2.99ms；
距离结果一致、可达35916格。这是单独搜索微测，不是整个仿真65/70倍提速。

真实仓库195446（headless、显式单目标短测）自主到达、返航驻留、保存成功：
102.20墙钟秒、11.59m航程、home误差0.0376m、48738有限XYZ点，RTF约0.505，
周期计算P95约76.5ms。`explicit_goal_limit_partial`、exploration_complete=false；
独立参考货架侧面覆盖10.36%，不是完整仓库。192710单目标、193317三目标后操作员
返航也成功，但都不是自然结束/全图验收。194829持续慢退让的负结果保留，不能据
195446不同初始状态的单场成功宣称贴边软代价消除了所有死锁或统计提速。

最终room8独立ROS合成闭房间：7个视点到达、已测表面74/76=97.37%、低增益收敛，
返回稳定后保存确认一次，exploration_complete=true、coverage_complete=false。
该夹具用了加速时间窗口与运动学跟随器，保存仅服务确认，不是真实PCD/PX4或全仓库。
room5/6曾达到约92%/91%质量后部分返航，保留为负结果；修正视点扇区边界量化和
补扫冷却后room7/8通过收敛标准，未降低95%阈值。

截至2026-09-30尚无全仓库长程自然收敛/95%参考侧面覆盖实验；历史长程RETURN_BLOCKED及
原生Gazebo卸载segfault仍未解决，195446退出仍重现后者。不能声明无bug、所有静态
面完整或全图零鬼影。地图清理仍依赖重新观测，固定高度二维规划仍有限制。
下方保留旧版本测试及历史限制，不把旧数值当成本次新参数的完整覆盖结果。

### 仓库贴墙风险修正（2026-09-30）

用户报告探索演示撞墙；旧运行日志中曾出现离障碍点约0.46m（机体半径配置0.40m），
同时控制器在静态紧急风险level 2仍执行最近障碍的斥力速度。这个方向未经相邻货架
另一侧的扫掠碰撞检查，不能当作安全逃生路径。现在**仅探索演示**将level 2改为
水平速度零指令，要求地图坐标系的新鲜注册点云才能导航，并把水平限速从0.55降到
0.30m/s。旧导航对比演示的控制参数不跟着改变。速度零指令并非瞬时物理刹车，
真实避碰仍受控制/传感器延迟、机体惯性、雷达遮挡和模型误差约束。

如果状态持续`HOLD_START_NOT_SAFE`或`RECOVER_CLEARANCE`，说明当前视点无法在
0.80m规划净空下安全连通。不要通过缩小机体半径、关闭风险检测或伪造自由空间
强行通过；先检查本轮`_exploration.log`的风险/速度和`_coverage.json`缺口。
限时测试或这种安全保持所保存的`_final_map.pcd`只是**部分地图**。
固定高度二维探索没有证明所有货架侧面、内部及遮挡后的表面可观测且可达；
实现完整三维地图需要额外的多高度视点与三维轨迹/碰撞验证，当前不能声称已实现。

### 首货架停滞修复及完整任务验收（2026-10-03）

复现失败轮`warehouse_20261003_215945`在首个货架附近仅到达0个目标，探索器
`HOLD_START_NOT_SAFE`，旧一级静态风险退让与探索器的安全退路不一致，二级风险又
完全停车。修复后普通规划仍要求0.80m机心净空；只有已在膨胀区内、机心距点云
至少0.45m（0.40m旋翼包络+0.05m）且沿已观测区域向0.80m安全格**单调增大净空**时，
探索器才发不超过0.20m的短退路。控制器只在退路与当前目标一致、动态风险为0、
扫描/风险新鲜并经扫描二次检查时才允许风险级1/2下最多0.08m/s的静态退让。
旧最近点斥力退让仅在探索launch关闭；其他演示默认控制未改。正常行进还增加
当前扫描0.80m前向刹车。无法恢复且30仿真秒内位移不足0.20m时明确报
`STALLED_UNSAFE`、保存部分地图，不再无限慢移/悬停。它仍不是三维碰撞证明。

完整无`--test-seconds`测试命令：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --headless --freedom-baseline
```

`warehouse_20261003_222021`一轮SITL自然返回起点并保存、runner退出码0；
93/97目标到达、547.88m水平航程、起点误差0.12m、维护地图284418点，
FreeDOM独立地图756031点。中间一次`RECOVER_CLEARANCE`后继续探索，未出现首货架
永久停滞；实际风险级1/2下的短退让本轮未触发，只由假飞控隔离回归验证。
本轮`finish_reason=wall_budget_partial`、`exploration_complete=false`：
维护地图货架侧面参考支持9267/10192=90.92%，最差单架29.59%；
配置验收为总体95%且各架90%，因此**全仓库建图未通过**。
FreeDOM同参考支持10091/10192=99.01%，最差单架85.71%，同样未过各架90%；
两种地图密度与维护机制不同，该数值不能证明动态过滤或导航性能优劣。
文件在`exploration_demo_runs/warehouse_20261003_222021_{result.json,coverage.json,final_map.pcd}`，
FreeDOM另有`_freedom_coverage.json`、`_freedom_static_map_point.pcd`。
本轮未采集Gazebo接触真值，不能声称零碰撞或以后绝不再停滞；失败时仍需看
`STALLED_UNSAFE`和风险/扫描日志，不能降低机体尺寸或把未知空间放通。

## 起飞修复验证（2026-09-29）

初始化门控与仿真时钟修正后，两次仓库冷启动均实际离地并自主飞行。
第二次短测到达1个观察目标，累计平面航程约6.77m，保存45900点维护地图；
因测试显式设置100墙钟秒限制，在返航过程中结束，不是成功返航或完整建图。
第一次飞行后持续安全退让，主动结束时也没有确认返回起点。这些导航问题没有
通过缩小安全距离掩盖。退出时仍重现已有Gazebo原生卸载崩溃。

## 历史探索验证边界（2026-09-28）

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

2026-09-29启动修复：启动器在初始化ROS节点前启用仿真时钟，避免系统时间与
MAVROS仿真时间混用。先等待PX4地面位姿连续4仿真秒稳定，再启动SLAM/初始对齐；
控制器在地面再检查3秒稳定窗口，解锁前用当前高度确定起飞基准。负的局部Z是
合法坐标，不能硬改成0。到达目标高度后，必须收到PX4在空中状态并稳定1秒，
`/mavros_avoidance_controller/takeoff_complete`才为true；探索器等待此新鲜心跳，
不再把初始化高度变化当作真实爬升。位姿断流、模式/解锁丢失或飞行中明显高度
跳变会撤销确认并保持；短暂位姿断流在新鲜数据恢复后可继续，不重新解锁。
高度跳变故障需重新初始化，不会在飞行故障后自动反复解锁。保持不是自动降落，
真机坐标复位补偿/故障接管尚未验证。慢速仿真等待的墙钟时间会更长，不要另开飞控。

可在另一个已source的终端检查：

```bash
rostopic echo /mavros_avoidance_controller/takeoff_complete
rostopic echo /uav1/exploration/status
```

起飞前状态`WAIT_TAKEOFF_CONFIRMATION`正常；之后应出现`OBSERVING`和`EXPLORING`。
一直等待时查看控制器/启动终端日志，而不是缩小安全距离或强制解锁。

## 黑箱边界与速度回归（2026-09-29）

探索器不读取货架碰撞盒；旧的固定[-4,-12,47,15]工作区先验已移除。现在只保留
有限的本地栅格内存窗口，车辆位姿/新鲜雷达自由空间靠近窗口边缘时扩窗，新区域仍为
未知并禁止通行。21项规划/状态回归覆盖扩窗坐标保持、任务记忆平移、未知区和容量保护。
探索巡航限速为0.45m/s，探索前视距离0.80m；净空0.80m、机体约0.40m与风险停车阈值未改。

headless PX4/Gazebo 一目标短测（`--return-after-goals 1 --test-seconds 150`）实际选点、
到达1个目标、返航并保存地图：仿真70.22秒/墙钟120.65秒，平面航程11.84m，返航XY误差
0.268m，维护地图46609点。它验证短程闭环，不代表覆盖完整，也不是完整探索结束；地图状态
仍为`coverage_complete=false`。配置dump确认限速0.45、总速度0.65、静态净空0.30/0.25m及
规划膨胀0.80m。实测仿真时间/墙钟约0.58，Gazebo低实时因子仍会拖长演示墙钟时间。
关闭仿真时仍见原生`Segmentation fault`，与本次规划变更无关且尚未修复。

本次短测数据：`exploration_demo_runs/warehouse_20260929_172101_result.json`、
`warehouse_20260929_172101_final_map.pcd`；此PCD只是该短程演示的有限维护图。

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
| initial_window_radius | 12m初始计算窗口半径；不是场景地图或仓库尺寸先验 |
| growth_margin / growth_chunk | UAV位姿或新鲜激光自由空间接近窗口边缘4m时扩窗；按8m块减少频繁重分配 |
| max_grid_cells | 平面栅格最多200000格；容量耗尽明确悬停，不把窗口边界误称探索完成 |
| resolution | 0.20m规划格；不改变0.15m维护点云体素 |
| clearance | 0.80m机心到障碍，已含约0.40m机体半径；另计栅格半对角 |
| unknown_margin | 0.40m机体到未知空间余量，不允许把未知默认作自由 |
| recovery_minimum_clearance | 仅用于已处在膨胀余量内且净空单调增大的短退让；默认0.45m（0.40m机体+0.05m），正常规划仍用0.80m；控制器再以新扫描核验 |
| unsafe_stall_seconds | 不安全机位连续30仿真秒无0.20m进度就报告STALLED_UNSAFE并保存部分地图 |
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
该实现不预载cangku货架/墙体几何：原world只用于Gazebo渲染与碰撞，`_scene.json`
仅给行人约束使用；探索器不订阅Gazebo真值。原先固定的[-4,-12,47,15]工作区矩形已
移除。现在栅格只是有限内存窗口，窗口边界按无人机位置与本轮新鲜雷达自由空间证据增长；
新增长格仍是未知，规划器不会穿越未知格。自主选择目标来自已观测可达自由空间附近的
frontier，而不是按仓库货架坐标预设巡航路径。
地图被清理后重新投影，不永久缓存人体占用；当前扫描优先，避免清图掩膜让真实人“隐身”。
暂停、新障碍、信息断流均应保持或触发原安全响应，不以缩小安全半径来强行推进。
当新观测或跟踪误差让当前位置不再满足规划膨胀时，先检查能否朝正常安全格退让：
整段都必须在已观测空间中，点云净空不低于起点且不低于0.70m，单次仅给0.20m前视点
（2026-09-29更正：原为0.10m），不降低普通规划净空。
这不删除起点障碍、不放宽正常规划膨胀；条件不满足时继续保持并服从原风险控制。
`stalled_goals`还包含因新占用而不可达的候选，并不全是持续飞行卡死；需结合状态/轨迹判断。

自主探索demo将巡航限速从仓库通用窄通道配置的0.25m/s提高到控制器允许的0.45m/s，
并将经过安全线段检查的探索前视点从0.60m调到0.80m，以免比例控制器的kp=0.45
把典型0.60m航点速度压到约0.27m/s；
静/动态风险停止、净空和全局总速度限制仍生效，因此0.45是上限而非全程实测速度。
Gazebo实时因子、PX4加减速和近障减速仍会影响墙钟耗时。

相关思想参考作者 [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) 的frontier探索。
本实现是独立的轻量固定高度平面探索，不是FUEL复现，未使用其FIS、巡回优化或最小时间轨迹。
因此是“场景几何未知、观测驱动的二维自主探索”，不是包含天花板/地面高度变化、楼层与垂直航路选择的任意三维黑箱探索。
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
