# cangku 仓库：单行人键盘、SLAM与避障

此入口复用本项目 `simulation/astra_gazebo_worlds/cangku.world`，不改变原文件或原example演示；没有启用回环。仓库有52个货架及围墙。本轮先在左侧较宽通道验证，不代表已经穿过每条狭窄货架通道。

## 启动及键盘

2026-09-27参数更新：仓库入口默认加载 `config/warehouse_navigation.yaml`，
下面旧实验的“安全距离未改”指更新前版本。原example仍使用基础参数。
参数/算法验证范围见下方“9月27日停滞专项修复”，旧实验不能替代新版本验证。

## 安全距离与规则怎么调整（2026-09-27）

只修改 `config/warehouse_navigation.yaml` 中对应节点的参数，保存后重启demo；
节点在构造时读取参数，运行中只执行 `rosparam set` 不会自动刷新算法。
该文件最后加载，仓库启用时会覆盖基础 `path_planner.yaml`、`static_risk.yaml`
和 `astra_mavros_controller.yaml` 中的同名项。动态行人风险配置不覆盖。

| 作用/参数 | 原配置 | 仓库配置 |
|---|---:|---:|
| 规划机心到表面膨胀 `inflation_radius` | 0.95m | 0.80m |
| 栅格边长 `resolution` | 0.25m | 0.10m |
| 静态预警机心距离 `uav_radius + safe_clearance` | 0.90m | 0.70m |
| 静态紧急机心距离 `uav_radius + emergency_clearance` | 0.65m | 0.65m |
| 预警解除机心距离 `uav_radius + exit_clearance` | 0.95m | 0.75m |
| 巡航速度上限 `max_navigation_speed` | 0.35m/s | 0.25m/s |
| 前视距离 `lookahead_distance` | 1.00m | 0.60m |
| 规划相对高度带 | [-0.80,+0.80]m | [-0.45,+0.55]m |
| 搜索外扩 `planning_margin` → `max_planning_margin` | 固定4m | 失败时4→8→12m |

注意距离定义：规划膨胀已经包含机体，不再加一次半径；风险的clearance需要加
uav_radius才是机心到表面距离。仓库取uav_radius=0.40m，不是缩小物理机体：
本地iris模型旋翼碰撞圆的水平外包络约sqrt(0.13²+0.22²)+0.128=0.384m。
模型几何不等于含姿态/定位误差的实机安全证明。

调参须维持“规划距离 > 预警解除距离 > 预警距离 > 紧急距离”，并留跟踪误差余量。
栅格膨胀还含半对角线，0.10m栅格约增加0.071m，实际通行受格子对齐、点云厚度
和高度带影响，不能声称1.60m通道一定能穿。货架盒近似测量存在0.56m至2m以上
的不同间隙，最窄缝不适合无人机穿行。优先选较宽通道再逐步验证，切勿设半径0。

规则：level0跟随A*；动态level1仍停车让行；仅静态level1在新鲜风险/扫描和
0.65m退让线段净空检查通过时以0.10m/s退让，否则停。level2抑制目标吸引，
优先执行融合避让。`stop_on_level1: false`会改变行人让行，不能当作“允许窄通道”的总开关。
该反应式规则不能保证遮挡、对向行人或复杂窄巷中总能解困；不是三维轨迹优化。

使用新配置：`bash scripts/run_sh/warehouse_dynamic_demo.sh`。
回到原距离：`bash scripts/run_sh/warehouse_dynamic_demo.sh --navigation-profile standard`。
单独launch时显式传 `warehouse_narrow:=true`；仅适用于已启动的正确仿真链路。
运行时可核对 `rosparam get /static_path_planner/inflation_radius` 和
`rosparam get /static_collision_risk`，预警距离观察
`rostopic echo /uav1/nearest_static_surface_distance`。

验证：独立ROS master启动真实规划/风险节点，不启动无人机；1.90m合成双墙通道
原参数发布单点保持，新参数产生到5m目标的路径。0.85m表面距离原level1、新level0；
新参数0.68m为level1、增至0.72m仍保持预警、0.78m恢复level0、0.60m为level2。
1.80m测试在本次栅格对齐下仍封闭，保留为边界负结果，不继续压低紧急距离。
复现：source工作区后运行 `python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_warehouse_clearance.py`。
这些不是Gazebo闭环穿行/碰撞测试，也不完成此前N018全部控制器回归。
此前“录包收尾误判导致退出”已另行修正：直接等待原生record完成索引再评测，
评测失败只报错而不关闭导航；清理阶段重复Ctrl-C也不再打断wait。
这不等于Gazebo原生退出段错误已根除，见末尾已知问题。

### 9月27日停滞专项修复

131318日志中的risk=0停车不是安全距离过大。连续RViz二维选点采纳实时高度，
令目标Z从1.60升到1.83m；±0.8m投影切片最终把2.42～2.45m顶棚当成平面障碍。
现二维选点保留此前目标Z，正数Z的三维目标仍显式改变高度；仓库切片单独缩至
[-0.45,+0.55]m。只是规划选点范围变化，点云地图里的屋顶没有删除。

132630实飞又发现固定4m搜索范围会截掉货架尽头的绕行路线。新增
`max_planning_margin: 12.0`，A*无路时逐级扩大搜索；保持0.80m膨胀及风险保护，
同一目标不再缩回已扩大的范围，仍受`max_grid_cells`上限约束。公共规划器栅格
原点固定在map网格上，避免无人机移动就改变整幅栅格对齐。

仓库另设置 `route_switch_improvement: 0.25`：旧路线仍安全时，新路线至少明显
缩短约25%才替换，防止两个货架端部路线交替占优造成往返。旧路线每轮重新检查，
当前点云占用使它失效时立即采用新路线；真实新目标清除旧路线。偏离路线时先连接
最近投影点，再沿旧路线前进，不直接跨拐角。参数0关闭滞回；这是稳定性与小幅
路径优化之间的取舍，不是永久锁定路线，也不保证全局最短。

局部自由证据的公共规划实现也已更正：原来在二维膨胀图上扩大擦除范围，会擦掉
附近/其他高度的真实障碍，特别容易在6m滚动局部地图边缘形成不稳定的假捷径。
现在先按三维距离匹配自由点（`local_free_clear_radius: 0.15`），剔除匹配的旧占用
点，再统一生成膨胀障碍；当前局部占用最后写入并始终优先。没有关闭自由空间
更新，也没有修改FAST-LIO原始地图；变化在规划栅格融合顺序。

控制器区分 `waypoint_tolerance: 0.03`（中间拐点）与 `goal_tolerance: 0.25`
（最终悬停）。不能把中间拐点也当作终点提前25cm停车，否则规划器还未能安全
暴露下一段路线、控制器却已HOLD。该参数在 `config/astra_mavros_controller.yaml`，
修改后重启完整demo生效；不要在真实飞行中随意重启控制器。

这不是保证任意目标可达：目标在货架实体内、
实际间隙不足、路径超出最大搜索范围或感知缺失时仍应停止，而不是强行穿越。

故障快照回归（不控制无人机，自动建立独立ROS master）：

```bash
source /opt/ros/noetic/setup.bash
source AstraDrone_ros1_ws/devel/setup.bash
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_planner_snapshot.py \
  dynamic_demo_runs/ours_cangku_20260927_132630_blocked.bag
```

输出键4/8/12是固定搜索外扩，0是自适应4→8→12；原4m单点保持，其他产生绕行。
`check_warehouse_clearance.py`还验证：相同水平参数下旧高度带封路、新高度带可行；
轻微位移不应反向切换仍有效的绕行路线；新增障碍使旧路线失效时必须切换。
完整多目标测试**会控制无人机，只用于该Gazebo仿真**；不要与键盘/手动目标同时使用：

```bash
# 先启动warehouse demo，然后在另一已source的终端运行
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_warehouse_multigoal.py \
  --output dynamic_demo_runs/my_multigoal.json
```

它记录初始目标→map(5.60,10.67)→(14.00,-0.15)→初始目标的逐帧里程计/路径/风险，
每段最多360墙钟秒；这个测试时限不会关闭demo。有限bag结束后的新目标需查该JSON，
不能用旧bag报告证明后来任务完成。`min_surface_distance_m`是点云测量，不是接触真值。

### 常规启动

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_dynamic_demo.sh
```

无人机从Gazebo世界坐标(-6,-4)起飞，起飞增高参数为1.2m，沿左通道前往约(-6,6)，到达后持续悬停；默认不会自行关闭。高度目标按控制器初始化的MAVROS本地高度加1.2m计算，不能把它当作精确的Gazebo世界绝对高度；初始化原点/估计偏差会改变实际高度。只有一名行人，模型名`0`，起点(-8,3)。本场景不运行PedSim自动横穿，导航就绪并出现“仓库仅0号行人”提示后即可控制。

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosrun fastlio_bridge pedestrian_control.py --keyboard
```

选择`0`（仓库没有1号）；按住`W/S`沿世界+Y/-Y，`A/D`沿-X/+X，空格停，`Q`退出键盘而不关仿真。默认0.6m/s，0.4秒无命令停车。行人使用现有人体网格做平移/转向，不是关节步行动画。行人有墙/货架/场景边界约束，避免键盘把人直接送进实体，但**不限制他向无人机走去**，因此仍可主动制造危险工况；无人机避障不是任意高速追撞的安全保证。

建议先保持行人不动观察SLAM；待无人机接近或到达，再按D从(-8,3)向(-4.2,3)横穿，停几秒，再按A返回。也可用W沿左通道靠近无人机。不要用过大速度或直接对着机体追撞来代替可重复横穿评测。`rostopic echo /demo/pedestrian/blocked`为true表示人工行人被场景碰撞/边界限制，不是无人机规划器故障。

原演示仍可启动：`bash scripts/run_sh/dynamic_comparison_demo.sh ours`。两套仿真不能并行启动。

## 实际加载的场景与修改位置

每次从原cangku生成 `dynamic_demo_runs/ours_cangku_时间_cangku.world` 及 `_scene.json`。副本移除编辑器保存的1477秒`<state>`（其中部分货架/建筑位置与模型定义不同），使用顶层模型定义从t=0启动；相对DAE路径解析为绝对file URI，避免换目录后找不到网格。原始模型定义中的顶板下沿约2.5m；未移动/删除货架给无人机让路。初测1.5m高度出现过顶棚进入规划高度带、无法到达的失败，因此仓库单独改为1.2m；9月27日又修正高度带与重复目标高度，当前参数以上表为准。example起飞增高仍为1.5m。这是适应低顶棚的场景配置，不是三维规划算法改进。

`scripts/warehouse_scene.py`：仓库起点、目标、行人初始位置及场景副本生成。
`scripts/warehouse_pedestrian.py`：单个行人的速度积分、断流停车、静态碰撞盒约束。碰撞盒只供人工行人移动约束和评测绘图，不作为无人机的先验地图。
`scripts/run_dynamic_comparison.py --scene cangku`：启动、录包、报告、持续展示和退出保存。仓库当前使用上表专用导航参数；下方9月24日/9月27日12:15旧实验使用当时未缩小的安全距离，不能混作同参数对照。

注意：Gazebo world不是导航map原点。此处map近似以无人机出生点为原点，因此世界(-6,6)约为map(0,10)，会有初始化小偏差。键盘方向采用世界轴；RViz的2D Nav Goal采用map坐标。残影评测已改用首个关联机体位姿转换坐标，不能直接拿world(-8,3)去索引map点云。

## 输出与复测

`dynamic_demo_runs/ours_cangku_时间*`保存bag、metrics/report、轨迹图、场景快照、场景来源SHA256和最终PCD/JSON。报告和bag在到达后结束，但Gazebo和算法继续运行；后续键盘活动不进入这份旧bag。最终地图会在正常Ctrl-C结束前更新。随时更新本次运行的`_final_map.pcd`可调用（会覆盖同名快照，需要保留多个时刻时先复制到另一个文件名）：

```bash
rosservice call /demo/save_map
```

地图源仍是RViz的`/uav1/fastlio/cloud_map`，不是未经清理的FAST-LIO内部地图。导出一致性、局部区域零残余与全图零鬼影是不同结论；遮挡区域没有再观测时不能保证清除。

只运行后端仿真（无新GUI）：`bash scripts/run_sh/warehouse_dynamic_demo.sh --headless`。
可选择`--auto-stop`用于自动测试；默认保留持续运行。

自动控制同一键盘话题的回归脚本（**会移动行人，仅供该Gazebo场景**）：

```bash
# 演示启动后，在另一已source工作区的终端运行；默认可在到达后测试。
python3 AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/test/check_warehouse_demo.py \
  --output dynamic_demo_runs/my_warehouse_test.json
# 若要飞行中横穿：在仿真刚启动时运行同一命令，追加 --during-flight。
# 脚本等无人机世界y到0.5再开始，不要同时使用键盘发送指令。
```

布局与录制轨迹：`python3 analysis/plot_warehouse_run.py dynamic_demo_runs/ours_cangku_时间`。图使用Gazebo真值作评估，没有把这些几何数据输入无人机算法。

## 实测结果与局限（2026-09-27补全）

### 14:10 停滞修复复测

数据前缀 `ours_cangku_20260927_133611`。开发中多次重新加载规划器，最后交接
控制器；因此这不是从起飞开始同版本连续四目标成功。最终代码版本的连续恢复
测试保存在 `_recovery_complete.json`，从约map(2.68,8.26)开始：

| 目标（map XY） | 到达 | 终点里程计误差 | 墙钟用时 | 最近点云表面 |
|---|---|---:|---:|---:|
| (14.00,-0.15) | 是 | 0.136m | 153.11s | 0.878m |
| 返回(-0.204,9.897) | 是 | 0.159m | 175.13s | 0.872m |

合计658个采样点未出现单点保持路径，风险均0，规划目标Z始终为1.245135m。
这是无主动移动行人的仓库恢复检查，不是动态横穿、真值接触净空或真机安全测试。
20秒发布采样的路径间隔P95约1.009墙钟秒，消息仿真时间戳年龄P95约0.018s；
不可把仿真时间戳年龄当作完整计算耗时。中途失败/主动中止文件全部保留，
`_recovery_final.json`是旧版超时负结果，不要因名字有final而误用它作成功证据。

编译、7项Python单测、隔离ROS节点回归、故障快照重放、配置/语法检查通过。
回归包含高度保持、真实顶棚图、搜索范围、路线稳定/新障碍改路、三维自由证据、
当前占用优先、拐点与最终悬停容差。两次重复Ctrl-C退出都未再产生启动器Python
KeyboardInterrupt，最终本轮测试进程已清理；原生Gazebo卸载段错误仍存在。
最终地图134644个有限XYZ点，frame=map，PCD头、数据长度、JSON点数与SHA256一致。
以上结果不保证所有狭窄缝隙或任意指定点可达；GUI本轮未重跑，采用headless SITL。

以下旧表的完整原始数据均保存在项目根目录`dynamic_demo_runs/`。旧表到达时间使用仿真时钟；ATE是与Gazebo真值做无尺度SE(3)最佳拟合后的3D RMSE。旧表飞行范围仅是左侧宽通道，不包含上面的新恢复航线。

| 运行前缀 | 起飞参数 | 测试 | 到达 | 用时 | 航程 | ATE |
|---|---:|---|---|---:|---:|---:|
| ours_cangku_20260924_155121 | 1.5m | 先静止行人导航，再追加移动 | 是 | 33.264s | 10.084m | 0.00697m |
| ours_cangku_20260924_155711 | 1.5m | 低顶棚阻塞失败样本 | 否 | — | 2.005m | 0.00586m |
| ours_cangku_20260924_160530 | 1.2m | 飞行中往返横穿、无GUI | 是 | 30.627s | 9.937m | 0.00755m |
| ours_cangku_20260927_121518 | 1.2m | 飞行中往返横穿、Gazebo/RViz | 是 | 31.824s | 9.887m | 0.00718m |

1.2m两次运行的首次/反向横穿实测确认分别54/51帧和40/39帧，均触发动态风险；这只是物体位置关联后的功能检查，没有逐点动态真值，不能当作分割召回率。最小人机中心XY距离约0.76/0.93m；中心距离不是机体与人体表面净空，未配置碰撞contact评估，因此不能据此宣称保证不碰撞。实时因子约0.53/0.51，约两倍墙钟时间才走完相同仿真时间。

160530的固定52853点地图与在线话题逐点一致，旧站立/暂停点半径0.4m、世界高度0.4~1.8m区域均0残留。121518的固定62560点地图同样一致，但旧站立区域仍1点、暂停区域0点；bag结束时更宽的旧位置统计为2点。不同ROI及采样时刻不能混用；少量点也未逐点核实为人体还是背景，不为追求零值强制裁剪。结论是已能明显清理并继续导航，尚不能保证全图零残影。

121518补测已目视确认Gazebo货架/行人与RViz点云/路径，Global Status为OK；真实键盘`0w`使行人世界y从6.0657变6.1857m，空格/Q正常停止退出。边界约束和命令断流停车通过；模型仍是站立人体平移，不带摆腿动画。截图为`_gazebo.png`/`_rviz.png`，布局和真值轨迹为`_warehouse_layout.png`。RViz同时显示顶棚、自由空间和扫描时较密，可临时取消Current LiDAR scan与Observed local free space，只保留Navigation map、UAV flight path及Confirmed dynamic objects观察。

备份位于`/home/a/AstraDroneOpen_backups/20260924_142901/AstraDroneOpen.tar`，已于9月27日覆盖更新。范围与SHA256见旁边README.txt；包括本轮收尾前的仓库实现及9月24日测试，未包含9月27日随后产生的复测和最终记录。没有改动原始cangku.world，也未提交或推送Git。

退出阶段已知问题：121518关闭时Gazebo出现段错误/崩溃报告。启动器已改成先关闭子节点、给roslaunch充分清理时间、最后关闭ROS master；122033针对性启停测试消除了本次日志中的XMLRPC连接错误，最终没有仿真进程残留，但Gazebo原生退出段错误仍出现，尚未定位到具体插件。地图在关闭节点前保存，PCD校验通过；不能把启动器返回0当作所有原生进程正常退出的证据。122033是主动提前停止的启停测试，不用于到达率或横穿效果统计。若上一次仍在生成崩溃报告，等其结束再重新启动，不要同时开第二套仿真。
