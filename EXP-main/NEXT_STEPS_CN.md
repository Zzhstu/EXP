# 当前应该做什么

## 结论

现在不要继续增加新的算法名词，也不要先训练 Cylinder3D。当前任务是把 V1.0 在你的 ROS Noetic 环境中完成“编译—合成测试—rosbag测试—SITL闭环”四级验收。只有这四级全部通过，才进入真实室内数据微调和 TensorRT 语义部署。

## 第一级：编译验收

目的：确认消息定义、Prometheus接口、PCL和所有节点在你的真实工作区一致。

```bash
cd ~/fastlio2
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roscd fastlio_bridge
python3 scripts/validate_project.py
```

验收：`catkin_make` 无错误，且离线检查输出 `PASS`。

如果 CMake 提示没有 `prometheus_msgs`，说明当前终端没有 source Prometheus 的工作空间，或两个工作空间没有正确叠加；感知节点仍能构建，但控制桥不会构建。

## 第二级：合成数据验收

目的：不依赖 FAST-LIO2、Gazebo 和真实点云，先确认几何检测、聚类跟踪、风险融合、静态地图的话题链没有断点。

```bash
roslaunch fastlio_bridge synthetic_test.launch
```

另开终端：

```bash
rostopic hz /uav1/fastlio/cloud_map
rostopic echo /uav1/dynamic_objects
rostopic echo /uav1/fused_collision_risk_level
rostopic echo /uav1/dynamic_system/diagnostics
```

预期：前两秒建立背景；之后出现运动目标，`dynamic_objects.objects` 由空变为非空，并输出连续ID和速度。

## 第三级：现有 rosbag 验收

目的：解决当前最核心的真实问题——无人机自身运动时静态墙体被判成动态。

至少测试四段数据：

1. 无人机静止、环境静止；
2. 无人机运动、环境静止；
3. 无人机静止、目标运动；
4. 无人机和目标同时运动。

启动：

```bash
roslaunch fastlio_bridge perception.launch \
  publish_mavros_vision_pose:=false enable_semantic:=false
rosbag play YOUR_BAG.bag --clock
```

记录：

```bash
rosbag record -O v1_result.bag \
  /uav1/fastlio/odom \
  /uav1/fastlio/cloud_map \
  /uav1/foreground_points \
  /uav1/dynamic_points \
  /uav1/dynamic_objects \
  /uav1/static_map
```

验收重点：第二类数据中不能持续输出整面墙的动态点；第三、四类数据中移动目标应形成稳定ID。

## 第四级：PX4 SITL闭环

目的：验证目标导航、风险融合和Prometheus命令仲裁，不接真实旋翼。

```bash
roslaunch fastlio_bridge complete_system.launch \
  publish_mavros_vision_pose:=true \
  enable_semantic:=false \
  enable_control:=false
```

使用 RViz 设置目标，检查：

```bash
rostopic echo /uav1/navigation_command
rostopic echo /uav1/fused_avoidance_velocity
rostopic echo /uav1/avoidance_command_preview
```

验收：

- level 0：预览命令跟随导航；
- level 1：导航速度与避障速度融合并允许切向绕行；
- level 2：导航被覆盖，输出紧急避障/悬停策略；
- 数据超时：输出安全悬停预览，不继续使用旧速度。

## 各代码文件的目的

| 文件 | 目的 |
|---|---|
| `fastlio_bridge.cpp` | 把FAST-LIO2里程计对齐到`map`并可发布MAVROS视觉位姿 |
| `cloud_bridge.cpp` | 用同一固定变换把`/cloud_registered`转换到`map` |
| `dynamic_detector.cpp` | 建立初始静态背景，输出非背景前景候选点 |
| `dynamic_cluster.cpp` | 聚类、轨迹管理、速度估计、动态确认，唯一ID来源 |
| `semantic_fusion.cpp` | 将三维目标投影到图像检测框；语义失效时透明转发 |
| `dynamic_predictor.cpp` | 输出固定预测时刻的目标位置，供可视化和规划扩展 |
| `collision_risk.cpp` | 根据相对运动、CPA和TTC计算动态碰撞风险 |
| `static_collision_risk.cpp` | 计算机体到静态表面的净空，并屏蔽已确认动态物体 |
| `avoidance_fusion.cpp` | 融合静态和动态风险以及避障速度 |
| `static_map_builder.cpp` | 建立静态体素地图，删除动态物体当前位置和历史轨迹 |
| `goal_navigation.cpp` | 把RViz目标转换为限速导航命令 |
| `avoidance_command_bridge.cpp` | 仲裁导航/避障并生成Prometheus命令；默认仅预览 |
| `system_watchdog.cpp` | 检查关键话题是否缺失、超时或频率异常 |

## 什么时候加入轻量语义网络

满足以下条件后再接 TensorRT 检测器：

- 四类 rosbag 中的几何—运动主链稳定；
- 静态误报、动态漏检和ID跳变已经有量化结果；
- SITL中完成至少三种动态障碍路线测试；
- Jetson上记录了无语义版本的CPU、GPU、内存、延迟基线。

语义模型只负责向 `/uav1/semantic/detections` 发布包内二维检测消息，不能阻塞点云回调，也不能作为动态判定的唯一条件。
