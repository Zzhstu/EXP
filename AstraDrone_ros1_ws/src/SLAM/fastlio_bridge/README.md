# Edge UAV Dynamic Perception V1.0

这是一个以 FAST-LIO2 为前端、面向 ROS1 Noetic + PX4/Prometheus 的室内无人机动态障碍感知与反应式避障完整基线包。它复用了项目现有节点，并补齐消息、构建系统、统一 launch、静态地图构建、目标导航、可选语义关联、系统诊断和离线校验。

## 1. V1.0 实现了什么

```text
/Odometry --------------------> fastlio_bridge ------> /uav1/fastlio/odom
/cloud_registered ------------> cloud_bridge --------> /uav1/fastlio/registered_scan
                                      |
                                      v
dynamic_detector -> foreground_points -> dynamic_cluster
                                           |       |
                                           |       +--> dynamic_points
                                           +----------> dynamic_objects
                                                          |
                          +-------------------------------+----------------+
                          v                                                v
                 dynamic collision risk                         static_map_builder
                          |                                                |
registered_scan -> static collision risk              +--------+--------+
                                                        |                 |
                                             rolling local map     mutable global map
                                             /uav1/local_static_map /uav1/fastlio/cloud_map
                                             /uav1/local_free_space (negative occupancy)
             |            |
             +--> avoidance_fusion --> fused risk/velocity
                                           |
RViz goal -> goal_navigation --------------+--> command_bridge
                                                 |-- preview (default)
                                                 +-- Prometheus command (opt-in)
```

主要功能：

- FAST-LIO2 与 `map`/PX4 坐标系桥接；
- 已运动补偿注册点云的统一转换；
- 初始静态背景体素建模和前景候选检测；
- 欧式聚类、轨迹关联、速度估计、动静确认和漏检保持；
- 动态目标 CPA/TTC 风险与静态表面净空风险；
- 静态/动态风险融合；
- 增量静态体素地图，并清理动态目标当前区域和反向历史轨迹；
- RViz目标到导航速度，风险状态下进行绕行/悬停仲裁；
- 语义检测与三维目标的可选投影关联；
- 话题频率和新鲜度诊断。

## 2. 为什么当前不再启动 dynamic_tracker

`dynamic_cluster.cpp` 已经包含预测门控、轨迹 ID、速度滤波、动态确认和漏检处理。再次串联旧 `dynamic_tracker.cpp` 会产生第二套 ID，并重复滤波速度。因此 V1.0 只有 `dynamic_cluster` 拥有轨迹状态；旧 tracker 不进入构建和 launch。

## 3. 依赖

核心感知：ROS Noetic、PCL、Eigen、`message_filters`、`tf2_ros`。

控制闭环可选依赖：工作区中的 `prometheus_msgs`。没有它时，感知和风险节点仍可编译，但 `goal_navigation_node` 与 `avoidance_command_bridge_node` 不构建。

语义节点使用包内的 `SemanticDetection2DArray`，避免 ROS Noetic 不同 `vision_msgs` 版本的类别 ID 类型差异。它不绑定某个 YOLO 仓库；缺少检测器时，几何—运动主链保持运行。

## 4. 放入现有工作区

把 `fastlio_bridge` 整个目录放到：

```text
~/fastlio2/src/fastlio_bridge
```

如果目标目录已有同名包，先备份用户自己的版本，再合并；不要覆盖未检查的参数或消息定义。

在 `~/fastlio2` 中执行：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

本交付环境没有安装 ROS，因此这里只执行了离线结构校验；最终 catkin 编译必须在你的 Ubuntu 20.04 + ROS Noetic 工作区完成。

## 5. 启动顺序

先启动 PX4 SITL/Gazebo、MAVROS、Prometheus 和 FAST-LIO2，再启动本包：

```bash
roslaunch fastlio_bridge complete_system.launch enable_control:=false
```

此时命令只发布到：

```text
/uav1/avoidance_command_preview
```

不会写入真实 Prometheus 控制话题。

使用 RViz 的 `3D Nav Goal` 或发布 `geometry_msgs/PoseStamped` 到 `/move_base_simple/goal` 设置目标。目标高度小于 0.1 m 时使用 `default_altitude=1.0 m`。

## 6. 必须检查的话题

```bash
rostopic hz /uav1/fastlio/odom
rostopic hz /uav1/fastlio/registered_scan
rostopic hz /uav1/fastlio/cloud_map
rostopic hz /uav1/foreground_points
rostopic hz /uav1/dynamic_objects
rostopic hz /uav1/fused_collision_risk_level
rostopic hz /uav1/fused_avoidance_velocity
rostopic echo /uav1/dynamic_system/diagnostics
```

确认预览命令的坐标方向、风险等级和高度门控都正确后，才进入受控实验场地的下一步验证。真实旋翼测试应遵守实验室安全规范和现场监督要求；项目默认不会自行开启真实控制。

## 7. 建图流程

1. 启动后保持场景静态约 30 帧，`dynamic_detector` 建立初始背景。
2. 每帧 `/uav1/fastlio/registered_scan` 更新滚动局部地图和全局体素存储。
3. `/uav1/fastlio/cloud_map` 以 2 Hz 发布可更新的全局地图；`/uav1/static_map` 是兼容别名。
4. 当前扫描射线提供负占据证据：局部规划立即覆盖旧障碍，连续空闲观测随后从全局图删除体素。
5. 动态目标被确认后，其完整近期轨迹区域会同时从局部图和全局图删除。
6. 需要重新建图时：

```bash
rosservice call /static_map_builder/clear
```

## 8. 语义接口

可启用语义关联：

```bash
roslaunch fastlio_bridge complete_system.launch \
  enable_semantic:=true enable_control:=false
```

需要提供：

- `/uav1/semantic/detections`：`fastlio_bridge/SemanticDetection2DArray`；
- `/camera/color/camera_info`：相机内参；
- `map -> camera_color_optical_frame` 的有效 TF。

节点把三维障碍中心投影到图像，关联检测框并输出 `/uav1/semantic_dynamic_objects`。该节点始终位于主链中；语义关闭、缺失或超时时会原样转发几何—运动结果。动态风险、静态动态掩膜、静态地图和预测器统一消费这个融合后话题。

## 9. 参数调试顺序

1. 先固定 `dynamic_detector.yaml`，验证无人机运动时墙面不会形成连续前景。
2. 再调 `dynamic_cluster.yaml`，主要看 `motion_speed_threshold`、`dynamic_confirm_frames` 和近场参数。
3. 最后调风险半径；不要用感知阈值掩盖坐标系或时间同步错误。
4. `uav_radius` 应按真实二维外包络测量，当前 0.50 m 是保守默认值，不是所有机型通用答案。

## 10. 离线校验

```bash
python3 src/fastlio_bridge/scripts/validate_project.py
```

它检查文件结构、XML、消息字段、CMake目标、风险阈值顺序和 `enable_control=false` 安全默认值。

编译完成后可运行不依赖 FAST-LIO2/PX4 的合成场景冒烟测试：

```bash
roslaunch fastlio_bridge synthetic_test.launch
rostopic echo /uav1/dynamic_objects
rostopic echo /uav1/fused_collision_risk_level
```

前两秒只有静态房间用于建立背景，之后一个三维障碍开始移动；正常情况下随后应出现非空动态目标、速度和风险输出。

## 11. 当前研究边界

V1.0 是一个可闭环运行的几何—运动动态避障基线，并提供了语义关联接口。它不是最终论文算法：下一研究迭代应将当前“初始背景差分”升级为自由空间/遮挡一致性动态检测，并在 Jetson 上接入经过室内数据微调和 TensorRT 优化的轻量语义模型。
