#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <prometheus_msgs/UAVCommand.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>


class AvoidanceCommandBridge
{
public:
    AvoidanceCommandBridge()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "odom_topic", odom_topic_, "/uav1/fastlio/odom");
        pnh_.param<std::string>(
            "risk_topic",
            risk_topic_,
            "/uav1/fused_collision_risk_level");
        pnh_.param<std::string>(
            "velocity_topic",
            velocity_topic_,
            "/uav1/fused_avoidance_velocity");
        pnh_.param<std::string>(
            "preview_topic",
            preview_topic_,
            "/uav1/avoidance_command_preview");
        pnh_.param<std::string>(
            "command_topic",
            command_topic_,
            "/uav1/prometheus/command");
        pnh_.param<std::string>(
            "navigation_topic",
            navigation_topic_,
            "/uav1/navigation_command");
        pnh_.param<std::string>("world_frame", world_frame_, "map");

        // 默认绝不写入真实控制话题。
        pnh_.param("enable_control", enable_control_, false);
        pnh_.param("odom_timeout", odom_timeout_, 0.30);
        pnh_.param("fusion_timeout", fusion_timeout_, 0.40);
        pnh_.param("navigation_timeout", navigation_timeout_, 0.50);
        pnh_.param("max_xy_speed", max_xy_speed_, 1.00);
        pnh_.param("level1_avoidance_gain", level1_avoidance_gain_, 1.00);
        pnh_.param("level1_tangent_speed", level1_tangent_speed_, 0.15);
        pnh_.param(
            "level1_opposition_cosine",
            level1_opposition_cosine_,
            -0.80);
        pnh_.param("level1_stall_speed", level1_stall_speed_, 0.08);
        pnh_.param("level1_side_preference", level1_side_preference_, 1.0);
        pnh_.param("trap_detection_window", trap_detection_window_, 2.0);
        pnh_.param("trap_min_progress", trap_min_progress_, 0.10);
        pnh_.param("trap_escape_duration", trap_escape_duration_, 1.50);
        pnh_.param("trap_release_progress", trap_release_progress_, 0.15);
        pnh_.param("trap_goal_tolerance", trap_goal_tolerance_, 0.25);
        pnh_.param("navigation_position_kp", navigation_position_kp_, 0.60);
        pnh_.param("configured_target_altitude", configured_altitude_, -1.0);
        pnh_.param(
            "avoidance_enable_altitude",
            avoidance_enable_altitude_,
            0.60);
        pnh_.param(
            "avoidance_disable_altitude",
            avoidance_disable_altitude_,
            0.40);

        odom_timeout_ = std::max(0.05, odom_timeout_);
        fusion_timeout_ = std::max(0.05, fusion_timeout_);
        navigation_timeout_ = std::max(0.05, navigation_timeout_);
        max_xy_speed_ = std::max(0.05, max_xy_speed_);
        level1_avoidance_gain_ = std::max(0.0, level1_avoidance_gain_);
        level1_tangent_speed_ = std::max(
            0.0, std::min(level1_tangent_speed_, max_xy_speed_));
        level1_opposition_cosine_ = std::max(
            -1.0, std::min(level1_opposition_cosine_, 0.0));
        level1_stall_speed_ = std::max(
            0.0, std::min(level1_stall_speed_, max_xy_speed_));
        level1_side_preference_ = level1_side_preference_ >= 0.0
            ? 1.0
            : -1.0;
        trap_detection_window_ = std::max(0.50, trap_detection_window_);
        trap_min_progress_ = std::max(0.01, trap_min_progress_);
        trap_escape_duration_ = std::max(0.20, trap_escape_duration_);
        trap_release_progress_ = std::max(0.01, trap_release_progress_);
        trap_goal_tolerance_ = std::max(0.05, trap_goal_tolerance_);
        navigation_position_kp_ = std::max(0.0, navigation_position_kp_);
        avoidance_enable_altitude_ = std::max(
            0.0, avoidance_enable_altitude_);
        avoidance_disable_altitude_ = std::max(
            0.0,
            std::min(
                avoidance_disable_altitude_,
                avoidance_enable_altitude_));

        odom_sub_ = nh_.subscribe(
            odom_topic_, 20, &AvoidanceCommandBridge::odomCallback, this);
        risk_sub_ = nh_.subscribe(
            risk_topic_, 10, &AvoidanceCommandBridge::riskCallback, this);
        velocity_sub_ = nh_.subscribe(
            velocity_topic_,
            10,
            &AvoidanceCommandBridge::velocityCallback,
            this);
        navigation_sub_ = nh_.subscribe(
            navigation_topic_,
            10,
            &AvoidanceCommandBridge::navigationCallback,
            this);

        preview_pub_ = nh_.advertise<prometheus_msgs::UAVCommand>(
            preview_topic_, 10);

        if (enable_control_)
        {
            command_pub_ = nh_.advertise<prometheus_msgs::UAVCommand>(
                command_topic_, 10);

            ROS_WARN(
                "[AvoidanceCommandBridge] REAL CONTROL ENABLED. "
                "Stop every other publisher on %s before flight testing.",
                command_topic_.c_str());
        }
        else
        {
            ROS_INFO(
                "[AvoidanceCommandBridge] Preview mode: commands are only "
                "published on %s.",
                preview_topic_.c_str());
        }

        ROS_INFO(
            "[AvoidanceCommandBridge] Navigation input: %s; "
            "level0=pass, level1=blend, level2=avoidance override.",
            navigation_topic_.c_str());

        ROS_INFO(
            "[AvoidanceCommandBridge] Altitude gate: enable>=%.2f m, "
            "disable<=%.2f m.",
            avoidance_enable_altitude_,
            avoidance_disable_altitude_);

        ROS_INFO(
            "[AvoidanceCommandBridge] Level1 tangent escape: "
            "speed=%.2f stall<=%.2f opposition<=%.2f side=%+.0f.",
            level1_tangent_speed_,
            level1_stall_speed_,
            level1_opposition_cosine_,
            level1_side_preference_);

        ROS_INFO(
            "[AvoidanceCommandBridge] Trap monitor: window=%.2f s "
            "progress>=%.2f m escape=%.2f s release=%.2f m "
            "goal<=%.2f m.",
            trap_detection_window_,
            trap_min_progress_,
            trap_escape_duration_,
            trap_release_progress_,
            trap_goal_tolerance_);

        timer_ = nh_.createTimer(
            ros::Duration(0.05),
            &AvoidanceCommandBridge::timerCallback,
            this);
    }

private:
    static double quaternionToYaw(
        const geometry_msgs::Quaternion& quaternion)
    {
        const double sin_yaw =
            2.0 * (quaternion.w * quaternion.z +
                   quaternion.x * quaternion.y);
        const double cos_yaw =
            1.0 - 2.0 *
                (quaternion.y * quaternion.y +
                 quaternion.z * quaternion.z);
        return std::atan2(sin_yaw, cos_yaw);
    }

    static void limitHorizontalVelocity(
        double& vx,
        double& vy,
        const double max_speed)
    {
        const double speed = std::hypot(vx, vy);

        if (speed > max_speed && speed > 1e-6)
        {
            const double scale = max_speed / speed;
            vx *= scale;
            vy *= scale;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        current_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
        odom_receive_time_ = ros::Time::now();
        have_odom_ = true;

        // 起飞和降落阶段不允许水平避障速度介入。使用不同的启用、
        // 关闭高度形成滞环，防止高度噪声导致门控反复切换。
        if (!avoidance_active_ &&
            current_z_ >= avoidance_enable_altitude_)
        {
            avoidance_active_ = true;
            ROS_INFO(
                "[AvoidanceCommandBridge] Altitude gate enabled at z=%.2f m.",
                current_z_);
        }
        else if (avoidance_active_ &&
                 current_z_ <= avoidance_disable_altitude_)
        {
            avoidance_active_ = false;
            ROS_INFO(
                "[AvoidanceCommandBridge] Altitude gate disabled at z=%.2f m.",
                current_z_);
        }

        // 预览和真实控制使用完全相同的保持目标：首次收到有效里程计
        // 时锁定高度与航向，后续不再随测量漂移更新目标值。
        if (!target_initialized_)
        {
            target_altitude_ = configured_altitude_ >= 0.0
                ? configured_altitude_
                : current_z_;
            target_yaw_ = current_yaw_;
            target_initialized_ = true;

            ROS_INFO(
                "[AvoidanceCommandBridge] Hold target captured: "
                "z=%.2f yaw=%.2f",
                target_altitude_,
                target_yaw_);
        }
    }

    void riskCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        risk_level_ = std::min<std::uint8_t>(msg->data, 2U);
        risk_receive_time_ = ros::Time::now();
        have_risk_ = true;
    }

    void velocityCallback(
        const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        avoidance_vx_ = msg->twist.linear.x;
        avoidance_vy_ = msg->twist.linear.y;
        velocity_receive_time_ = ros::Time::now();
        have_velocity_ = true;
    }

    void navigationCallback(
        const prometheus_msgs::UAVCommand::ConstPtr& msg)
    {
        const bool old_world_position_goal =
            have_navigation_ &&
            navigation_command_.Agent_CMD ==
                prometheus_msgs::UAVCommand::Move &&
            navigation_command_.Move_mode ==
                prometheus_msgs::UAVCommand::XYZ_POS;
        const bool new_world_position_goal =
            msg->Agent_CMD == prometheus_msgs::UAVCommand::Move &&
            msg->Move_mode == prometheus_msgs::UAVCommand::XYZ_POS;

        const bool target_changed =
            !old_world_position_goal ||
            !new_world_position_goal ||
            std::hypot(
                msg->position_ref[0] -
                    navigation_command_.position_ref[0],
                msg->position_ref[1] -
                    navigation_command_.position_ref[1]) > 0.05;

        if (target_changed)
        {
            resetTrapMonitor();
        }

        navigation_command_ = *msg;
        navigation_receive_time_ = ros::Time::now();
        have_navigation_ = true;
    }

    void resetTrapMonitor()
    {
        progress_monitor_initialized_ = false;
        trap_escape_active_ = false;
    }

    void updateTrapMonitor(
        const ros::Time& now,
        const std::uint8_t active_level,
        const bool navigation_available)
    {
        const bool has_world_position_goal =
            navigation_available &&
            navigation_command_.Agent_CMD ==
                prometheus_msgs::UAVCommand::Move &&
            navigation_command_.Move_mode ==
                prometheus_msgs::UAVCommand::XYZ_POS;

        if (!has_world_position_goal || active_level != 1U)
        {
            resetTrapMonitor();
            return;
        }

        const double goal_distance = std::hypot(
            navigation_command_.position_ref[0] - current_x_,
            navigation_command_.position_ref[1] - current_y_);

        if (goal_distance <= trap_goal_tolerance_)
        {
            resetTrapMonitor();
            return;
        }

        if (trap_escape_active_)
        {
            const bool made_escape_progress =
                goal_distance <=
                trap_escape_start_distance_ - trap_release_progress_;
            const bool escape_timed_out = now >= trap_escape_end_time_;

            if (!made_escape_progress && !escape_timed_out)
            {
                return;
            }

            ROS_INFO(
                "[AvoidanceCommandBridge] Trap escape released: "
                "distance=%.2f progress=%.2f timeout=%d.",
                goal_distance,
                trap_escape_start_distance_ - goal_distance,
                escape_timed_out ? 1 : 0);
            trap_escape_active_ = false;
            progress_monitor_initialized_ = false;
        }

        if (!progress_monitor_initialized_)
        {
            progress_reference_distance_ = goal_distance;
            progress_window_start_ = now;
            progress_monitor_initialized_ = true;
            return;
        }

        if (goal_distance <=
            progress_reference_distance_ - trap_min_progress_)
        {
            progress_reference_distance_ = goal_distance;
            progress_window_start_ = now;
            return;
        }

        if ((now - progress_window_start_).toSec() <
            trap_detection_window_)
        {
            return;
        }

        trap_escape_active_ = true;
        trap_escape_start_distance_ = goal_distance;
        trap_escape_end_time_ = now + ros::Duration(trap_escape_duration_);
        trap_escape_side_ =
            (trap_escape_count_ % 2U == 0U)
            ? level1_side_preference_
            : -level1_side_preference_;
        ++trap_escape_count_;
        progress_monitor_initialized_ = false;

        ROS_WARN(
            "[AvoidanceCommandBridge] Local trap detected: "
            "distance=%.2f progress=%.2f in %.2f s; "
            "escape side=%+.0f for %.2f s.",
            goal_distance,
            progress_reference_distance_ - goal_distance,
            trap_detection_window_,
            trap_escape_side_,
            trap_escape_duration_);
    }

    bool navigationVelocity(
        const prometheus_msgs::UAVCommand& navigation,
        double& vx,
        double& vy) const
    {
        if (navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XY_VEL_Z_POS ||
            navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XYZ_VEL)
        {
            vx = navigation.velocity_ref[0];
            vy = navigation.velocity_ref[1];
            return true;
        }

        if (navigation.Move_mode ==
            prometheus_msgs::UAVCommand::XYZ_POS)
        {
            vx = navigation_position_kp_ *
                (navigation.position_ref[0] - current_x_);
            vy = navigation_position_kp_ *
                (navigation.position_ref[1] - current_y_);
            return true;
        }

        return false;
    }

    double navigationAltitude(
        const prometheus_msgs::UAVCommand& navigation) const
    {
        if (navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XYZ_POS ||
            navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XY_VEL_Z_POS ||
            navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XYZ_POS_BODY ||
            navigation.Move_mode ==
                prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY)
        {
            return navigation.position_ref[2];
        }

        return current_z_;
    }

    static bool navigationCommandCanLatch(
        const prometheus_msgs::UAVCommand& navigation)
    {
        // uav_command_pub 的位置类命令通常只发布一次。位置目标可以安全
        // 锁存；含速度/姿态的 Move 命令必须持续更新，否则按超时处理。
        if (navigation.Agent_CMD !=
            prometheus_msgs::UAVCommand::Move)
        {
            return true;
        }

        return navigation.Move_mode ==
                   prometheus_msgs::UAVCommand::XYZ_POS ||
               navigation.Move_mode ==
                   prometheus_msgs::UAVCommand::XYZ_POS_BODY ||
               navigation.Move_mode ==
                   prometheus_msgs::UAVCommand::LAT_LON_ALT;
    }

    prometheus_msgs::UAVCommand makeHoverCommand(
        const ros::Time& now) const
    {
        prometheus_msgs::UAVCommand command;
        command.header.stamp = now;
        command.header.frame_id = world_frame_;
        command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        command.Control_Level =
            prometheus_msgs::UAVCommand::DEFAULT_CONTROL;
        command.Move_mode =
            prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
        command.position_ref[0] = static_cast<float>(current_x_);
        command.position_ref[1] = static_cast<float>(current_y_);
        command.position_ref[2] = static_cast<float>(target_altitude_);
        command.velocity_ref[0] = 0.0F;
        command.velocity_ref[1] = 0.0F;
        command.velocity_ref[2] = 0.0F;
        command.acceleration_ref[0] = 0.0F;
        command.acceleration_ref[1] = 0.0F;
        command.acceleration_ref[2] = 0.0F;
        command.yaw_ref = static_cast<float>(target_yaw_);
        command.Yaw_Rate_Mode = false;
        command.yaw_rate_ref = 0.0F;
        return command;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        const bool navigation_recent =
            have_navigation_ &&
            (now - navigation_receive_time_).toSec() <=
                navigation_timeout_;
        const bool navigation_latched =
            have_navigation_ &&
            navigationCommandCanLatch(navigation_command_);
        const bool navigation_available =
            navigation_recent || navigation_latched;

        // Land、Takeoff、Disarm 等非 Move 状态命令必须拥有最高
        // 仲裁优先级。它们不依赖里程计和避障融合，也不能被水平
        // 避障速度替换，否则控制器会继续执行先前的悬停/速度命令。
        if (navigation_available &&
            navigation_command_.Agent_CMD !=
                prometheus_msgs::UAVCommand::Move)
        {
            resetTrapMonitor();

            prometheus_msgs::UAVCommand action = navigation_command_;
            action.header.stamp = now;
            if (action.header.frame_id.empty())
            {
                action.header.frame_id = world_frame_;
            }
            action.Command_ID = ++command_id_;

            preview_pub_.publish(action);
            if (enable_control_)
            {
                command_pub_.publish(action);
            }

            ROS_INFO_THROTTLE(
                0.25,
                "[AvoidanceCommandBridge] preview=1 control=%d "
                "gate=%d trap=0 level=0 source=priority_action "
                "nav=1 Agent_CMD=%u id=%u",
                enable_control_ ? 1 : 0,
                avoidance_active_ ? 1 : 0,
                static_cast<unsigned int>(action.Agent_CMD),
                command_id_);
            return;
        }

        if (!have_odom_ ||
            (now - odom_receive_time_).toSec() > odom_timeout_ ||
            !target_initialized_)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[AvoidanceCommandBridge] Odometry missing or stale; "
                "no command is generated.");
            return;
        }

        const bool fusion_fresh =
            have_risk_ &&
            have_velocity_ &&
            (now - risk_receive_time_).toSec() <= fusion_timeout_ &&
            (now - velocity_receive_time_).toSec() <= fusion_timeout_;

        const std::uint8_t active_level =
            fusion_fresh && avoidance_active_
            ? risk_level_
            : 0U;

        updateTrapMonitor(now, active_level, navigation_available);

        prometheus_msgs::UAVCommand command = makeHoverCommand(now);
        double command_vx = 0.0;
        double command_vy = 0.0;
        const char* source = "hover";

        if (fusion_fresh && navigation_available && active_level == 0U)
        {
            // 无风险时完整转发导航命令，仲裁器只重写时间戳和命令编号。
            command = navigation_command_;
            command.header.stamp = now;
            if (command.header.frame_id.empty())
            {
                command.header.frame_id = world_frame_;
            }
            command_vx = command.velocity_ref[0];
            command_vy = command.velocity_ref[1];
            source = "navigation";
        }
        else if (active_level > 0U)
        {
            double navigation_vx = 0.0;
            double navigation_vy = 0.0;
            const bool can_blend_navigation =
                navigation_available &&
                navigation_command_.Agent_CMD ==
                    prometheus_msgs::UAVCommand::Move &&
                navigationVelocity(
                    navigation_command_, navigation_vx, navigation_vy);

            command = makeHoverCommand(now);
            if (navigation_available)
            {
                command.position_ref[2] = static_cast<float>(
                    navigationAltitude(navigation_command_));
                command.yaw_ref = navigation_command_.yaw_ref;
                command.Yaw_Rate_Mode =
                    navigation_command_.Yaw_Rate_Mode;
                command.yaw_rate_ref =
                    navigation_command_.yaw_rate_ref;
            }

            if (active_level == 1U && can_blend_navigation)
            {
                command_vx = navigation_vx +
                    level1_avoidance_gain_ * avoidance_vx_;
                command_vy = navigation_vy +
                    level1_avoidance_gain_ * avoidance_vy_;
                source = "blend";

                const double navigation_speed = std::hypot(
                    navigation_vx, navigation_vy);
                const double avoidance_speed = std::hypot(
                    avoidance_vx_, avoidance_vy_);
                const double blended_speed = std::hypot(
                    command_vx, command_vy);

                if (trap_escape_active_)
                {
                    double tangent_x = 0.0;
                    double tangent_y = 0.0;

                    if (avoidance_speed > 0.05)
                    {
                        tangent_x = trap_escape_side_ *
                            (-avoidance_vy_ / avoidance_speed);
                        tangent_y = trap_escape_side_ *
                            (avoidance_vx_ / avoidance_speed);
                    }
                    else if (navigation_speed > 0.05)
                    {
                        tangent_x = trap_escape_side_ *
                            (-navigation_vy / navigation_speed);
                        tangent_y = trap_escape_side_ *
                            (navigation_vx / navigation_speed);
                    }

                    command_vx += level1_tangent_speed_ * tangent_x;
                    command_vy += level1_tangent_speed_ * tangent_y;
                    source = "blend_escape";
                }

                // 导航吸引速度与避障排斥速度正面对抗时，简单相加会
                // 落入人工势场局部最小值。仅在 level=1 且融合结果
                // 接近停滞时加入固定侧向的切向速度，绕过障碍；
                // level=2 仍保持纯紧急避障，不使用该逻辑。
                if (!trap_escape_active_ &&
                    navigation_speed > 0.05 &&
                    avoidance_speed > 0.05 &&
                    blended_speed <= level1_stall_speed_)
                {
                    const double opposition_cosine =
                        (navigation_vx * avoidance_vx_ +
                         navigation_vy * avoidance_vy_) /
                        (navigation_speed * avoidance_speed);

                    if (opposition_cosine <=
                        level1_opposition_cosine_)
                    {
                        const double tangent_x =
                            level1_side_preference_ *
                            (-avoidance_vy_ / avoidance_speed);
                        const double tangent_y =
                            level1_side_preference_ *
                            (avoidance_vx_ / avoidance_speed);

                        command_vx +=
                            level1_tangent_speed_ * tangent_x;
                        command_vy +=
                            level1_tangent_speed_ * tangent_y;
                        source = "blend_tangent";

                        ROS_INFO_THROTTLE(
                            0.5,
                            "[AvoidanceCommandBridge] Tangent escape "
                            "engaged: cos=%.2f blended=%.2f.",
                            opposition_cosine,
                            blended_speed);
                    }
                }
            }
            else
            {
                // level=2 必须停止导航；不支持的导航模式在 level=1
                // 也采用纯避障，避免混用 map/body 坐标系。
                command_vx = avoidance_vx_;
                command_vy = avoidance_vy_;
                source = active_level == 2U
                    ? "emergency"
                    : "avoidance";
            }

            limitHorizontalVelocity(
                command_vx, command_vy, max_xy_speed_);
            command.velocity_ref[0] = static_cast<float>(command_vx);
            command.velocity_ref[1] = static_cast<float>(command_vy);
        }

        command.Command_ID = ++command_id_;

        preview_pub_.publish(command);

        if (enable_control_)
        {
            command_pub_.publish(command);
        }

        if (!fusion_fresh)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[AvoidanceCommandBridge] Fusion data stale; "
                "publishing zero-XY hover command.");
        }

        ROS_INFO_THROTTLE(
            0.25,
            "[AvoidanceCommandBridge] preview=%d control=%d gate=%d "
            "trap=%d level=%u source=%s nav=%d v=(%.2f %.2f) "
            "z=%.2f yaw=%.2f id=%u",
            1,
            enable_control_ ? 1 : 0,
            avoidance_active_ ? 1 : 0,
            trap_escape_active_ ? 1 : 0,
            static_cast<unsigned int>(active_level),
            source,
            navigation_available ? 1 : 0,
            command_vx,
            command_vy,
            command.position_ref[2],
            command.yaw_ref,
            command_id_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber risk_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber navigation_sub_;
    ros::Publisher preview_pub_;
    ros::Publisher command_pub_;
    ros::Timer timer_;

    std::string odom_topic_;
    std::string risk_topic_;
    std::string velocity_topic_;
    std::string preview_topic_;
    std::string command_topic_;
    std::string navigation_topic_;
    std::string world_frame_;

    bool enable_control_{false};
    double odom_timeout_{0.30};
    double fusion_timeout_{0.40};
    double navigation_timeout_{0.50};
    double max_xy_speed_{1.00};
    double level1_avoidance_gain_{1.00};
    double level1_tangent_speed_{0.15};
    double level1_opposition_cosine_{-0.80};
    double level1_stall_speed_{0.08};
    double level1_side_preference_{1.0};
    double trap_detection_window_{2.0};
    double trap_min_progress_{0.10};
    double trap_escape_duration_{1.50};
    double trap_release_progress_{0.15};
    double trap_goal_tolerance_{0.25};
    double navigation_position_kp_{0.60};
    double configured_altitude_{-1.0};
    double avoidance_enable_altitude_{0.60};
    double avoidance_disable_altitude_{0.40};

    bool have_odom_{false};
    bool have_risk_{false};
    bool have_velocity_{false};
    bool have_navigation_{false};
    bool target_initialized_{false};
    bool avoidance_active_{false};
    bool progress_monitor_initialized_{false};
    bool trap_escape_active_{false};

    ros::Time odom_receive_time_;
    ros::Time risk_receive_time_;
    ros::Time velocity_receive_time_;
    ros::Time navigation_receive_time_;
    ros::Time progress_window_start_;
    ros::Time trap_escape_end_time_;

    prometheus_msgs::UAVCommand navigation_command_;

    std::uint8_t risk_level_{0U};
    std::uint32_t command_id_{0U};
    std::uint32_t trap_escape_count_{0U};
    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};
    double current_yaw_{0.0};
    double target_altitude_{0.0};
    double target_yaw_{0.0};
    double avoidance_vx_{0.0};
    double avoidance_vy_{0.0};
    double progress_reference_distance_{0.0};
    double trap_escape_start_distance_{0.0};
    double trap_escape_side_{1.0};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidance_command_bridge");
    AvoidanceCommandBridge node;
    ros::spin();
    return 0;
}
