#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Native MAVROS controller for AstraDroneOpen.
//
// EXP originally emitted prometheus_msgs/UAVCommand.  This project uses the
// standard, non-namespaced MAVROS interface, so this node closes the loop with
// velocity setpoints while retaining EXP's fused static/dynamic risk output.
class MavrosAvoidanceController
{
public:
    enum class Phase { WAIT_DATA, PRESTREAM, TAKEOFF, NAVIGATE, HOLD };

    MavrosAvoidanceController() : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("state_topic", state_topic_, "/mavros/state");
        pnh_.param<std::string>("mavros_pose_topic", mavros_pose_topic_,
                                "/mavros/local_position/pose");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/uav1/fastlio/odom");
        pnh_.param("use_path_planner", use_path_planner_, true);
        pnh_.param<std::string>("final_goal_topic", final_goal_topic_,
                                "/move_base_simple/goal");
        pnh_.param<std::string>("planner_waypoint_topic",
                                planner_waypoint_topic_,
                                "/uav1/planner/waypoint");
        goal_topic_ = use_path_planner_
            ? planner_waypoint_topic_ : final_goal_topic_;
        pnh_.param<std::string>("risk_topic", risk_topic_,
                                "/uav1/fused_collision_risk_level");
        pnh_.param<std::string>("avoidance_topic", avoidance_topic_,
                                "/uav1/fused_avoidance_velocity");
        pnh_.param<std::string>("setpoint_topic", setpoint_topic_,
                                "/mavros/setpoint_velocity/cmd_vel");
        pnh_.param("enable_control", enable_control_, false);
        pnh_.param("auto_arm", auto_arm_, true);
        pnh_.param("auto_goal", auto_goal_, true);
        pnh_.param<std::string>("auto_goal_mode", auto_goal_mode_, "relative");
        pnh_.param("relative_goal_x", relative_goal_x_, 0.0);
        pnh_.param("relative_goal_y", relative_goal_y_, 6.0);
        pnh_.param("absolute_goal_x", absolute_goal_x_, 0.0);
        pnh_.param("absolute_goal_y", absolute_goal_y_, 6.0);
        pnh_.param("absolute_goal_z", absolute_goal_z_, 1.5);
        pnh_.param("takeoff_height", takeoff_height_, 1.5);
        pnh_.param("prestream_seconds", prestream_seconds_, 3.0);
        pnh_.param("goal_delay_seconds", goal_delay_seconds_, 3.0);
        pnh_.param("position_kp", position_kp_, 0.45);
        pnh_.param("altitude_kp", altitude_kp_, 0.8);
        pnh_.param("max_navigation_speed", max_navigation_speed_, 0.65);
        pnh_.param("max_total_speed", max_total_speed_, 0.85);
        pnh_.param("max_vertical_speed", max_vertical_speed_, 0.5);
        pnh_.param("avoidance_gain", avoidance_gain_, 1.0);
        pnh_.param("level1_navigation_scale", level1_navigation_scale_, 0.15);
        pnh_.param("stop_on_level1", stop_on_level1_, true);
        pnh_.param("static_recovery_speed", static_recovery_speed_, 0.15);
        pnh_.param("recovery_surface_distance", recovery_surface_distance_, 0.65);
        pnh_.param("recovery_horizon", recovery_horizon_, 1.0);
        static_recovery_speed_ = std::max(0.0, std::min(0.20, static_recovery_speed_));
        recovery_surface_distance_ = std::max(0.65, recovery_surface_distance_);
        recovery_horizon_ = std::max(1.0, recovery_horizon_);
        pnh_.param("goal_tolerance", goal_tolerance_, 0.25);
        pnh_.param("waypoint_tolerance", waypoint_tolerance_, 0.03);
        waypoint_tolerance_ = std::max(0.01, std::min(goal_tolerance_, waypoint_tolerance_));
        pnh_.param("takeoff_tolerance", takeoff_tolerance_, 0.12);
        pnh_.param("risk_timeout", risk_timeout_, 0.45);
        pnh_.param("odom_timeout", odom_timeout_, 0.35);
        pnh_.param("publish_rate", publish_rate_, 30.0);

        level1_navigation_scale_ = std::max(
            0.0, std::min(1.0, level1_navigation_scale_));
        if (auto_goal_mode_ != "relative" && auto_goal_mode_ != "absolute")
        {
            ROS_WARN("[MavrosAvoidanceController] Unknown auto_goal_mode='%s'; "
                     "falling back to 'relative'.", auto_goal_mode_.c_str());
            auto_goal_mode_ = "relative";
        }

        state_sub_ = nh_.subscribe(state_topic_, 10, &MavrosAvoidanceController::stateCb, this);
        mavros_pose_sub_ = nh_.subscribe(mavros_pose_topic_, 20,
                                         &MavrosAvoidanceController::mavrosPoseCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &MavrosAvoidanceController::odomCb, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 5, &MavrosAvoidanceController::goalCb, this);
        if (use_path_planner_)
        {
            final_goal_monitor_sub_ = nh_.subscribe(
                final_goal_topic_, 5,
                &MavrosAvoidanceController::finalGoalObservedCb, this);
            final_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                final_goal_topic_, 1, true);
        }
        risk_sub_ = nh_.subscribe(risk_topic_, 10, &MavrosAvoidanceController::riskCb, this);
        avoidance_sub_ = nh_.subscribe(avoidance_topic_, 10,
                                       &MavrosAvoidanceController::avoidanceCb, this);
        source_sub_ = nh_.subscribe("/uav1/fused_avoidance_source", 1,
            &MavrosAvoidanceController::sourceCb, this);
        dynamic_risk_sub_ = nh_.subscribe("/uav1/collision_risk_level", 1,
            &MavrosAvoidanceController::dynamicRiskCb, this);
        scan_sub_ = nh_.subscribe("/uav1/fastlio/registered_scan", 1,
            &MavrosAvoidanceController::scanCb, this);
        setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(setpoint_topic_, 20);
        preview_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("command_preview", 20);
        // Latched one-shot signal used by the demo to start pedestrians only
        // when takeoff is complete and horizontal navigation is about to begin.
        navigation_ready_pub_ = pnh_.advertise<std_msgs::Bool>("navigation_ready", 1, true);
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(5.0, publish_rate_)),
                                 &MavrosAvoidanceController::timerCb, this);

        ROS_WARN("[MavrosAvoidanceController] control=%s auto_arm=%s auto_goal=%s. "
                 "Do not run another Offboard setpoint publisher at the same time.",
                 enable_control_ ? "ENABLED" : "preview-only",
                 auto_arm_ ? "true" : "false", auto_goal_ ? "true" : "false");
        ROS_INFO("[MavrosAvoidanceController] automatic goal mode: %s",
                 auto_goal_mode_.c_str());
        ROS_INFO("[MavrosAvoidanceController] path planner: %s; control goal=%s",
                 use_path_planner_ ? "enabled" : "disabled",
                 goal_topic_.c_str());
    }

private:
    void stateCb(const mavros_msgs::State::ConstPtr& msg) { state_ = *msg; }

    void mavrosPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        mavros_pose_ = *msg;
        mavros_pose_time_ = ros::Time::now();
        if (!have_mavros_pose_)
        {
            home_z_ = msg->pose.position.z;
            // Takeoff is home-relative MAVROS Z. Navigation goals are map Z:
            // these origins are NOT interchangeable, even with aligned ENU axes.
            target_z_ = home_z_ + takeoff_height_;
        }
        have_mavros_pose_ = true;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_ = *msg;
        odom_time_ = ros::Time::now();
        have_odom_ = true;
    }

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "map" &&
            msg->header.frame_id != "/map")
        {
            ROS_ERROR("[MavrosAvoidanceController] Reject goal frame '%s'; expected map.",
                      msg->header.frame_id.c_str());
            return;
        }
        goal_ = *msg;
        if (goal_.pose.position.z <= 0.1 && have_odom_)
            goal_.pose.position.z = odom_.pose.pose.position.z;
        goal_time_ = ros::Time::now();
        have_goal_ = true;
        ROS_INFO_THROTTLE(1.0, "[MavrosAvoidanceController] waypoint map=(%.2f %.2f %.2f)",
                 goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
    }

    void finalGoalObservedCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // Prevent the delayed automatic goal from overwriting a destination
        // already selected in RViz while the planner is building its first path.
        final_goal_observed_ = true;
        final_goal_ = *msg;
    }

    void riskCb(const std_msgs::UInt8::ConstPtr& msg)
    {
        risk_level_ = std::min<std::uint8_t>(2U, msg->data);
        risk_time_ = ros::Time::now();
        have_risk_ = true;
    }

    void avoidanceCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        avoidance_ = *msg;
        avoidance_time_ = ros::Time::now();
        have_avoidance_ = true;
    }

    void sourceCb(const std_msgs::String::ConstPtr& msg)
    {
        avoidance_source_ = msg->data;
        source_time_ = ros::Time::now();
    }

    void dynamicRiskCb(const std_msgs::UInt8::ConstPtr& msg)
    {
        dynamic_risk_ = msg->data;
        dynamic_risk_time_ = ros::Time::now();
    }

    void scanCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (msg->header.frame_id != "map" && msg->header.frame_id != "/map") return;
        pcl::fromROSMsg(*msg, recovery_scan_);
        scan_time_ = msg->header.stamp;  // Do not treat delayed scans as fresh.
    }

    bool staticRecovery(const ros::Time& now, double& vx, double& vy) const
    {
        // Static shelves do not move away when we wait. Retreat slowly, only
        // for a fresh static-only warning; pedestrians retain stop-and-wait.
        if (static_recovery_speed_ <= 0.0 || avoidance_source_ != "static_avoid" ||
            source_time_.isZero() || dynamic_risk_time_.isZero() || scan_time_.isZero() ||
            (now - source_time_).toSec() > risk_timeout_ ||
            (now - dynamic_risk_time_).toSec() > risk_timeout_ || dynamic_risk_ != 0U ||
            (now - scan_time_).toSec() < 0.0 ||
            (now - scan_time_).toSec() > 0.30 || recovery_scan_.empty()) return false;
        const double ax = avoidance_.twist.linear.x, ay = avoidance_.twist.linear.y;
        const double norm = std::hypot(ax, ay);
        if (!std::isfinite(norm) || norm < 0.05) return false;
        const double candidate_x = static_recovery_speed_ * ax / norm;
        const double candidate_y = static_recovery_speed_ * ay / norm;
        const double sx = candidate_x * recovery_horizon_;
        const double sy = candidate_y * recovery_horizon_;
        // Check the entire retreat capsule, not just the nearest obstacle:
        // moving away from one shelf must not back into the opposite shelf.
        // This is a scan-based guard, not a proof about occluded/unknown space.
        for (const auto& p : recovery_scan_)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const double dz = p.z - odom_.pose.pose.position.z;
            if (dz < -0.45 || dz > 0.55) continue;
            const double x = p.x - odom_.pose.pose.position.x;
            const double y = p.y - odom_.pose.pose.position.y;
            const double t = std::max(0.0, std::min(1.0, (x*sx + y*sy)/(sx*sx + sy*sy)));
            if (std::hypot(x-t*sx, y-t*sy) < recovery_surface_distance_) return false;
        }
        vx = candidate_x;
        vy = candidate_y;
        return true;
    }

    void signalNavigationReady()
    {
        if (navigation_ready_sent_) return;
        std_msgs::Bool ready;
        ready.data = true;
        navigation_ready_pub_.publish(ready);
        navigation_ready_sent_ = true;
    }

    static void limit2D(double& x, double& y, double maximum)
    {
        const double norm = std::hypot(x, y);
        if (norm > maximum && norm > 1e-6)
        {
            x *= maximum / norm;
            y *= maximum / norm;
        }
    }

    void requestFlightMode(const ros::Time& now)
    {
        if (!enable_control_ || !auto_arm_ ||
            (now - last_request_).toSec() < 2.0) return;

        if (state_.mode != "OFFBOARD")
        {
            mavros_msgs::SetMode mode;
            mode.request.custom_mode = "OFFBOARD";
            if (mode_client_.call(mode) && mode.response.mode_sent)
                ROS_INFO("[MavrosAvoidanceController] OFFBOARD requested.");
        }
        else if (!state_.armed)
        {
            mavros_msgs::CommandBool arm;
            arm.request.value = true;
            if (arm_client_.call(arm) && arm.response.success)
                ROS_INFO("[MavrosAvoidanceController] arming requested.");
        }
        last_request_ = now;
    }

    bool dataFresh(const ros::Time& now) const
    {
        return have_odom_ && have_risk_ && have_avoidance_ &&
               (now - mavros_pose_time_).toSec() <= odom_timeout_ &&
               (now - odom_time_).toSec() <= odom_timeout_ &&
               (now - risk_time_).toSec() <= risk_timeout_ &&
               (now - avoidance_time_).toSec() <= risk_timeout_;
    }

    void timerCb(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();
        geometry_msgs::TwistStamped command;
        command.header.stamp = now;
        command.header.frame_id = "map";

        if (!state_.connected || !have_mavros_pose_)
        {
            ROS_WARN_THROTTLE(2.0, "[MavrosAvoidanceController] Waiting for MAVROS state/pose.");
            publish(command);
            return;
        }

        if (phase_ == Phase::WAIT_DATA)
        {
            phase_ = Phase::PRESTREAM;
            phase_start_ = now;
        }

        if (phase_ == Phase::PRESTREAM)
        {
            if ((now - phase_start_).toSec() >= prestream_seconds_)
            {
                requestFlightMode(now);
                if (!auto_arm_ || (state_.mode == "OFFBOARD" && state_.armed))
                {
                    phase_ = Phase::TAKEOFF;
                    phase_start_ = now;
                    ROS_INFO("[MavrosAvoidanceController] TAKEOFF to %.2f m.", target_z_);
                }
            }
            publish(command);
            return;
        }

        requestFlightMode(now);

        // During flight, compare map goal Z with map odometry Z. Using MAVROS
        // Z here caused each RViz 2D goal to add the map/local origin offset.
        const double z_error = phase_ != Phase::TAKEOFF && have_goal_ && have_odom_
            ? goal_.pose.position.z - odom_.pose.pose.position.z
            : target_z_ - mavros_pose_.pose.position.z;
        command.twist.linear.z = std::max(-max_vertical_speed_,
            std::min(max_vertical_speed_, altitude_kp_ * z_error));

        if (phase_ == Phase::TAKEOFF)
        {
            if (std::fabs(z_error) <= takeoff_tolerance_)
            {
                phase_ = Phase::NAVIGATE;
                phase_start_ = now;
                ROS_INFO("[MavrosAvoidanceController] Takeoff complete; waiting for navigation goal.");
            }
            publish(command);
            return;
        }

        if (auto_goal_ && !have_goal_ && !final_goal_observed_ && have_odom_ &&
            (now - phase_start_).toSec() >= goal_delay_seconds_)
        {
            geometry_msgs::PoseStamped automatic_goal;
            automatic_goal.header.stamp = now;
            automatic_goal.header.frame_id = "map";
            if (auto_goal_mode_ == "absolute")
            {
                // Fixed map coordinate, useful for a repeatable experiment.
                automatic_goal.pose.position.x = absolute_goal_x_;
                automatic_goal.pose.position.y = absolute_goal_y_;
            }
            else
            {
                // Offset from the FAST-LIO position observed after takeoff.
                automatic_goal.pose.position.x =
                    odom_.pose.pose.position.x + relative_goal_x_;
                automatic_goal.pose.position.y =
                    odom_.pose.pose.position.y + relative_goal_y_;
            }
            automatic_goal.pose.position.z = auto_goal_mode_ == "absolute"
                ? absolute_goal_z_
                : odom_.pose.pose.position.z + target_z_ - mavros_pose_.pose.position.z;
            automatic_goal.pose.orientation.w = 1.0;

            if (use_path_planner_)
            {
                final_goal_observed_ = true;
                final_goal_pub_.publish(automatic_goal);
            }
            else
            {
                goal_ = automatic_goal;
                goal_time_ = now;
                have_goal_ = true;
            }
            ROS_INFO("[MavrosAvoidanceController] Automatic %s goal=(%.2f %.2f %.2f).",
                     auto_goal_mode_.c_str(),
                     automatic_goal.pose.position.x,
                     automatic_goal.pose.position.y, automatic_goal.pose.position.z);
        }

        // Also release the delayed pedestrian scenario for a goal supplied by
        // RViz/rostopic.  Publishing this only once avoids repeatedly launching
        // the scenario when a user changes the destination during flight.
        if (have_goal_) signalNavigationReady();

        if (!have_goal_ || !dataFresh(now))
        {
            command.twist.linear.z = 0.0;
            ROS_WARN_THROTTLE(1.0, "[MavrosAvoidanceController] Holding: goal or fresh avoidance chain missing.");
            publish(command);
            return;
        }

        const double dx = goal_.pose.position.x - odom_.pose.pose.position.x;
        const double dy = goal_.pose.position.y - odom_.pose.pose.position.y;
        const double goal_distance = std::hypot(dx, dy);
        // A corner is NOT the final destination. Stopping 25cm before a
        // visible corner prevents the planner from exposing the next segment.
        // Preserve final-goal hover tolerance but track intermediate corners
        // more accurately; collision protection still overrides this command.
        const bool final_waypoint = final_goal_observed_ &&
            std::hypot(goal_.pose.position.x-final_goal_.pose.position.x,
                       goal_.pose.position.y-final_goal_.pose.position.y) < 0.05;
        const double tolerance = use_path_planner_ && !final_waypoint
            ? waypoint_tolerance_ : goal_tolerance_;
        const bool waypoint_reached = goal_distance <= tolerance;
        if (waypoint_reached)
        {
            phase_ = Phase::HOLD;
            command.twist.linear.x = 0.0;
            command.twist.linear.y = 0.0;
            ROS_INFO_THROTTLE(2.0, "[MavrosAvoidanceController] Waypoint/hold position reached; not necessarily the final goal.");
            // Do not return here: emergency/recovery also applies while HOLD.
        }

        if (!waypoint_reached) phase_ = Phase::NAVIGATE;
        const bool planner_fresh = !use_path_planner_ || (now - goal_time_).toSec() <= 1.5;
        double nav_x = waypoint_reached || !planner_fresh ? 0.0 : position_kp_ * dx;
        double nav_y = waypoint_reached || !planner_fresh ? 0.0 : position_kp_ * dy;
        limit2D(nav_x, nav_y, max_navigation_speed_);

        if (risk_level_ == 0U)
        {
            command.twist.linear.x = nav_x;
            command.twist.linear.y = nav_y;
        }
        else if (risk_level_ == 1U)
        {
            if (stop_on_level1_)
            {
                // 交叉交通优先采用 stop-and-wait：预警时悬停让行，避免
                // 横移方向与行人轨迹再次相交。若目标继续逼近，level 2
                // 仍会立即切换到主动逃逸速度。
                command.twist.linear.x = 0.0;
                command.twist.linear.y = 0.0;
                const bool recovering = staticRecovery(now, command.twist.linear.x,
                                                        command.twist.linear.y);
                ROS_INFO_THROTTLE(1.0, "[MavrosAvoidanceController] level1 source=%s action=%s",
                    avoidance_source_.c_str(), recovering ? "guarded_static_retreat" : "hold");
            }
            else
            {
                command.twist.linear.x =
                    level1_navigation_scale_ * nav_x +
                    avoidance_gain_ * avoidance_.twist.linear.x;
                command.twist.linear.y =
                    level1_navigation_scale_ * nav_y +
                    avoidance_gain_ * avoidance_.twist.linear.y;
            }
        }
        else
        {
            // Emergency level suppresses goal attraction; moving away from the
            // threat has priority over progress toward the goal.
            command.twist.linear.x = avoidance_.twist.linear.x;
            command.twist.linear.y = avoidance_.twist.linear.y;
        }
        limit2D(command.twist.linear.x, command.twist.linear.y, max_total_speed_);
        ROS_INFO_THROTTLE(0.5, "[MavrosAvoidanceController] risk=%u goal_d=%.2f cmd=(%.2f %.2f %.2f)",
                          static_cast<unsigned>(risk_level_), goal_distance,
                          command.twist.linear.x, command.twist.linear.y,
                          command.twist.linear.z);
        publish(command);
    }

    void publish(const geometry_msgs::TwistStamped& command)
    {
        preview_pub_.publish(command);
        if (enable_control_) setpoint_pub_.publish(command);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber state_sub_, mavros_pose_sub_, odom_sub_, goal_sub_;
    ros::Subscriber final_goal_monitor_sub_, risk_sub_, avoidance_sub_;
    ros::Subscriber source_sub_, dynamic_risk_sub_, scan_sub_;
    pcl::PointCloud<pcl::PointXYZ> recovery_scan_;
    std::string avoidance_source_;
    std::uint8_t dynamic_risk_{255U};
    ros::Time source_time_, dynamic_risk_time_, scan_time_, goal_time_;
    double static_recovery_speed_{0.15}, recovery_surface_distance_{0.65}, recovery_horizon_{1.0};
    ros::Publisher setpoint_pub_, preview_pub_, navigation_ready_pub_;
    ros::Publisher final_goal_pub_;
    ros::ServiceClient arm_client_, mode_client_;
    ros::Timer timer_;
    mavros_msgs::State state_;
    geometry_msgs::PoseStamped mavros_pose_, goal_, final_goal_;
    geometry_msgs::TwistStamped avoidance_;
    nav_msgs::Odometry odom_;
    std::string state_topic_, mavros_pose_topic_, odom_topic_, goal_topic_, risk_topic_;
    std::string final_goal_topic_, planner_waypoint_topic_;
    std::string avoidance_topic_, setpoint_topic_, auto_goal_mode_{"relative"};
    bool enable_control_{false}, auto_arm_{true}, auto_goal_{true};
    bool use_path_planner_{true};
    bool stop_on_level1_{true};
    bool have_mavros_pose_{false}, have_odom_{false}, have_goal_{false};
    bool have_risk_{false}, have_avoidance_{false};
    bool navigation_ready_sent_{false};
    bool final_goal_observed_{false};
    std::uint8_t risk_level_{0U};
    double relative_goal_x_{0.0}, relative_goal_y_{6.0};
    double absolute_goal_x_{0.0}, absolute_goal_y_{6.0}, absolute_goal_z_{1.5};
    double takeoff_height_{1.5}, home_z_{0.0}, target_z_{1.5};
    double prestream_seconds_{3.0}, goal_delay_seconds_{3.0};
    double position_kp_{0.45}, altitude_kp_{0.8};
    double max_navigation_speed_{0.65}, max_total_speed_{0.85}, max_vertical_speed_{0.5};
    double avoidance_gain_{1.0}, level1_navigation_scale_{0.15};
    double goal_tolerance_{0.25}, takeoff_tolerance_{0.12};
    double waypoint_tolerance_{0.03};
    double risk_timeout_{0.45}, odom_timeout_{0.35}, publish_rate_{30.0};
    Phase phase_{Phase::WAIT_DATA};
    ros::Time phase_start_, last_request_, mavros_pose_time_, odom_time_, risk_time_, avoidance_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_avoidance_controller");
    MavrosAvoidanceController controller;
    ros::spin();
    return 0;
}
