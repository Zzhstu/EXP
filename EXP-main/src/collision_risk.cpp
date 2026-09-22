#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>


class CollisionRisk
{
public:
    enum RiskLevel : std::uint8_t
    {
        SAFE = 0,
        AVOID = 1,
        EMERGENCY = 2
    };

    CollisionRisk()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "objects_topic",
            objects_topic_,
            "/uav1/dynamic_objects");

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/uav1/fastlio/odom");

        pnh_.param("prediction_horizon", prediction_horizon_, 3.0);
        pnh_.param("safe_radius", safe_radius_, 1.5);
        pnh_.param("vertical_safe_distance", vertical_safe_distance_, 1.0);
        pnh_.param("emergency_distance", emergency_distance_, 0.8);
        pnh_.param(
            "emergency_time_to_cpa", emergency_time_to_cpa_, 1.20);
        pnh_.param(
            "emergency_cpa_distance", emergency_cpa_distance_, 0.60);
        pnh_.param("emergency_hold_time", emergency_hold_time_, 0.50);
        pnh_.param("max_avoid_speed", max_avoid_speed_, 1.0);
        pnh_.param("min_relative_speed", min_relative_speed_, 0.05);
        pnh_.param("objects_timeout", objects_timeout_, 0.30);
        pnh_.param("odom_timeout", odom_timeout_, 0.30);
        pnh_.param("velocity_alpha", velocity_alpha_, 0.30);
        pnh_.param("risk_exit_radius", risk_exit_radius_, 1.80);
        pnh_.param("risk_release_time", risk_release_time_, 0.50);
        pnh_.param("target_hold_time", target_hold_time_, 0.60);
        pnh_.param("avoidance_alpha", avoidance_alpha_, 0.20);
        pnh_.param("direction_alpha", direction_alpha_, 0.15);
        pnh_.param("min_avoid_speed", min_avoid_speed_, 0.30);

        risk_exit_radius_ = std::max(risk_exit_radius_, safe_radius_);
        emergency_distance_ = std::max(0.0, emergency_distance_);
        emergency_time_to_cpa_ = std::max(
            0.0,
            std::min(emergency_time_to_cpa_, prediction_horizon_));
        emergency_cpa_distance_ = std::max(
            0.0,
            std::min(emergency_cpa_distance_, safe_radius_));
        emergency_hold_time_ = std::max(0.0, emergency_hold_time_);
        max_avoid_speed_ = std::max(0.05, max_avoid_speed_);
        min_avoid_speed_ = std::max(
            0.0,
            std::min(min_avoid_speed_, max_avoid_speed_));
        avoidance_alpha_ = std::max(
            0.0,
            std::min(1.0, avoidance_alpha_));
        direction_alpha_ = std::max(
            0.0,
            std::min(1.0, direction_alpha_));

        objects_sub_ = nh_.subscribe(
            objects_topic_,
            10,
            &CollisionRisk::objectsCallback,
            this);

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &CollisionRisk::odomCallback,
            this);

        risk_pub_ = nh_.advertise<std_msgs::UInt8>(
            "/uav1/collision_risk_level",
            10);

        avoidance_velocity_pub_ =
            nh_.advertise<geometry_msgs::TwistStamped>(
                "/uav1/avoidance_velocity",
                10);

        dangerous_object_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(
                "/uav1/most_dangerous_object",
                10);

        timer_ = nh_.createTimer(
            ros::Duration(0.05),
            &CollisionRisk::timerCallback,
            this);

        ROS_INFO(
            "[CollisionRisk] Started. horizon=%.2f safe=%.2f "
            "vertical=%.2f emergency=%.2f exit=%.2f "
            "emergency_cpa=(t<=%.2f d<%.2f) hold=%.2f "
            "max_avoid_speed=%.2f",
            prediction_horizon_,
            safe_radius_,
            vertical_safe_distance_,
            emergency_distance_,
            risk_exit_radius_,
            emergency_time_to_cpa_,
            emergency_cpa_distance_,
            emergency_hold_time_,
            max_avoid_speed_);
    }

private:
    struct Evaluation
    {
        bool threatening{false};
        bool emergency{false};
        bool current_distance_emergency{false};
        bool predicted_cpa_emergency{false};
        bool within_exit_zone{false};
        int id{-1};
        double raw_time_to_cpa{0.0};
        double time_to_cpa{0.0};
        double distance_at_cpa{std::numeric_limits<double>::infinity()};
        double current_distance{std::numeric_limits<double>::infinity()};
        double score{-1.0};
        Eigen::Vector2d avoidance_direction{Eigen::Vector2d::Zero()};
        Eigen::Vector3d object_position{Eigen::Vector3d::Zero()};
    };

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const Eigen::Vector3d current_position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        if (have_odom_)
        {
            const double dt =
                (msg->header.stamp - odom_stamp_).toSec();

            if (dt > 0.005 && dt < 0.5)
            {
                const Eigen::Vector3d measured_velocity =
                    (current_position - uav_position_) / dt;

                if (measured_velocity.norm() < 10.0)
                {
                    uav_velocity_ =
                        (1.0 - velocity_alpha_) * uav_velocity_ +
                        velocity_alpha_ * measured_velocity;
                }
            }
        }

        uav_position_ = current_position;
        odom_stamp_ = msg->header.stamp;
        odom_receive_time_ = ros::Time::now();
        have_odom_ = true;
    }

    void objectsCallback(
        const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        latest_objects_ = *msg;
        objects_stamp_ = msg->header.stamp;
        objects_receive_time_ = ros::Time::now();
        have_objects_message_ = true;
    }

    Evaluation evaluateObject(
        const fastlio_bridge::DynamicObject& object) const
    {
        Evaluation result;
        result.id = object.id;

        result.object_position = Eigen::Vector3d(
            object.pose.position.x,
            object.pose.position.y,
            object.pose.position.z);

        const Eigen::Vector3d object_velocity(
            object.twist.linear.x,
            object.twist.linear.y,
            object.twist.linear.z);

        // r 表示从无人机指向障碍物。
        const Eigen::Vector3d relative_position =
            result.object_position - uav_position_;

        const Eigen::Vector3d relative_velocity =
            object_velocity - uav_velocity_;

        const Eigen::Vector2d r_xy = relative_position.head<2>();
        const Eigen::Vector2d v_xy = relative_velocity.head<2>();

        result.current_distance = r_xy.norm();

        const double relative_speed_sq = v_xy.squaredNorm();

        if (relative_speed_sq >
            min_relative_speed_ * min_relative_speed_)
        {
            result.raw_time_to_cpa =
                -r_xy.dot(v_xy) / relative_speed_sq;
        }
        else
        {
            result.raw_time_to_cpa = 0.0;
        }

        result.time_to_cpa = std::max(
            0.0,
            std::min(prediction_horizon_, result.raw_time_to_cpa));

        const Eigen::Vector2d closest_relative_xy =
            r_xy + v_xy * result.time_to_cpa;

        const double closest_relative_z =
            relative_position.z() +
            relative_velocity.z() * result.time_to_cpa;

        result.distance_at_cpa = closest_relative_xy.norm();

        const bool approaching =
            r_xy.dot(v_xy) < 0.0;

        const bool horizontally_unsafe =
            result.distance_at_cpa < safe_radius_;

        const bool vertically_unsafe =
            std::fabs(closest_relative_z) <
            vertical_safe_distance_;

        result.threatening =
            approaching &&
            result.time_to_cpa <= prediction_horizon_ &&
            horizontally_unsafe &&
            vertically_unsafe;

        result.current_distance_emergency =
            result.current_distance < emergency_distance_ &&
            std::fabs(relative_position.z()) <
                vertical_safe_distance_;

        // 目标还没有进入当前距离紧急区，但很快会以很小 CPA 距离
        // 接近时，也必须提前升级为 level=2。
        result.predicted_cpa_emergency =
            approaching &&
            result.raw_time_to_cpa >= 0.0 &&
            result.time_to_cpa <= emergency_time_to_cpa_ &&
            result.distance_at_cpa < emergency_cpa_distance_ &&
            vertically_unsafe;

        result.emergency =
            result.current_distance_emergency ||
            result.predicted_cpa_emergency;

        // 退出阈值大于进入阈值，避免风险等级在边界反复跳变。
        result.within_exit_zone =
            approaching &&
            result.distance_at_cpa < risk_exit_radius_ &&
            vertically_unsafe;

        if (!result.threatening &&
            !result.emergency &&
            !result.within_exit_zone)
        {
            return result;
        }

        // 远离预测最近接近点。若两者几乎重合，选择相对速度的法向。
        if (closest_relative_xy.norm() > 1e-3)
        {
            result.avoidance_direction =
                -closest_relative_xy.normalized();
        }
        else if (v_xy.norm() > 1e-3)
        {
            result.avoidance_direction =
                Eigen::Vector2d(-v_xy.y(), v_xy.x()).normalized();
        }
        else if (r_xy.norm() > 1e-3)
        {
            result.avoidance_direction = -r_xy.normalized();
        }
        else
        {
            result.avoidance_direction = Eigen::Vector2d(1.0, 0.0);
        }

        const double distance_score = std::max(
            0.0,
            1.0 - result.distance_at_cpa / safe_radius_);

        const double time_score = std::max(
            0.0,
            1.0 - result.time_to_cpa / prediction_horizon_);

        result.score = 0.7 * distance_score + 0.3 * time_score;

        if (result.emergency)
        {
            result.score += 1.0;
        }

        return result;
    }

    void publishSafe(const ros::Time& stamp)
    {
        std_msgs::UInt8 risk;
        risk.data = SAFE;
        risk_pub_.publish(risk);

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = stamp;
        velocity.header.frame_id = "map";
        avoidance_velocity_pub_.publish(velocity);
    }

    void resetDecisionState()
    {
        risk_active_ = false;
        locked_target_id_ = -1;
        locked_direction_.setZero();
        filtered_avoidance_velocity_.setZero();
        locked_object_position_.setZero();
        last_level_ = SAFE;
        last_emergency_time_ = ros::Time(0);
    }

    void publishDecision(
        const RiskLevel level,
        const ros::Time& stamp,
        const Eigen::Vector2d& velocity,
        const Eigen::Vector3d& object_position)
    {
        std_msgs::UInt8 risk;
        risk.data = level;
        risk_pub_.publish(risk);

        geometry_msgs::TwistStamped avoidance_velocity;
        avoidance_velocity.header.stamp = stamp;
        avoidance_velocity.header.frame_id = "map";
        avoidance_velocity.twist.linear.x = velocity.x();
        avoidance_velocity.twist.linear.y = velocity.y();
        avoidance_velocity.twist.linear.z = 0.0;
        avoidance_velocity_pub_.publish(avoidance_velocity);

        geometry_msgs::PoseStamped dangerous_object;
        dangerous_object.header.stamp = stamp;
        dangerous_object.header.frame_id = "map";
        dangerous_object.pose.position.x = object_position.x();
        dangerous_object.pose.position.y = object_position.y();
        dangerous_object.pose.position.z = object_position.z();
        dangerous_object.pose.orientation.w = 1.0;
        dangerous_object_pub_.publish(dangerous_object);
    }

    void acquireOrUpdateDirection(const Evaluation& evaluation)
    {
        Eigen::Vector2d candidate =
            evaluation.avoidance_direction;

        if (candidate.norm() < 1e-6)
        {
            return;
        }

        candidate.normalize();

        if (locked_target_id_ != evaluation.id ||
            locked_direction_.norm() < 1e-6)
        {
            // 切换目标时，尽量保持当前速度所在的半平面，
            // 避免目标ID变化造成左右方向瞬间反转。
            if (filtered_avoidance_velocity_.norm() > 1e-3 &&
                candidate.dot(filtered_avoidance_velocity_) < 0.0)
            {
                candidate = -candidate;
            }

            locked_target_id_ = evaluation.id;
            locked_direction_ = candidate;
            return;
        }

        // 同一目标不允许候选方向翻转到另一侧。
        if (candidate.dot(locked_direction_) < 0.0)
        {
            candidate = -candidate;
        }

        Eigen::Vector2d blended =
            (1.0 - direction_alpha_) * locked_direction_ +
            direction_alpha_ * candidate;

        if (blended.norm() > 1e-6)
        {
            locked_direction_ = blended.normalized();
        }
    }

    void publishHeldDecision(const ros::Time& now)
    {
        const RiskLevel held_level =
            last_level_ == EMERGENCY ? EMERGENCY : AVOID;

        publishDecision(
            held_level,
            now,
            filtered_avoidance_velocity_,
            locked_object_position_);

        ROS_INFO_THROTTLE(
            0.25,
            "[CollisionRisk] hold=1 level=%u id=%d avoid=(%.2f %.2f)",
            static_cast<unsigned int>(held_level),
            locked_target_id_,
            filtered_avoidance_velocity_.x(),
            filtered_avoidance_velocity_.y());
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        if (!have_odom_ ||
            (now - odom_receive_time_).toSec() > odom_timeout_)
        {
            resetDecisionState();
            publishSafe(now);

            ROS_WARN_THROTTLE(
                1.0,
                "[CollisionRisk] Odometry missing or stale; "
                "no control command is generated.");
            return;
        }

        if (!have_objects_message_ ||
            (now - objects_receive_time_).toSec() > objects_timeout_)
        {
            resetDecisionState();
            publishSafe(now);

            ROS_WARN_THROTTLE(
                1.0,
                "[CollisionRisk] Dynamic-object message missing or stale.");
            return;
        }

        Evaluation best_new_threat;
        Evaluation locked_evaluation;
        bool have_locked_evaluation = false;

        for (const auto& object : latest_objects_.objects)
        {
            const Evaluation evaluation = evaluateObject(object);

            if (evaluation.id == locked_target_id_ &&
                (evaluation.threatening ||
                 evaluation.emergency ||
                 evaluation.within_exit_zone))
            {
                locked_evaluation = evaluation;
                have_locked_evaluation = true;
            }

            if ((evaluation.threatening || evaluation.emergency) &&
                evaluation.score > best_new_threat.score)
            {
                best_new_threat = evaluation;
            }
        }

        Evaluation selected;
        bool have_selected = false;

        // 新出现的紧急目标允许立即打断原目标锁定。
        if (best_new_threat.emergency &&
            best_new_threat.id != locked_target_id_)
        {
            selected = best_new_threat;
            have_selected = true;
        }
        else if (risk_active_ && have_locked_evaluation)
        {
            selected = locked_evaluation;
            have_selected = true;
            locked_last_seen_time_ = now;
        }
        else if (!risk_active_ && best_new_threat.score >= 0.0)
        {
            selected = best_new_threat;
            have_selected = true;
        }
        else if (risk_active_ && !have_locked_evaluation)
        {
            const double target_missing_time =
                (now - locked_last_seen_time_).toSec();

            if (target_missing_time <= target_hold_time_)
            {
                publishHeldDecision(now);
                return;
            }

            if (best_new_threat.score >= 0.0)
            {
                selected = best_new_threat;
                have_selected = true;
            }
        }

        if (!have_selected)
        {
            if (risk_active_ &&
                (now - last_active_time_).toSec() <=
                    risk_release_time_)
            {
                publishHeldDecision(now);
                return;
            }

            resetDecisionState();
            publishSafe(now);
            return;
        }

        const bool continue_emergency =
            risk_active_ &&
            last_level_ == EMERGENCY &&
            selected.id == locked_target_id_ &&
            !last_emergency_time_.isZero() &&
            (now - last_emergency_time_).toSec() <=
                emergency_hold_time_;

        acquireOrUpdateDirection(selected);

        risk_active_ = true;
        locked_last_seen_time_ = now;
        last_active_time_ = now;
        locked_object_position_ = selected.object_position;

        if (selected.emergency)
        {
            last_emergency_time_ = now;
        }

        const RiskLevel level =
            (selected.emergency || continue_emergency)
                ? EMERGENCY
                : AVOID;

        double speed =
            min_avoid_speed_ +
            std::min(1.0, std::max(0.0, selected.score)) *
                (max_avoid_speed_ - min_avoid_speed_);

        if (level == EMERGENCY)
        {
            speed = max_avoid_speed_;
        }

        const Eigen::Vector2d raw_avoidance_velocity =
            speed * locked_direction_;

        const double alpha =
            level == EMERGENCY
                ? std::max(0.50, avoidance_alpha_)
                : avoidance_alpha_;

        filtered_avoidance_velocity_ =
            (1.0 - alpha) * filtered_avoidance_velocity_ +
            alpha * raw_avoidance_velocity;

        if (filtered_avoidance_velocity_.norm() > max_avoid_speed_)
        {
            filtered_avoidance_velocity_ =
                max_avoid_speed_ *
                filtered_avoidance_velocity_.normalized();
        }

        last_level_ = level;

        publishDecision(
            level,
            now,
            filtered_avoidance_velocity_,
            selected.object_position);

        ROS_INFO_THROTTLE(
            0.25,
            "[CollisionRisk] level=%u id=%d current=%.2f "
            "t_raw=%.2f t_cpa=%.2f d_cpa=%.2f "
            "e_now=%d e_cpa=%d avoid=(%.2f %.2f)",
            static_cast<unsigned int>(level),
            selected.id,
            selected.current_distance,
            selected.raw_time_to_cpa,
            selected.time_to_cpa,
            selected.distance_at_cpa,
            selected.current_distance_emergency ? 1 : 0,
            selected.predicted_cpa_emergency ? 1 : 0,
            filtered_avoidance_velocity_.x(),
            filtered_avoidance_velocity_.y());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber objects_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher risk_pub_;
    ros::Publisher avoidance_velocity_pub_;
    ros::Publisher dangerous_object_pub_;
    ros::Timer timer_;

    std::string objects_topic_;
    std::string odom_topic_;

    double prediction_horizon_{3.0};
    double safe_radius_{1.5};
    double vertical_safe_distance_{1.0};
    double emergency_distance_{0.8};
    double emergency_time_to_cpa_{1.20};
    double emergency_cpa_distance_{0.60};
    double emergency_hold_time_{0.50};
    double max_avoid_speed_{1.0};
    double min_relative_speed_{0.05};
    double objects_timeout_{0.30};
    double odom_timeout_{0.30};
    double velocity_alpha_{0.30};
    double risk_exit_radius_{1.80};
    double risk_release_time_{0.50};
    double target_hold_time_{0.60};
    double avoidance_alpha_{0.20};
    double direction_alpha_{0.15};
    double min_avoid_speed_{0.30};

    bool have_odom_{false};
    bool have_objects_message_{false};

    ros::Time odom_stamp_;
    ros::Time objects_stamp_;
    ros::Time odom_receive_time_;
    ros::Time objects_receive_time_;
    ros::Time locked_last_seen_time_;
    ros::Time last_active_time_;
    ros::Time last_emergency_time_;

    Eigen::Vector3d uav_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d uav_velocity_{Eigen::Vector3d::Zero()};

    bool risk_active_{false};
    int locked_target_id_{-1};
    RiskLevel last_level_{SAFE};
    Eigen::Vector2d locked_direction_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d filtered_avoidance_velocity_{
        Eigen::Vector2d::Zero()};
    Eigen::Vector3d locked_object_position_{
        Eigen::Vector3d::Zero()};

    fastlio_bridge::DynamicObjectArray latest_objects_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_risk");
    CollisionRisk node;
    ros::spin();
    return 0;
}
