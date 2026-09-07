#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>


class AvoidanceFusion
{
public:
    enum RiskLevel : std::uint8_t
    {
        SAFE = 0,
        AVOID = 1,
        EMERGENCY = 2
    };

    AvoidanceFusion()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "static_risk_topic",
            static_risk_topic_,
            "/uav1/static_collision_risk_level");
        pnh_.param<std::string>(
            "static_velocity_topic",
            static_velocity_topic_,
            "/uav1/static_avoidance_velocity");
        pnh_.param<std::string>(
            "dynamic_risk_topic",
            dynamic_risk_topic_,
            "/uav1/collision_risk_level");
        pnh_.param<std::string>(
            "dynamic_velocity_topic",
            dynamic_velocity_topic_,
            "/uav1/avoidance_velocity");
        pnh_.param<std::string>("world_frame", world_frame_, "map");

        pnh_.param("source_timeout", source_timeout_, 0.40);
        pnh_.param("max_avoid_speed", max_avoid_speed_, 1.00);
        pnh_.param("velocity_alpha", velocity_alpha_, 0.25);
        pnh_.param("static_emergency_weight", static_emergency_weight_, 0.70);
        pnh_.param("dynamic_emergency_weight", dynamic_emergency_weight_, 0.30);

        source_timeout_ = std::max(0.05, source_timeout_);
        max_avoid_speed_ = std::max(0.05, max_avoid_speed_);
        velocity_alpha_ = std::max(
            0.0,
            std::min(1.0, velocity_alpha_));
        static_emergency_weight_ =
            std::max(0.0, static_emergency_weight_);
        dynamic_emergency_weight_ =
            std::max(0.0, dynamic_emergency_weight_);

        static_risk_sub_ = nh_.subscribe(
            static_risk_topic_, 10, &AvoidanceFusion::staticRiskCallback, this);
        static_velocity_sub_ = nh_.subscribe(
            static_velocity_topic_,
            10,
            &AvoidanceFusion::staticVelocityCallback,
            this);
        dynamic_risk_sub_ = nh_.subscribe(
            dynamic_risk_topic_, 10, &AvoidanceFusion::dynamicRiskCallback, this);
        dynamic_velocity_sub_ = nh_.subscribe(
            dynamic_velocity_topic_,
            10,
            &AvoidanceFusion::dynamicVelocityCallback,
            this);

        fused_risk_pub_ = nh_.advertise<std_msgs::UInt8>(
            "/uav1/fused_collision_risk_level", 10);
        fused_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            "/uav1/fused_avoidance_velocity", 10);
        fused_source_pub_ = nh_.advertise<std_msgs::String>(
            "/uav1/fused_avoidance_source", 10);

        timer_ = nh_.createTimer(
            ros::Duration(0.05), &AvoidanceFusion::timerCallback, this);

        ROS_INFO(
            "[AvoidanceFusion] Started. timeout=%.2f max_speed=%.2f "
            "emergency_weights=(static=%.2f dynamic=%.2f)",
            source_timeout_,
            max_avoid_speed_,
            static_emergency_weight_,
            dynamic_emergency_weight_);
    }

private:
    struct SourceState
    {
        RiskLevel level{SAFE};
        double vx{0.0};
        double vy{0.0};
        bool have_risk{false};
        bool have_velocity{false};
        ros::Time risk_receive_time;
        ros::Time velocity_receive_time;
    };

    static RiskLevel toRiskLevel(const std::uint8_t value)
    {
        return value >= EMERGENCY
            ? EMERGENCY
            : static_cast<RiskLevel>(value);
    }

    bool isFresh(const SourceState& source, const ros::Time& now) const
    {
        return source.have_risk &&
               source.have_velocity &&
               (now - source.risk_receive_time).toSec() <= source_timeout_ &&
               (now - source.velocity_receive_time).toSec() <= source_timeout_;
    }

    static void limitMagnitude(
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

    void staticRiskCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        static_source_.level = toRiskLevel(msg->data);
        static_source_.risk_receive_time = ros::Time::now();
        static_source_.have_risk = true;
    }

    void staticVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        static_source_.vx = msg->twist.linear.x;
        static_source_.vy = msg->twist.linear.y;
        static_source_.velocity_receive_time = ros::Time::now();
        static_source_.have_velocity = true;
    }

    void dynamicRiskCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        dynamic_source_.level = toRiskLevel(msg->data);
        dynamic_source_.risk_receive_time = ros::Time::now();
        dynamic_source_.have_risk = true;
    }

    void dynamicVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        dynamic_source_.vx = msg->twist.linear.x;
        dynamic_source_.vy = msg->twist.linear.y;
        dynamic_source_.velocity_receive_time = ros::Time::now();
        dynamic_source_.have_velocity = true;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        const bool static_fresh = isFresh(static_source_, now);
        const bool dynamic_fresh = isFresh(dynamic_source_, now);

        const RiskLevel static_level = static_fresh
            ? static_source_.level
            : SAFE;
        const RiskLevel dynamic_level = dynamic_fresh
            ? dynamic_source_.level
            : SAFE;

        RiskLevel fused_level = std::max(static_level, dynamic_level);
        double raw_vx = 0.0;
        double raw_vy = 0.0;
        std::string source = "none";

        if (static_level == EMERGENCY && dynamic_level == EMERGENCY)
        {
            raw_vx = static_emergency_weight_ * static_source_.vx +
                     dynamic_emergency_weight_ * dynamic_source_.vx;
            raw_vy = static_emergency_weight_ * static_source_.vy +
                     dynamic_emergency_weight_ * dynamic_source_.vy;

            // 两个紧急方向几乎抵消时，优先选择瞬时净空的静态避障方向。
            if (std::hypot(raw_vx, raw_vy) < 0.05)
            {
                raw_vx = static_source_.vx;
                raw_vy = static_source_.vy;
                source = "static_emergency_priority";
            }
            else
            {
                source = "static+dynamic_emergency";
            }
        }
        else if (static_level == EMERGENCY)
        {
            raw_vx = static_source_.vx;
            raw_vy = static_source_.vy;
            source = "static_emergency";
        }
        else if (dynamic_level == EMERGENCY)
        {
            raw_vx = dynamic_source_.vx;
            raw_vy = dynamic_source_.vy;
            source = "dynamic_emergency";
        }
        else if (static_level == AVOID && dynamic_level == AVOID)
        {
            raw_vx = static_source_.vx + dynamic_source_.vx;
            raw_vy = static_source_.vy + dynamic_source_.vy;
            source = "static+dynamic_avoid";
        }
        else if (static_level == AVOID)
        {
            raw_vx = static_source_.vx;
            raw_vy = static_source_.vy;
            source = "static_avoid";
        }
        else if (dynamic_level == AVOID)
        {
            raw_vx = dynamic_source_.vx;
            raw_vy = dynamic_source_.vy;
            source = "dynamic_avoid";
        }

        limitMagnitude(raw_vx, raw_vy, max_avoid_speed_);

        const double alpha = fused_level == EMERGENCY
            ? std::max(0.60, velocity_alpha_)
            : velocity_alpha_;

        filtered_vx_ = (1.0 - alpha) * filtered_vx_ + alpha * raw_vx;
        filtered_vy_ = (1.0 - alpha) * filtered_vy_ + alpha * raw_vy;

        if (fused_level == SAFE)
        {
            filtered_vx_ = 0.0;
            filtered_vy_ = 0.0;
        }

        limitMagnitude(filtered_vx_, filtered_vy_, max_avoid_speed_);

        std_msgs::UInt8 risk;
        risk.data = fused_level;
        fused_risk_pub_.publish(risk);

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = now;
        velocity.header.frame_id = world_frame_;
        velocity.twist.linear.x = filtered_vx_;
        velocity.twist.linear.y = filtered_vy_;
        velocity.twist.linear.z = 0.0;
        fused_velocity_pub_.publish(velocity);

        std_msgs::String source_message;
        source_message.data = source;
        fused_source_pub_.publish(source_message);

        ROS_INFO_THROTTLE(
            0.25,
            "[AvoidanceFusion] level=%u static=%u dynamic=%u source=%s "
            "avoid=(%.2f %.2f)",
            static_cast<unsigned int>(fused_level),
            static_cast<unsigned int>(static_level),
            static_cast<unsigned int>(dynamic_level),
            source.c_str(),
            filtered_vx_,
            filtered_vy_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber static_risk_sub_;
    ros::Subscriber static_velocity_sub_;
    ros::Subscriber dynamic_risk_sub_;
    ros::Subscriber dynamic_velocity_sub_;
    ros::Publisher fused_risk_pub_;
    ros::Publisher fused_velocity_pub_;
    ros::Publisher fused_source_pub_;
    ros::Timer timer_;

    std::string static_risk_topic_;
    std::string static_velocity_topic_;
    std::string dynamic_risk_topic_;
    std::string dynamic_velocity_topic_;
    std::string world_frame_;

    double source_timeout_{0.40};
    double max_avoid_speed_{1.00};
    double velocity_alpha_{0.25};
    double static_emergency_weight_{0.70};
    double dynamic_emergency_weight_{0.30};

    SourceState static_source_;
    SourceState dynamic_source_;
    double filtered_vx_{0.0};
    double filtered_vy_{0.0};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidance_fusion");
    AvoidanceFusion node;
    ros::spin();
    return 0;
}
