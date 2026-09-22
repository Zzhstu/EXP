#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <fastlio_bridge/DynamicObjectArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8.h>

#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

class SystemWatchdog
{
public:
    SystemWatchdog() : nh_(), pnh_("~")
    {
        pnh_.param("timeout", timeout_, 0.60);
        pnh_.param("startup_grace", startup_grace_, 5.0);
        pnh_.param("publish_rate", publish_rate_, 2.0);
        pnh_.param<std::string>("odom_topic", odom_topic_, "/uav1/fastlio/odom");
        pnh_.param<std::string>("cloud_topic", cloud_topic_,
                                "/uav1/fastlio/registered_scan");
        pnh_.param<std::string>("objects_topic", objects_topic_, "/uav1/dynamic_objects");
        pnh_.param<std::string>("risk_topic", risk_topic_, "/uav1/fused_collision_risk_level");
        pnh_.param<std::string>("velocity_topic", velocity_topic_, "/uav1/fused_avoidance_velocity");

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &SystemWatchdog::odomCallback, this);
        cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &SystemWatchdog::cloudCallback, this);
        objects_sub_ = nh_.subscribe(objects_topic_, 10, &SystemWatchdog::objectsCallback, this);
        risk_sub_ = nh_.subscribe(risk_topic_, 10, &SystemWatchdog::riskCallback, this);
        velocity_sub_ = nh_.subscribe(velocity_topic_, 10, &SystemWatchdog::velocityCallback, this);
        pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/uav1/dynamic_system/diagnostics", 2);
        start_time_ = ros::Time::now();
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.2, publish_rate_)),
                                 &SystemWatchdog::timerCallback, this);
    }

private:
    struct State
    {
        ros::Time last;
        std::uint64_t count{0};
        double hz{0.0};
        void update()
        {
            const ros::Time now = ros::Time::now();
            if (!last.isZero())
            {
                const double dt = (now - last).toSec();
                if (dt > 1e-4) hz = hz <= 0.0 ? 1.0 / dt : 0.9 * hz + 0.1 / dt;
            }
            last = now;
            ++count;
        }
    };

    static diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value)
    {
        diagnostic_msgs::KeyValue item;
        item.key = key;
        item.value = value;
        return item;
    }

    diagnostic_msgs::DiagnosticStatus makeStatus(
        const std::string& name, const std::string& topic, const State& state,
        const ros::Time& now, bool required) const
    {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "edge_uav_dynamic/" + name;
        status.hardware_id = "jetson_companion";
        const double age = state.last.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - state.last).toSec();
        const bool grace = (now - start_time_).toSec() < startup_grace_;
        if (state.last.isZero())
        {
            status.level = grace || !required
                ? diagnostic_msgs::DiagnosticStatus::WARN
                : diagnostic_msgs::DiagnosticStatus::ERROR;
            status.message = "no message received";
        }
        else if (age > timeout_)
        {
            status.level = required
                ? diagnostic_msgs::DiagnosticStatus::ERROR
                : diagnostic_msgs::DiagnosticStatus::WARN;
            status.message = "stale";
        }
        else
        {
            status.level = diagnostic_msgs::DiagnosticStatus::OK;
            status.message = "ok";
        }
        std::ostringstream age_stream, hz_stream;
        age_stream << std::fixed << std::setprecision(3) << age;
        hz_stream << std::fixed << std::setprecision(1) << state.hz;
        status.values.push_back(kv("topic", topic));
        status.values.push_back(kv("age_s", age_stream.str()));
        status.values.push_back(kv("estimated_hz", hz_stream.str()));
        status.values.push_back(kv("messages", std::to_string(state.count)));
        return status;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr&) { odom_.update(); }
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr&) { cloud_.update(); }
    void objectsCallback(const fastlio_bridge::DynamicObjectArray::ConstPtr&) { objects_.update(); }
    void riskCallback(const std_msgs::UInt8::ConstPtr&) { risk_.update(); }
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr&) { velocity_.update(); }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();
        diagnostic_msgs::DiagnosticArray output;
        output.header.stamp = now;
        output.status.push_back(makeStatus("odometry", odom_topic_, odom_, now, true));
        output.status.push_back(makeStatus("registered_cloud", cloud_topic_, cloud_, now, true));
        // Empty object arrays are valid, but the topic itself must remain alive.
        output.status.push_back(makeStatus("dynamic_objects", objects_topic_, objects_, now, true));
        output.status.push_back(makeStatus("fused_risk", risk_topic_, risk_, now, true));
        output.status.push_back(makeStatus("fused_velocity", velocity_topic_, velocity_, now, true));
        pub_.publish(output);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber odom_sub_, cloud_sub_, objects_sub_, risk_sub_, velocity_sub_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::string odom_topic_, cloud_topic_, objects_topic_, risk_topic_, velocity_topic_;
    double timeout_{0.60}, startup_grace_{5.0}, publish_rate_{2.0};
    ros::Time start_time_;
    State odom_, cloud_, objects_, risk_, velocity_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_system_watchdog");
    SystemWatchdog node;
    ros::spin();
    return 0;
}
