#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <prometheus_msgs/UAVCommand.h>

#include <algorithm>
#include <cmath>
#include <string>

class GoalNavigation
{
public:
    GoalNavigation() : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/uav1/fastlio/odom");
        pnh_.param<std::string>("output_topic", output_topic_, "/uav1/navigation_command");
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param("position_kp", position_kp_, 0.60);
        pnh_.param("max_speed", max_speed_, 0.70);
        pnh_.param("goal_tolerance", goal_tolerance_, 0.20);
        pnh_.param("default_altitude", default_altitude_, 1.0);
        pnh_.param("publish_rate", publish_rate_, 20.0);

        goal_sub_ = nh_.subscribe(goal_topic_, 5, &GoalNavigation::goalCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &GoalNavigation::odomCallback, this);
        command_pub_ = nh_.advertise<prometheus_msgs::UAVCommand>(output_topic_, 10);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                                 &GoalNavigation::timerCallback, this);
        ROS_INFO("[GoalNavigation] RViz 3D Nav Goal -> %s; max_speed=%.2f",
                 output_topic_.c_str(), max_speed_);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
        have_odom_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != world_frame_ &&
            msg->header.frame_id != "/" + world_frame_)
        {
            ROS_ERROR("[GoalNavigation] goal frame '%s' must be '%s'.",
                      msg->header.frame_id.c_str(), world_frame_.c_str());
            return;
        }
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_z_ = msg->pose.position.z > 0.10
            ? msg->pose.position.z : default_altitude_;
        have_goal_ = true;
        ROS_INFO("[GoalNavigation] goal=(%.2f %.2f %.2f)", goal_x_, goal_y_, goal_z_);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!have_odom_ || !have_goal_) return;
        const double dx = goal_x_ - x_;
        const double dy = goal_y_ - y_;
        const double distance = std::hypot(dx, dy);
        double vx = position_kp_ * dx;
        double vy = position_kp_ * dy;
        const double speed = std::hypot(vx, vy);
        if (speed > max_speed_)
        {
            vx *= max_speed_ / speed;
            vy *= max_speed_ / speed;
        }
        if (distance <= goal_tolerance_) vx = vy = 0.0;

        prometheus_msgs::UAVCommand command;
        command.header.stamp = ros::Time::now();
        command.header.frame_id = world_frame_;
        command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        command.Control_Level = prometheus_msgs::UAVCommand::DEFAULT_CONTROL;
        command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS;
        command.position_ref[0] = static_cast<float>(goal_x_);
        command.position_ref[1] = static_cast<float>(goal_y_);
        command.position_ref[2] = static_cast<float>(goal_z_);
        command.velocity_ref[0] = static_cast<float>(vx);
        command.velocity_ref[1] = static_cast<float>(vy);
        command.velocity_ref[2] = 0.0F;
        command.yaw_ref = 0.0F;
        command.Yaw_Rate_Mode = false;
        command_pub_.publish(command);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber goal_sub_, odom_sub_;
    ros::Publisher command_pub_;
    ros::Timer timer_;
    std::string goal_topic_, odom_topic_, output_topic_, world_frame_;
    double position_kp_{0.60}, max_speed_{0.70}, goal_tolerance_{0.20};
    double default_altitude_{1.0}, publish_rate_{20.0};
    double x_{0.0}, y_{0.0}, z_{0.0};
    double goal_x_{0.0}, goal_y_{0.0}, goal_z_{1.0};
    bool have_odom_{false}, have_goal_{false};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_navigation");
    GoalNavigation node;
    ros::spin();
    return 0;
}

