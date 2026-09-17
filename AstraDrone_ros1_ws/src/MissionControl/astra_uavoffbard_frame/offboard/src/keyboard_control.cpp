#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

class KeyboardControl {
public:
    KeyboardControl()
        : nh_(), pnh_("~"), key_ready_(false), initial_z_(0.0), target_z_(0.0),
          have_pose_(false), land_requested_(false) {
        pnh_.param("step", step_, 0.2);
        pnh_.param("takeoff_height", takeoff_height_, 1.0);
        pnh_.param("min_height", min_height_, 0.15);

        state_sub_ = nh_.subscribe("mavros/state", 10, &KeyboardControl::stateCb, this);
        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &KeyboardControl::poseCb, this);
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        old_terminal_ = new termios;
        tcgetattr(STDIN_FILENO, old_terminal_);
        termios raw = *old_terminal_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        key_ready_ = true;
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

        std::cout << "Keyboard control: w/s=x, a/d=y, r/f=up/down, t=takeoff, l=land, Ctrl-C=exit\n";
    }

    ~KeyboardControl() {
        if (key_ready_) {
            tcsetattr(STDIN_FILENO, TCSANOW, old_terminal_);
        }
        delete old_terminal_;
    }

    void run() {
        ros::Rate rate(20.0);
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        while (ros::ok() && !have_pose_) {
            ros::spinOnce();
            rate.sleep();
        }

        initial_z_ = current_pose_.pose.position.z;
        target_ = current_pose_;
        target_z_ = initial_z_;

        // PX4 requires a stream of setpoints before accepting OFFBOARD.
        for (int i = 0; ros::ok() && i < 100; ++i) {
            publishTarget();
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offboard;
        offboard.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm;
        arm.request.value = true;
        ros::Time last_request = ros::Time::now();

        while (ros::ok()) {
            readKeys();

            if (current_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (mode_client_.call(offboard) && offboard.response.mode_sent)
                    ROS_INFO("OFFBOARD enabled");
                last_request = ros::Time::now();
            } else if (!current_state_.armed &&
                       (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (arm_client_.call(arm) && arm.response.success)
                    ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }

            target_.header.stamp = ros::Time::now();
            target_.header.frame_id = "map";
            target_.pose.position.z = target_z_;
            publishTarget();

            if (land_requested_ && current_state_.armed &&
                std::abs(current_pose_.pose.position.z - initial_z_) < 0.10) {
                mavros_msgs::CommandBool disarm;
                disarm.request.value = false;
                if (arm_client_.call(disarm) && disarm.response.success) {
                    ROS_INFO("Landed and disarmed");
                    ros::shutdown();
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void stateCb(const mavros_msgs::State::ConstPtr& msg) { current_state_ = *msg; }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        if (!have_pose_) have_pose_ = true;
    }

    void publishTarget() { setpoint_pub_.publish(target_); }

    void readKeys() {
        char key;
        while (read(STDIN_FILENO, &key, 1) > 0) {
            switch (key) {
            case 'w': target_.pose.position.x += step_; break;
            case 's': target_.pose.position.x -= step_; break;
            case 'a': target_.pose.position.y += step_; break;
            case 'd': target_.pose.position.y -= step_; break;
            case 'r': target_z_ += step_; break;
            case 'f': target_z_ -= step_; break;
            case 't': target_z_ = initial_z_ + takeoff_height_; break;
            case 'l': target_z_ = initial_z_; land_requested_ = true; break;
            default: break;
            }
            target_z_ = std::max(initial_z_, target_z_);
            ROS_INFO_THROTTLE(1.0, "Target: x=%.2f y=%.2f z=%.2f",
                              target_.pose.position.x, target_.pose.position.y, target_z_);
        }
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber state_sub_, pose_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arm_client_, mode_client_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_, target_;
    double step_, takeoff_height_, min_height_, initial_z_, target_z_;
    bool key_ready_, have_pose_, land_requested_;
    termios* old_terminal_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    KeyboardControl controller;
    controller.run();
    return 0;
}
