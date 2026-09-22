// Repeatable PX4-SITL flight for the FAST-LIO long-corridor benchmark.
// The route is expressed relative to the MAVROS local pose at startup:
// take off -> +X 60 m -> hold -> return -> land and disarm.

#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

namespace {
constexpr double kPublishHz = 20.0;

enum class Phase { PRESTREAM, ARMING, TAKEOFF, SETTLE, OUTBOUND, TURN_HOLD, RETURNING, LANDING, DONE, ABORT_HOLD };

const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::PRESTREAM: return "PRESTREAM";
    case Phase::ARMING: return "ARMING";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::SETTLE: return "SETTLE";
    case Phase::OUTBOUND: return "OUTBOUND";
    case Phase::TURN_HOLD: return "TURN_HOLD";
    case Phase::RETURNING: return "RETURNING";
    case Phase::LANDING: return "LANDING";
    case Phase::DONE: return "DONE";
    case Phase::ABORT_HOLD: return "ABORT_HOLD";
  }
  return "UNKNOWN";
}
}  // namespace

class LongCorridorFlight {
 public:
  LongCorridorFlight() : nh_(), pnh_("~") {
    pnh_.param("takeoff_height", takeoff_height_, 1.5);
    pnh_.param("corridor_distance", corridor_distance_, 60.0);
    pnh_.param("cruise_speed", cruise_speed_, 0.8);
    pnh_.param("settle_seconds", settle_seconds_, 5.0);
    pnh_.param("turn_hold_seconds", turn_hold_seconds_, 3.0);
    pnh_.param("position_tolerance", position_tolerance_, 0.30);
    pnh_.param("landing_tolerance", landing_tolerance_, 0.12);
    pnh_.param("max_tracking_error", max_tracking_error_, 3.0);
    pnh_.param("auto_arm", auto_arm_, true);
    pnh_.param("auto_land", auto_land_, true);

    state_sub_ = nh_.subscribe("mavros/state", 10, &LongCorridorFlight::stateCb, this);
    pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &LongCorridorFlight::poseCb, this);
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  }

  void run() {
    ros::Rate rate(kPublishHz);
    while (ros::ok() && (!state_.connected || !have_pose_)) {
      ROS_INFO_THROTTLE(2.0, "Waiting for FCU and local pose (connected=%s pose=%s)",
                        state_.connected ? "true" : "false", have_pose_ ? "true" : "false");
      ros::spinOnce();
      rate.sleep();
    }
    if (!ros::ok()) return;

    start_pose_ = current_pose_;
    target_ = start_pose_;
    target_.header.frame_id = "map";
    phase_ = Phase::PRESTREAM;
    phase_started_ = ros::Time::now();
    last_update_ = phase_started_;
    last_request_ = ros::Time(0);
    ROS_INFO("Corridor route: start=(%.2f, %.2f, %.2f), height=%.2f m, +X=%.2f m, speed=%.2f m/s",
             start_pose_.pose.position.x, start_pose_.pose.position.y, start_pose_.pose.position.z,
             takeoff_height_, corridor_distance_, cruise_speed_);

    while (ros::ok() && phase_ != Phase::DONE) {
      const ros::Time now = ros::Time::now();
      const double dt = std::min(0.20, std::max(0.0, (now - last_update_).toSec()));
      last_update_ = now;
      update(now, dt);
      target_.header.stamp = now;
      setpoint_pub_.publish(target_);  // Never stop the stream while in OFFBOARD.
      ROS_INFO_THROTTLE(1.0, "[%s] target=(%.2f, %.2f, %.2f), current=(%.2f, %.2f, %.2f)", phaseName(phase_),
                        target_.pose.position.x, target_.pose.position.y, target_.pose.position.z,
                        current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void stateCb(const mavros_msgs::State::ConstPtr& msg) { state_ = *msg; }
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) { current_pose_ = *msg; have_pose_ = true; }

  void setPhase(Phase next, const ros::Time& now) {
    ROS_INFO("Route phase: %s -> %s", phaseName(phase_), phaseName(next));
    phase_ = next;
    phase_started_ = now;
  }

  double distanceToTarget() const {
    const double dx = current_pose_.pose.position.x - target_.pose.position.x;
    const double dy = current_pose_.pose.position.y - target_.pose.position.y;
    const double dz = current_pose_.pose.position.z - target_.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void requestOffboardAndArm(const ros::Time& now) {
    if (!auto_arm_ || now - last_request_ < ros::Duration(2.0)) return;
    if (state_.mode != "OFFBOARD") {
      mavros_msgs::SetMode request;
      request.request.custom_mode = "OFFBOARD";
      if (mode_client_.call(request) && request.response.mode_sent) ROS_INFO("OFFBOARD mode requested");
    } else if (!state_.armed) {
      mavros_msgs::CommandBool request;
      request.request.value = true;
      if (arming_client_.call(request) && request.response.success) ROS_INFO("Vehicle arming requested");
    }
    last_request_ = now;
  }

  void update(const ros::Time& now, double dt) {
    if (phase_ == Phase::PRESTREAM) {
      target_ = start_pose_;
      if (now - phase_started_ >= ros::Duration(5.0)) setPhase(Phase::ARMING, now);
      return;
    }
    if (phase_ == Phase::ARMING) {
      target_ = start_pose_;
      requestOffboardAndArm(now);
      if (state_.mode == "OFFBOARD" && state_.armed) {
        target_.pose.position.z = start_pose_.pose.position.z + takeoff_height_;
        setPhase(Phase::TAKEOFF, now);
      }
      return;
    }
    if (phase_ == Phase::TAKEOFF) {
      target_ = start_pose_;
      target_.pose.position.z = start_pose_.pose.position.z + takeoff_height_;
      if (std::abs(current_pose_.pose.position.z - target_.pose.position.z) < position_tolerance_) setPhase(Phase::SETTLE, now);
      return;
    }
    if (phase_ == Phase::SETTLE) {
      if (now - phase_started_ >= ros::Duration(settle_seconds_)) setPhase(Phase::OUTBOUND, now);
      return;
    }
    if (phase_ == Phase::OUTBOUND || phase_ == Phase::RETURNING) {
      const double direction = phase_ == Phase::OUTBOUND ? 1.0 : -1.0;
      target_.pose.position.x += direction * cruise_speed_ * dt;
      const double end_x = start_pose_.pose.position.x + (phase_ == Phase::OUTBOUND ? corridor_distance_ : 0.0);
      if ((direction > 0.0 && target_.pose.position.x > end_x) || (direction < 0.0 && target_.pose.position.x < end_x)) target_.pose.position.x = end_x;
      target_.pose.position.y = start_pose_.pose.position.y;
      target_.pose.position.z = start_pose_.pose.position.z + takeoff_height_;

      // A failed controller or collision must not let the reference race away.
      if (distanceToTarget() > max_tracking_error_) {
        target_ = current_pose_;
        target_.header.frame_id = "map";
        setPhase(Phase::ABORT_HOLD, now);
        ROS_ERROR("Tracking error exceeded %.2f m; holding current position. No auto-land is sent.", max_tracking_error_);
        return;
      }
      if (std::abs(current_pose_.pose.position.x - end_x) < position_tolerance_ &&
          std::abs(target_.pose.position.x - end_x) < 1e-3) {
        setPhase(phase_ == Phase::OUTBOUND ? Phase::TURN_HOLD : Phase::LANDING, now);
      }
      return;
    }
    if (phase_ == Phase::TURN_HOLD) {
      if (now - phase_started_ >= ros::Duration(turn_hold_seconds_)) setPhase(Phase::RETURNING, now);
      return;
    }
    if (phase_ == Phase::LANDING) {
      target_.pose.position.x = start_pose_.pose.position.x;
      target_.pose.position.y = start_pose_.pose.position.y;
      target_.pose.position.z = start_pose_.pose.position.z;
      if (std::abs(current_pose_.pose.position.z - start_pose_.pose.position.z) < landing_tolerance_) {
        if (auto_land_ && state_.armed) {
          mavros_msgs::CommandBool request;
          request.request.value = false;
          if (arming_client_.call(request) && request.response.success) ROS_INFO("Landing complete; vehicle disarmed.");
        }
        setPhase(Phase::DONE, now);
      }
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber state_sub_, pose_sub_;
  ros::Publisher setpoint_pub_;
  ros::ServiceClient arming_client_, mode_client_;
  mavros_msgs::State state_;
  geometry_msgs::PoseStamped current_pose_, start_pose_, target_;
  bool have_pose_ = false, auto_arm_ = true, auto_land_ = true;
  double takeoff_height_, corridor_distance_, cruise_speed_, settle_seconds_, turn_hold_seconds_;
  double position_tolerance_, landing_tolerance_, max_tracking_error_;
  Phase phase_ = Phase::PRESTREAM;
  ros::Time phase_started_, last_update_, last_request_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "long_corridor_baseline_flight");
  LongCorridorFlight route;
  route.run();
  return 0;
}
