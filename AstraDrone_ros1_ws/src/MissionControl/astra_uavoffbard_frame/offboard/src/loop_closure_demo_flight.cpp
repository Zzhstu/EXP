// Repeatable PX4-SITL route for evaluating loop closure.
// The vehicle follows a 10 m square and revisits its exact takeoff XY pose.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace {
constexpr double kPublishHz = 20.0;

enum class Phase { PRESTREAM, ARMING, TAKEOFF, SETTLE, ROUTE, CORNER_HOLD, FINAL_HOLD, LANDING, DONE, ABORT_HOLD };

const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::PRESTREAM: return "PRESTREAM";
    case Phase::ARMING: return "ARMING";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::SETTLE: return "SETTLE";
    case Phase::ROUTE: return "ROUTE";
    case Phase::CORNER_HOLD: return "CORNER_HOLD";
    case Phase::FINAL_HOLD: return "FINAL_HOLD";
    case Phase::LANDING: return "LANDING";
    case Phase::DONE: return "DONE";
    case Phase::ABORT_HOLD: return "ABORT_HOLD";
  }
  return "UNKNOWN";
}
}  // namespace

class LoopClosureDemoFlight {
 public:
  LoopClosureDemoFlight() : nh_(), pnh_("~") {
    pnh_.param("takeoff_height", takeoff_height_, 2.5);
    pnh_.param("side_length", side_length_, 10.0);
    pnh_.param("cruise_speed", cruise_speed_, 0.75);
    pnh_.param("settle_seconds", settle_seconds_, 5.0);
    pnh_.param("corner_hold_seconds", corner_hold_seconds_, 1.5);
    // Keep scanning at the revisited pose so the backend gets several full MID360 sweeps.
    pnh_.param("final_hold_seconds", final_hold_seconds_, 12.0);
    pnh_.param("position_tolerance", position_tolerance_, 0.30);
    pnh_.param("landing_tolerance", landing_tolerance_, 0.15);
    pnh_.param("max_tracking_error", max_tracking_error_, 2.5);
    pnh_.param("auto_arm", auto_arm_, true);
    pnh_.param("auto_land", auto_land_, true);

    state_sub_ = nh_.subscribe("mavros/state", 10, &LoopClosureDemoFlight::stateCb, this);
    pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &LoopClosureDemoFlight::poseCb, this);
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    finished_pub_ = nh_.advertise<std_msgs::Bool>("loop_closure_demo/finished", 1, true);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  }

  void run() {
    ros::Rate rate(kPublishHz);
    while (ros::ok() && (!state_.connected || !have_pose_)) {
      ROS_INFO_THROTTLE(2.0, "Waiting for PX4 and local pose (FCU=%s pose=%s)",
                        state_.connected ? "ready" : "waiting", have_pose_ ? "ready" : "waiting");
      ros::spinOnce();
      rate.sleep();
    }
    if (!ros::ok()) return;

    start_pose_ = current_pose_;
    target_ = start_pose_;
    target_.header.frame_id = "map";
    const double z = start_pose_.pose.position.z + takeoff_height_;
    // Clockwise square. Keeping yaw fixed gives a hard 360-degree revisit test
    // without making the flight controller rotate at every corner.
    addWaypoint(0.0, 0.0, z);
    addWaypoint(side_length_, 0.0, z);
    addWaypoint(side_length_, side_length_, z);
    addWaypoint(0.0, side_length_, z);
    addWaypoint(0.0, 0.0, z);

    const ros::Time now = ros::Time::now();
    phase_started_ = now;
    last_update_ = now;
    ROS_INFO("Loop route: start=(%.2f, %.2f), side=%.1f m, altitude=%.1f m, speed=%.2f m/s",
             start_pose_.pose.position.x, start_pose_.pose.position.y,
             side_length_, takeoff_height_, cruise_speed_);

    while (ros::ok() && phase_ != Phase::DONE) {
      const ros::Time stamp = ros::Time::now();
      const double dt = std::min(0.20, std::max(0.0, (stamp - last_update_).toSec()));
      last_update_ = stamp;
      update(stamp, dt);
      target_.header.stamp = stamp;
      setpoint_pub_.publish(target_);  // OFFBOARD requires a continuous stream.
      ROS_INFO_THROTTLE(1.0, "[%s %zu/%zu] target=(%.2f %.2f %.2f), pose=(%.2f %.2f %.2f)",
                        phaseName(phase_), waypoint_index_, waypoints_.size() - 1,
                        target_.pose.position.x, target_.pose.position.y, target_.pose.position.z,
                        current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void stateCb(const mavros_msgs::State::ConstPtr& msg) { state_ = *msg; }
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) { current_pose_ = *msg; have_pose_ = true; }

  void addWaypoint(double dx, double dy, double z) {
    geometry_msgs::PoseStamped pose = start_pose_;
    pose.header.frame_id = "map";
    pose.pose.position.x += dx;
    pose.pose.position.y += dy;
    pose.pose.position.z = z;
    waypoints_.push_back(pose);
  }

  void setPhase(Phase next, const ros::Time& now) {
    ROS_INFO("Loop route phase: %s -> %s", phaseName(phase_), phaseName(next));
    phase_ = next;
    phase_started_ = now;
  }

  double distance(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) const {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void requestOffboardAndArm(const ros::Time& now) {
    if (!auto_arm_ || now - last_request_ < ros::Duration(2.0)) return;
    if (state_.mode != "OFFBOARD") {
      mavros_msgs::SetMode request;
      request.request.custom_mode = "OFFBOARD";
      if (mode_client_.call(request) && request.response.mode_sent) ROS_INFO("OFFBOARD requested");
    } else if (!state_.armed) {
      mavros_msgs::CommandBool request;
      request.request.value = true;
      if (arming_client_.call(request) && request.response.success) ROS_INFO("Arming requested");
    }
    last_request_ = now;
  }

  bool moveTargetToward(const geometry_msgs::PoseStamped& destination, double step) {
    const double dx = destination.pose.position.x - target_.pose.position.x;
    const double dy = destination.pose.position.y - target_.pose.position.y;
    const double dz = destination.pose.position.z - target_.pose.position.z;
    const double remaining = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (remaining <= step || remaining < 1e-6) {
      target_ = destination;
      return true;
    }
    target_.pose.position.x += step * dx / remaining;
    target_.pose.position.y += step * dy / remaining;
    target_.pose.position.z += step * dz / remaining;
    return false;
  }

  void update(const ros::Time& now, double dt) {
    if (phase_ == Phase::PRESTREAM) {
      target_ = start_pose_;
      if (now - phase_started_ >= ros::Duration(5.0)) setPhase(Phase::ARMING, now);
      return;
    }
    if (phase_ == Phase::ARMING) {
      requestOffboardAndArm(now);
      if (state_.mode == "OFFBOARD" && state_.armed) {
        target_ = waypoints_.front();
        setPhase(Phase::TAKEOFF, now);
      }
      return;
    }
    if (phase_ == Phase::TAKEOFF) {
      target_ = waypoints_.front();
      if (distance(current_pose_, target_) < position_tolerance_) setPhase(Phase::SETTLE, now);
      return;
    }
    if (phase_ == Phase::SETTLE) {
      if (now - phase_started_ >= ros::Duration(settle_seconds_)) {
        waypoint_index_ = 1;
        setPhase(Phase::ROUTE, now);
      }
      return;
    }
    if (phase_ == Phase::ROUTE) {
      const bool reference_arrived = moveTargetToward(waypoints_[waypoint_index_], cruise_speed_ * dt);
      if (distance(current_pose_, target_) > max_tracking_error_) {
        target_ = current_pose_;
        target_.header.frame_id = "map";
        setPhase(Phase::ABORT_HOLD, now);
        ROS_ERROR("Tracking error > %.2f m. Holding instead of continuing an unsafe route.", max_tracking_error_);
        return;
      }
      if (reference_arrived && distance(current_pose_, waypoints_[waypoint_index_]) < position_tolerance_) {
        setPhase(Phase::CORNER_HOLD, now);
      }
      return;
    }
    if (phase_ == Phase::CORNER_HOLD && now - phase_started_ >= ros::Duration(corner_hold_seconds_)) {
      if (++waypoint_index_ >= waypoints_.size()) {
        waypoint_index_ = waypoints_.size() - 1;
        setPhase(Phase::FINAL_HOLD, now);
      } else {
        setPhase(Phase::ROUTE, now);
      }
      return;
    }
    if (phase_ == Phase::FINAL_HOLD) {
      target_ = waypoints_.back();
      if (now - phase_started_ >= ros::Duration(final_hold_seconds_)) setPhase(Phase::LANDING, now);
      return;
    }
    if (phase_ == Phase::LANDING) {
      target_.pose.position.x = start_pose_.pose.position.x;
      target_.pose.position.y = start_pose_.pose.position.y;
      target_.pose.position.z = start_pose_.pose.position.z;
      // Let PX4's land detector decide when it is safe to disarm. Directly
      // disarming at a small pose-Z threshold can still be rejected while the
      // vehicle has downward velocity, after which stopping OFFBOARD causes a
      // needless failsafe.
      if (auto_land_ && state_.armed && state_.mode != "AUTO.LAND" &&
          now - last_request_ >= ros::Duration(2.0)) {
        mavros_msgs::SetMode request;
        request.request.custom_mode = "AUTO.LAND";
        if (mode_client_.call(request) && request.response.mode_sent)
          ROS_INFO("AUTO.LAND requested; waiting for PX4 land detection and auto-disarm.");
        last_request_ = now;
      }
      const bool manual_landing_complete = !auto_land_ &&
          std::abs(current_pose_.pose.position.z - start_pose_.pose.position.z) <
              landing_tolerance_ &&
          now - phase_started_ >= ros::Duration(2.0);
      if (!state_.armed || manual_landing_complete) {
        std_msgs::Bool finished;
        finished.data = true;
        finished_pub_.publish(finished);
        setPhase(Phase::DONE, now);
      }
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber state_sub_, pose_sub_;
  ros::Publisher setpoint_pub_, finished_pub_;
  ros::ServiceClient arming_client_, mode_client_;
  mavros_msgs::State state_;
  geometry_msgs::PoseStamped current_pose_, start_pose_, target_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  bool have_pose_ = false, auto_arm_ = true, auto_land_ = true;
  double takeoff_height_, side_length_, cruise_speed_, settle_seconds_, corner_hold_seconds_, final_hold_seconds_;
  double position_tolerance_, landing_tolerance_, max_tracking_error_;
  std::size_t waypoint_index_ = 0;
  Phase phase_ = Phase::PRESTREAM;
  ros::Time phase_started_, last_update_, last_request_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_closure_demo_flight");
  LoopClosureDemoFlight flight;
  flight.run();
  return 0;
}
