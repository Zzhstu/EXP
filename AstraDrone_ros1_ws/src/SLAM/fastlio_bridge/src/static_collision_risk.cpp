#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>


class StaticCollisionRisk
{
public:
    enum RiskLevel : std::uint8_t
    {
        SAFE = 0,
        AVOID = 1,
        EMERGENCY = 2
    };

    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    StaticCollisionRisk()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "cloud_topic",
            cloud_topic_,
            "/uav1/fastlio/registered_scan");

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/uav1/fastlio/odom");

        pnh_.param<std::string>(
            "dynamic_objects_topic",
            dynamic_objects_topic_,
            "/uav1/dynamic_objects");

        pnh_.param<std::string>("world_frame", world_frame_, "map");

        // 先测量机体中心到障碍物表面的距离，再扣除无人机二维外包络半径。
        // 默认 0.50 m 比原先的 0.25 m 更保守；实际机型可由私有参数
        // _uav_radius 覆盖。风险等级始终以扣除后的 clearance 为准。
        pnh_.param("uav_radius", uav_radius_, 0.50);
        // 等级标准：
        // clearance < 0.60 m -> level=2
        // 0.60 m <= clearance < 1.00 m -> level=1
        // clearance >= 1.00 m -> level=0
        // 用户指定 clearance < 0.10 m 时仍优先输出 level=2。
        pnh_.param("safe_clearance", safe_clearance_, 1.00);
        pnh_.param("emergency_clearance", emergency_clearance_, 0.60);
        pnh_.param("exit_clearance", exit_clearance_, 1.00);

        // 只检测与机身高度重叠的点，排除地面和较高障碍物。
        pnh_.param("min_relative_z", min_relative_z_, -0.35);
        pnh_.param("max_relative_z", max_relative_z_, 0.45);
        pnh_.param("check_radius", check_radius_, 3.0);
        pnh_.param("min_valid_distance", min_valid_distance_, 0.05);

        // 最近点附近必须有多个支持点，避免单个离群点触发紧急状态。
        pnh_.param("support_radius", support_radius_, 0.35);
        pnh_.param("min_support_points", min_support_points_, 3);

        pnh_.param("min_avoid_speed", min_avoid_speed_, 0.25);
        pnh_.param("max_avoid_speed", max_avoid_speed_, 0.80);
        pnh_.param("avoidance_alpha", avoidance_alpha_, 0.25);

        pnh_.param("cloud_timeout", cloud_timeout_, 0.30);
        pnh_.param("odom_timeout", odom_timeout_, 0.30);

        // dynamic_objects 只提供目标中心，因此用圆柱形区域屏蔽该目标
        // 在当前 registered_scan 中的点。短暂漏检时继续保留掩膜，避免
        // 静态风险抖动。
        pnh_.param("dynamic_mask_radius", dynamic_mask_radius_, 0.60);
        pnh_.param(
            "dynamic_mask_half_height",
            dynamic_mask_half_height_,
            1.00);
        pnh_.param(
            "dynamic_mask_hold_time",
            dynamic_mask_hold_time_,
            0.60);

        emergency_clearance_ = std::max(0.0, emergency_clearance_);
        uav_radius_ = std::max(0.0, uav_radius_);
        safe_clearance_ = std::max(
            emergency_clearance_ + 0.05,
            safe_clearance_);
        exit_clearance_ = std::max(safe_clearance_, exit_clearance_);
        min_avoid_speed_ = std::max(0.0, min_avoid_speed_);
        max_avoid_speed_ = std::max(min_avoid_speed_, max_avoid_speed_);
        avoidance_alpha_ = std::max(
            0.0,
            std::min(1.0, avoidance_alpha_));
        min_support_points_ = std::max(1, min_support_points_);
        dynamic_mask_radius_ = std::max(0.0, dynamic_mask_radius_);
        dynamic_mask_half_height_ =
            std::max(0.0, dynamic_mask_half_height_);
        dynamic_mask_hold_time_ =
            std::max(0.0, dynamic_mask_hold_time_);

        cloud_sub_ = nh_.subscribe(
            cloud_topic_,
            2,
            &StaticCollisionRisk::cloudCallback,
            this);

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &StaticCollisionRisk::odomCallback,
            this);

        dynamic_objects_sub_ = nh_.subscribe(
            dynamic_objects_topic_,
            10,
            &StaticCollisionRisk::dynamicObjectsCallback,
            this);

        risk_pub_ = nh_.advertise<std_msgs::UInt8>(
            "/uav1/static_collision_risk_level",
            10);

        avoidance_velocity_pub_ =
            nh_.advertise<geometry_msgs::TwistStamped>(
                "/uav1/static_avoidance_velocity",
                10);

        nearest_obstacle_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(
                "/uav1/nearest_static_obstacle",
                10);

        clearance_pub_ = nh_.advertise<std_msgs::Float32>(
            "/uav1/static_clearance",
            10);

        surface_distance_pub_ = nh_.advertise<std_msgs::Float32>(
            "/uav1/nearest_static_surface_distance",
            10);

        timer_ = nh_.createTimer(
            ros::Duration(0.05),
            &StaticCollisionRisk::timerCallback,
            this);

        ROS_INFO(
            "[StaticCollisionRisk] Started. radius=%.2f "
            "emergency_clearance=%.2f safe_clearance=%.2f "
            "exit_clearance=%.2f z=[%.2f, %.2f] "
            "dynamic_mask=(r=%.2f h=%.2f hold=%.2f)",
            uav_radius_,
            emergency_clearance_,
            safe_clearance_,
            exit_clearance_,
            min_relative_z_,
            max_relative_z_,
            dynamic_mask_radius_,
            dynamic_mask_half_height_,
            dynamic_mask_hold_time_);
    }

private:
    struct CandidatePoint
    {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        double horizontal_distance{
            std::numeric_limits<double>::infinity()};
    };

    struct DynamicMask
    {
        int id{-1};
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    };

    static std::string normalizedFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/')
        {
            frame.erase(frame.begin());
        }
        return frame;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        uav_position_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        odom_receive_time_ = ros::Time::now();
        have_odom_ = true;
    }

    void dynamicObjectsCallback(
        const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        if (normalizedFrame(msg->header.frame_id) !=
            normalizedFrame(world_frame_))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[StaticCollisionRisk] Dynamic-object frame is '%s', "
                "expected '%s'; mask update ignored.",
                msg->header.frame_id.c_str(),
                world_frame_.c_str());
            return;
        }

        // 空数组不立即清除旧掩膜；由 hold_time 处理短暂漏检。
        if (msg->objects.empty())
        {
            return;
        }

        std::vector<DynamicMask> updated_masks;
        updated_masks.reserve(msg->objects.size());

        for (const auto& object : msg->objects)
        {
            const Eigen::Vector3d center(
                object.pose.position.x,
                object.pose.position.y,
                object.pose.position.z);

            if (!center.allFinite())
            {
                continue;
            }

            DynamicMask mask;
            mask.id = object.id;
            mask.center = center;
            updated_masks.push_back(mask);
        }

        if (!updated_masks.empty())
        {
            dynamic_masks_ = std::move(updated_masks);
            dynamic_masks_receive_time_ = ros::Time::now();
            have_dynamic_masks_ = true;
        }
    }

    bool isInsideDynamicMask(
        const Eigen::Vector3d& position,
        const ros::Time& now) const
    {
        if (!have_dynamic_masks_ || dynamic_masks_.empty() ||
            dynamic_mask_radius_ <= 0.0 ||
            (now - dynamic_masks_receive_time_).toSec() >
                dynamic_mask_hold_time_)
        {
            return false;
        }

        const double radius_squared =
            dynamic_mask_radius_ * dynamic_mask_radius_;

        for (const auto& mask : dynamic_masks_)
        {
            const double dx = position.x() - mask.center.x();
            const double dy = position.y() - mask.center.y();
            const double dz = std::fabs(position.z() - mask.center.z());

            if (dx * dx + dy * dy <= radius_squared &&
                dz <= dynamic_mask_half_height_)
            {
                return true;
            }
        }

        return false;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        cloud_receive_time_ = ros::Time::now();
        have_cloud_message_ = true;

        if (!have_odom_)
        {
            measurement_valid_ = false;
            return;
        }

        if (normalizedFrame(msg->header.frame_id) !=
            normalizedFrame(world_frame_))
        {
            measurement_valid_ = false;

            ROS_ERROR_THROTTLE(
                1.0,
                "[StaticCollisionRisk] Cloud frame is '%s', expected '%s'.",
                msg->header.frame_id.c_str(),
                world_frame_.c_str());
            return;
        }

        CloudT cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::vector<CandidatePoint> candidates;
        candidates.reserve(cloud.size());

        masked_point_count_ = 0;
        const ros::Time now = ros::Time::now();

        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z))
            {
                continue;
            }

            const Eigen::Vector3d position(
                point.x,
                point.y,
                point.z);

            const Eigen::Vector3d relative =
                position - uav_position_;

            if (relative.z() < min_relative_z_ ||
                relative.z() > max_relative_z_)
            {
                continue;
            }

            const double horizontal_distance =
                relative.head<2>().norm();

            if (horizontal_distance < min_valid_distance_ ||
                horizontal_distance > check_radius_)
            {
                continue;
            }

            if (isInsideDynamicMask(position, now))
            {
                ++masked_point_count_;
                continue;
            }

            CandidatePoint candidate;
            candidate.position = position;
            candidate.horizontal_distance = horizontal_distance;
            candidates.push_back(candidate);
        }

        if (static_cast<int>(candidates.size()) < min_support_points_)
        {
            measurement_valid_ = false;
            support_count_ = static_cast<int>(candidates.size());
            return;
        }

        const auto nearest_it = std::min_element(
            candidates.begin(),
            candidates.end(),
            [](const CandidatePoint& a, const CandidatePoint& b)
            {
                return a.horizontal_distance < b.horizontal_distance;
            });

        const Eigen::Vector3d nearest_raw_point =
            nearest_it->position;

        std::vector<double> support_distances;
        support_distances.reserve(candidates.size());

        Eigen::Vector3d support_position_sum =
            Eigen::Vector3d::Zero();

        for (const auto& candidate : candidates)
        {
            if ((candidate.position - nearest_raw_point).norm() <=
                support_radius_)
            {
                support_position_sum += candidate.position;
                support_distances.push_back(
                    candidate.horizontal_distance);
            }
        }

        support_count_ = static_cast<int>(support_distances.size());

        if (support_count_ < min_support_points_)
        {
            measurement_valid_ = false;
            return;
        }

        std::sort(
            support_distances.begin(),
            support_distances.end());

        nearest_surface_distance_ =
            support_distances[support_distances.size() / 2];

        nearest_obstacle_position_ =
            support_position_sum /
            static_cast<double>(support_count_);

        measurement_valid_ = true;
    }

    void publishSafe(const ros::Time& now)
    {
        std_msgs::UInt8 risk;
        risk.data = SAFE;
        risk_pub_.publish(risk);

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = now;
        velocity.header.frame_id = world_frame_;
        avoidance_velocity_pub_.publish(velocity);

        std_msgs::Float32 clearance;
        clearance.data = measurement_valid_
            ? static_cast<float>(
                  nearest_surface_distance_ - uav_radius_)
            : std::numeric_limits<float>::infinity();
        clearance_pub_.publish(clearance);

        std_msgs::Float32 surface_distance;
        surface_distance.data = measurement_valid_
            ? static_cast<float>(nearest_surface_distance_)
            : std::numeric_limits<float>::infinity();
        surface_distance_pub_.publish(surface_distance);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        if (!have_odom_ ||
            (now - odom_receive_time_).toSec() > odom_timeout_)
        {
            risk_active_ = false;
            filtered_avoidance_velocity_.setZero();
            publishSafe(now);

            ROS_WARN_THROTTLE(
                1.0,
                "[StaticCollisionRisk] Odometry missing or stale.");
            return;
        }

        if (!have_cloud_message_ ||
            (now - cloud_receive_time_).toSec() > cloud_timeout_)
        {
            risk_active_ = false;
            filtered_avoidance_velocity_.setZero();
            measurement_valid_ = false;
            publishSafe(now);

            ROS_WARN_THROTTLE(
                1.0,
                "[StaticCollisionRisk] Point cloud missing or stale.");
            return;
        }

        if (!measurement_valid_)
        {
            risk_active_ = false;
            filtered_avoidance_velocity_.setZero();
            publishSafe(now);

            ROS_INFO_THROTTLE(
                1.0,
                "[StaticCollisionRisk] level=0 no supported obstacle "
                "(support=%d masked=%d)",
                support_count_,
                masked_point_count_);
            return;
        }

        const double clearance_value =
            nearest_surface_distance_ - uav_radius_;

        RiskLevel level = SAFE;

        if (clearance_value < emergency_clearance_)
        {
            level = EMERGENCY;
        }
        else if (clearance_value < safe_clearance_)
        {
            level = AVOID;
        }
        else if (risk_active_ &&
                 clearance_value < exit_clearance_)
        {
            level = AVOID;
        }

        if (level == SAFE)
        {
            risk_active_ = false;
            filtered_avoidance_velocity_.setZero();
            publishSafe(now);

            ROS_INFO_THROTTLE(
                1.0,
                "[StaticCollisionRisk] level=0 distance=%.2f "
                "clearance=%.2f support=%d masked=%d",
                nearest_surface_distance_,
                clearance_value,
                support_count_,
                masked_point_count_);
            return;
        }

        risk_active_ = true;

        Eigen::Vector2d away_direction =
            (uav_position_ - nearest_obstacle_position_).head<2>();

        if (away_direction.norm() < 1e-6)
        {
            away_direction = Eigen::Vector2d(1.0, 0.0);
        }
        else
        {
            away_direction.normalize();
        }

        const double denominator =
            std::max(
                0.05,
                safe_clearance_ - emergency_clearance_);

        const double severity = std::max(
            0.0,
            std::min(
                1.0,
                (safe_clearance_ - clearance_value) /
                    denominator));

        double speed =
            min_avoid_speed_ +
            severity * (max_avoid_speed_ - min_avoid_speed_);

        if (level == EMERGENCY)
        {
            speed = max_avoid_speed_;
        }

        const Eigen::Vector2d raw_velocity =
            speed * away_direction;

        const double alpha = level == EMERGENCY
            ? std::max(0.60, avoidance_alpha_)
            : avoidance_alpha_;

        filtered_avoidance_velocity_ =
            (1.0 - alpha) * filtered_avoidance_velocity_ +
            alpha * raw_velocity;

        if (filtered_avoidance_velocity_.norm() > max_avoid_speed_)
        {
            filtered_avoidance_velocity_ =
                max_avoid_speed_ *
                filtered_avoidance_velocity_.normalized();
        }

        std_msgs::UInt8 risk;
        risk.data = level;
        risk_pub_.publish(risk);

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = now;
        velocity.header.frame_id = world_frame_;
        velocity.twist.linear.x = filtered_avoidance_velocity_.x();
        velocity.twist.linear.y = filtered_avoidance_velocity_.y();
        velocity.twist.linear.z = 0.0;
        avoidance_velocity_pub_.publish(velocity);

        geometry_msgs::PoseStamped nearest_obstacle;
        nearest_obstacle.header.stamp = now;
        nearest_obstacle.header.frame_id = world_frame_;
        nearest_obstacle.pose.position.x =
            nearest_obstacle_position_.x();
        nearest_obstacle.pose.position.y =
            nearest_obstacle_position_.y();
        nearest_obstacle.pose.position.z =
            nearest_obstacle_position_.z();
        nearest_obstacle.pose.orientation.w = 1.0;
        nearest_obstacle_pub_.publish(nearest_obstacle);

        std_msgs::Float32 clearance;
        clearance.data = static_cast<float>(clearance_value);
        clearance_pub_.publish(clearance);

        std_msgs::Float32 surface_distance;
        surface_distance.data =
            static_cast<float>(nearest_surface_distance_);
        surface_distance_pub_.publish(surface_distance);

        ROS_INFO_THROTTLE(
            0.25,
            "[StaticCollisionRisk] level=%u distance=%.2f "
            "clearance=%.2f support=%d masked=%d avoid=(%.2f %.2f)",
            static_cast<unsigned int>(level),
            nearest_surface_distance_,
            clearance_value,
            support_count_,
            masked_point_count_,
            filtered_avoidance_velocity_.x(),
            filtered_avoidance_velocity_.y());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber dynamic_objects_sub_;
    ros::Publisher risk_pub_;
    ros::Publisher avoidance_velocity_pub_;
    ros::Publisher nearest_obstacle_pub_;
    ros::Publisher clearance_pub_;
    ros::Publisher surface_distance_pub_;
    ros::Timer timer_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string dynamic_objects_topic_;
    std::string world_frame_;

    double uav_radius_{0.50};
    double safe_clearance_{1.00};
    double emergency_clearance_{0.60};
    double exit_clearance_{1.00};
    double min_relative_z_{-0.35};
    double max_relative_z_{0.45};
    double check_radius_{3.0};
    double min_valid_distance_{0.05};
    double support_radius_{0.35};
    int min_support_points_{3};
    double min_avoid_speed_{0.25};
    double max_avoid_speed_{0.80};
    double avoidance_alpha_{0.25};
    double cloud_timeout_{0.30};
    double odom_timeout_{0.30};
    double dynamic_mask_radius_{0.60};
    double dynamic_mask_half_height_{1.00};
    double dynamic_mask_hold_time_{0.60};

    bool have_odom_{false};
    bool have_cloud_message_{false};
    bool measurement_valid_{false};
    bool risk_active_{false};
    bool have_dynamic_masks_{false};

    ros::Time odom_receive_time_;
    ros::Time cloud_receive_time_;
    ros::Time dynamic_masks_receive_time_;

    std::vector<DynamicMask> dynamic_masks_;

    Eigen::Vector3d uav_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d nearest_obstacle_position_{
        Eigen::Vector3d::Zero()};
    Eigen::Vector2d filtered_avoidance_velocity_{
        Eigen::Vector2d::Zero()};

    double nearest_surface_distance_{
        std::numeric_limits<double>::infinity()};
    int support_count_{0};
    int masked_point_count_{0};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_collision_risk");
    StaticCollisionRisk node;
    ros::spin();
    return 0;
}
