#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

class StaticMapBuilder
{
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    struct Key
    {
        int x{0};
        int y{0};
        int z{0};
        bool operator==(const Key& rhs) const
        {
            return x == rhs.x && y == rhs.y && z == rhs.z;
        }
    };

    struct KeyHash
    {
        std::size_t operator()(const Key& key) const
        {
            const std::size_t x = std::hash<int>()(key.x);
            const std::size_t y = std::hash<int>()(key.y);
            const std::size_t z = std::hash<int>()(key.z);
            return x ^ (y << 1U) ^ (z << 2U);
        }
    };

    struct Cell
    {
        Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
        std::uint32_t hits{0};
        std::uint64_t last_frame{0};
    };

    StaticMapBuilder() : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("cloud_topic", cloud_topic_,
                                "/uav1/fastlio/cloud_map");
        pnh_.param<std::string>("objects_topic", objects_topic_,
                                "/uav1/dynamic_objects");
        pnh_.param<std::string>("output_topic", output_topic_,
                                "/uav1/static_map");
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param("voxel_size", voxel_size_, 0.15);
        pnh_.param("dynamic_mask_radius", dynamic_mask_radius_, 0.65);
        pnh_.param("dynamic_mask_half_height", dynamic_mask_half_height_, 1.0);
        pnh_.param("backward_trail_seconds", backward_trail_seconds_, 1.5);
        pnh_.param("trail_step", trail_step_, 0.15);
        pnh_.param("minimum_static_hits", minimum_static_hits_, 2);
        pnh_.param("object_timeout", object_timeout_, 0.40);
        pnh_.param("publish_rate", publish_rate_, 2.0);
        pnh_.param("max_voxels", max_voxels_, 500000);

        voxel_size_ = std::max(0.03, voxel_size_);
        dynamic_mask_radius_ = std::max(voxel_size_, dynamic_mask_radius_);
        trail_step_ = std::max(voxel_size_, trail_step_);
        minimum_static_hits_ = std::max(1, minimum_static_hits_);
        publish_rate_ = std::max(0.2, publish_rate_);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 2,
            &StaticMapBuilder::cloudCallback, this);
        objects_sub_ = nh_.subscribe(objects_topic_, 10,
            &StaticMapBuilder::objectsCallback, this);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, true);
        clear_service_ = pnh_.advertiseService("clear",
            &StaticMapBuilder::clearCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
            &StaticMapBuilder::timerCallback, this);

        ROS_INFO("[StaticMapBuilder] cloud=%s objects=%s output=%s voxel=%.2f mask=%.2f",
                 cloud_topic_.c_str(), objects_topic_.c_str(),
                 output_topic_.c_str(), voxel_size_, dynamic_mask_radius_);
    }

private:
    static std::string normalizeFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
        return frame;
    }

    Key keyFor(const Eigen::Vector3d& p) const
    {
        return {static_cast<int>(std::floor(p.x() / voxel_size_)),
                static_cast<int>(std::floor(p.y() / voxel_size_)),
                static_cast<int>(std::floor(p.z() / voxel_size_))};
    }

    bool maskedByDynamicObject(const Eigen::Vector3d& point,
                               const ros::Time& now) const
    {
        if (!have_objects_ || (now - objects_receive_time_).toSec() > object_timeout_)
            return false;

        const double radius_sq = dynamic_mask_radius_ * dynamic_mask_radius_;
        for (const auto& object : latest_objects_.objects)
        {
            const Eigen::Vector3d center(object.pose.position.x,
                                         object.pose.position.y,
                                         object.pose.position.z);
            const Eigen::Vector3d velocity(object.twist.linear.x,
                                           object.twist.linear.y,
                                           object.twist.linear.z);
            const double speed = velocity.head<2>().norm();
            const int samples = speed > 0.05
                ? std::max(1, static_cast<int>(std::ceil(
                      speed * backward_trail_seconds_ / trail_step_)))
                : 1;

            for (int i = 0; i <= samples; ++i)
            {
                const double t = samples > 0
                    ? backward_trail_seconds_ * static_cast<double>(i) /
                          static_cast<double>(samples)
                    : 0.0;
                const Eigen::Vector3d trail_center = center - velocity * t;
                if (std::abs(point.z() - trail_center.z()) >
                    dynamic_mask_half_height_)
                    continue;
                if ((point.head<2>() - trail_center.head<2>()).squaredNorm() <=
                    radius_sq)
                    return true;
            }
        }
        return false;
    }

    void clearDynamicVoxels(const ros::Time& now)
    {
        if (!have_objects_ || (now - objects_receive_time_).toSec() > object_timeout_)
            return;
        for (auto it = cells_.begin(); it != cells_.end();)
        {
            if (maskedByDynamicObject(it->second.mean, now))
                it = cells_.erase(it);
            else
                ++it;
        }
    }

    void objectsCallback(const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        latest_objects_ = *msg;
        objects_receive_time_ = ros::Time::now();
        have_objects_ = true;
        clearDynamicVoxels(objects_receive_time_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
        {
            ROS_ERROR_THROTTLE(1.0,
                "[StaticMapBuilder] cloud frame '%s' != '%s'; no relabeling is performed.",
                msg->header.frame_id.c_str(), world_frame_.c_str());
            return;
        }

        CloudT cloud;
        pcl::fromROSMsg(*msg, cloud);
        const ros::Time now = ros::Time::now();
        ++frame_index_;
        clearDynamicVoxels(now);

        std::size_t accepted = 0;
        for (const auto& p : cloud.points)
        {
            const Eigen::Vector3d point(p.x, p.y, p.z);
            if (!point.allFinite() || maskedByDynamicObject(point, now)) continue;
            Cell& cell = cells_[keyFor(point)];
            ++cell.hits;
            cell.last_frame = frame_index_;
            const double alpha = 1.0 / static_cast<double>(std::min<std::uint32_t>(cell.hits, 20));
            cell.mean = (1.0 - alpha) * cell.mean + alpha * point;
            ++accepted;
        }

        enforceMemoryLimit();
        last_header_ = msg->header;
        ROS_INFO_THROTTLE(1.0,
            "[StaticMapBuilder] input=%zu accepted=%zu voxels=%zu",
            cloud.size(), accepted, cells_.size());
    }

    void enforceMemoryLimit()
    {
        if (max_voxels_ <= 0 || cells_.size() <= static_cast<std::size_t>(max_voxels_))
            return;
        std::vector<std::pair<Key, std::uint64_t>> ages;
        ages.reserve(cells_.size());
        for (const auto& item : cells_) ages.emplace_back(item.first, item.second.last_frame);
        const std::size_t remove_count = cells_.size() - static_cast<std::size_t>(max_voxels_);
        std::nth_element(ages.begin(), ages.begin() + remove_count, ages.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        for (std::size_t i = 0; i < remove_count; ++i) cells_.erase(ages[i].first);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        CloudT map;
        map.reserve(cells_.size());
        for (const auto& item : cells_)
        {
            if (item.second.hits < static_cast<std::uint32_t>(minimum_static_hits_))
                continue;
            PointT p;
            p.x = static_cast<float>(item.second.mean.x());
            p.y = static_cast<float>(item.second.mean.y());
            p.z = static_cast<float>(item.second.mean.z());
            map.push_back(p);
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(map, output);
        output.header = last_header_;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = world_frame_;
        map_pub_.publish(output);
    }

    bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        cells_.clear();
        frame_index_ = 0;
        ROS_WARN("[StaticMapBuilder] map cleared by service request.");
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber objects_sub_;
    ros::Publisher map_pub_;
    ros::ServiceServer clear_service_;
    ros::Timer timer_;
    std::string cloud_topic_, objects_topic_, output_topic_, world_frame_;
    double voxel_size_{0.15}, dynamic_mask_radius_{0.65};
    double dynamic_mask_half_height_{1.0}, backward_trail_seconds_{1.5};
    double trail_step_{0.15}, object_timeout_{0.40}, publish_rate_{2.0};
    int minimum_static_hits_{2}, max_voxels_{500000};
    std::uint64_t frame_index_{0};
    std_msgs::Header last_header_;
    std::unordered_map<Key, Cell, KeyHash> cells_;
    fastlio_bridge::DynamicObjectArray latest_objects_;
    ros::Time objects_receive_time_;
    bool have_objects_{false};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_map_builder");
    StaticMapBuilder node;
    ros::spin();
    return 0;
}
