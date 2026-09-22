#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
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
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
        // Consecutive current scans whose rays traversed this voxel without a
        // return. This is the negative evidence needed to remove old people.
        std::uint16_t free_misses{0};
    };

    struct TrailSample
    {
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        ros::Time stamp;
    };

    struct FrameCell
    {
        Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
        std::uint32_t count{0};
    };

    StaticMapBuilder() : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("cloud_topic", cloud_topic_,
                                "/uav1/fastlio/registered_scan");
        pnh_.param<std::string>("objects_topic", objects_topic_,
                                "/uav1/dynamic_objects");
        pnh_.param<std::string>("output_topic", output_topic_,
                                "/uav1/fastlio/cloud_map");
        pnh_.param<std::string>("legacy_output_topic", legacy_output_topic_,
                                "/uav1/static_map");
        pnh_.param<std::string>("stable_output_topic", stable_output_topic_,
                                "/uav1/stable_static_map");
        pnh_.param<std::string>("local_output_topic", local_output_topic_,
                                "/uav1/local_static_map");
        pnh_.param<std::string>("free_space_output_topic",
                                free_space_output_topic_,
                                "/uav1/local_free_space");
        pnh_.param<std::string>("odom_topic", odom_topic_,
                                "/uav1/fastlio/odom");
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param("voxel_size", voxel_size_, 0.15);
        pnh_.param("dynamic_mask_radius", dynamic_mask_radius_, 0.65);
        pnh_.param("dynamic_mask_half_height", dynamic_mask_half_height_, 1.0);
        pnh_.param("backward_trail_seconds", backward_trail_seconds_, 1.5);
        pnh_.param("trail_step", trail_step_, 0.15);
        pnh_.param("dynamic_history_seconds", dynamic_history_seconds_, 20.0);
        pnh_.param("minimum_static_hits", minimum_static_hits_, 2);
        pnh_.param("stable_minimum_static_hits",
                   stable_minimum_static_hits_, 8);
        pnh_.param("local_minimum_hits", local_minimum_hits_, 1);
        pnh_.param("local_max_age_frames", local_max_age_frames_, 15);
        pnh_.param("local_radius", local_radius_, 6.0);
        pnh_.param("enable_free_space_clearing",
                   enable_free_space_clearing_, true);
        pnh_.param("free_space_miss_threshold",
                   free_space_miss_threshold_, 3);
        pnh_.param("free_space_endpoint_margin",
                   free_space_endpoint_margin_, 0.30);
        pnh_.param("free_space_max_range", free_space_max_range_, 6.0);
        pnh_.param("free_space_ray_step", free_space_ray_step_, 0.15);
        pnh_.param("sensor_origin_z_offset", sensor_origin_z_offset_, 0.0);
        pnh_.param("max_free_rays", max_free_rays_, 8000);
        pnh_.param("max_free_voxels", max_free_voxels_, 200000);
        pnh_.param("object_timeout", object_timeout_, 0.40);
        pnh_.param("publish_rate", publish_rate_, 2.0);
        pnh_.param("max_voxels", max_voxels_, 500000);

        voxel_size_ = std::max(0.03, voxel_size_);
        dynamic_mask_radius_ = std::max(voxel_size_, dynamic_mask_radius_);
        trail_step_ = std::max(voxel_size_, trail_step_);
        minimum_static_hits_ = std::max(1, minimum_static_hits_);
        stable_minimum_static_hits_ = std::max(
            minimum_static_hits_, stable_minimum_static_hits_);
        local_minimum_hits_ = std::max(1, local_minimum_hits_);
        local_max_age_frames_ = std::max(1, local_max_age_frames_);
        local_radius_ = std::max(1.0, local_radius_);
        free_space_miss_threshold_ = std::max(1, free_space_miss_threshold_);
        free_space_endpoint_margin_ = std::max(
            voxel_size_, free_space_endpoint_margin_);
        free_space_max_range_ = std::max(1.0, free_space_max_range_);
        free_space_ray_step_ = std::max(voxel_size_, free_space_ray_step_);
        max_free_rays_ = std::max(100, max_free_rays_);
        max_free_voxels_ = std::max(1000, max_free_voxels_);
        dynamic_history_seconds_ = std::max(1.0, dynamic_history_seconds_);
        publish_rate_ = std::max(0.2, publish_rate_);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 2,
            &StaticMapBuilder::cloudCallback, this);
        objects_sub_ = nh_.subscribe(objects_topic_, 10,
            &StaticMapBuilder::objectsCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20,
            &StaticMapBuilder::odomCallback, this);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, true);
        if (!legacy_output_topic_.empty() &&
            legacy_output_topic_ != output_topic_)
        {
            legacy_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                legacy_output_topic_, 1, true);
            publish_legacy_map_ = true;
        }
        stable_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            stable_output_topic_, 1, true);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            local_output_topic_, 1, true);
        free_space_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            free_space_output_topic_, 1, true);
        clear_service_ = pnh_.advertiseService("clear",
            &StaticMapBuilder::clearCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
            &StaticMapBuilder::timerCallback, this);

        ROS_INFO("[StaticMapBuilder] scan=%s objects=%s global=%s stable=%s "
                 "local=%s free=%s voxel=%.2f mask=%.2f history=%.1fs",
                 cloud_topic_.c_str(), objects_topic_.c_str(),
                 output_topic_.c_str(), stable_output_topic_.c_str(),
                 local_output_topic_.c_str(), free_space_output_topic_.c_str(),
                 voxel_size_,
                 dynamic_mask_radius_, dynamic_history_seconds_);
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

    Eigen::Vector3d centerFor(const Key& key) const
    {
        return Eigen::Vector3d((static_cast<double>(key.x) + 0.5) * voxel_size_,
                               (static_cast<double>(key.y) + 0.5) * voxel_size_,
                               (static_cast<double>(key.z) + 0.5) * voxel_size_);
    }

    void updateFreeSpace(
        const std::unordered_map<Key, FrameCell, KeyHash>& frame_cells)
    {
        latest_free_cells_.clear();
        if (!enable_free_space_clearing_ || !have_odom_) return;

        Eigen::Vector3d sensor_origin = uav_position_;
        sensor_origin.z() += sensor_origin_z_offset_;
        latest_free_cells_.reserve(std::min<std::size_t>(
            frame_cells.size() * 8U,
            static_cast<std::size_t>(max_free_voxels_)));

        const std::size_t ray_stride = std::max<std::size_t>(
            1U, (frame_cells.size() + static_cast<std::size_t>(max_free_rays_) - 1U) /
                    static_cast<std::size_t>(max_free_rays_));
        std::size_t ray_index = 0;
        for (const auto& item : frame_cells)
        {
            if ((ray_index++ % ray_stride) != 0U) continue;
            const Eigen::Vector3d endpoint =
                item.second.sum / static_cast<double>(item.second.count);
            const Eigen::Vector3d ray = endpoint - sensor_origin;
            const double distance = ray.norm();
            const double trace_length = std::min(
                free_space_max_range_, distance - free_space_endpoint_margin_);
            if (distance < 1e-6 || trace_length <= free_space_ray_step_) continue;
            const Eigen::Vector3d direction = ray / distance;
            const int samples = static_cast<int>(std::floor(
                trace_length / free_space_ray_step_));
            for (int i = 1; i <= samples; ++i)
            {
                latest_free_cells_.insert(keyFor(
                    sensor_origin + direction * (i * free_space_ray_step_)));
                if (latest_free_cells_.size() >=
                    static_cast<std::size_t>(max_free_voxels_))
                    break;
            }
            if (latest_free_cells_.size() >=
                static_cast<std::size_t>(max_free_voxels_))
                break;
        }

        // A measured endpoint always wins over a traversed ray in the same
        // frame. This protects thin real structures from accidental clearing.
        for (const auto& item : frame_cells)
            latest_free_cells_.erase(item.first);

        for (const auto& key : latest_free_cells_)
        {
            auto cell = cells_.find(key);
            if (cell == cells_.end()) continue;
            cell->second.free_misses = static_cast<std::uint16_t>(std::min<int>(
                free_space_miss_threshold_, cell->second.free_misses + 1));
            if (cell->second.free_misses >= free_space_miss_threshold_)
                cells_.erase(cell);
        }
    }

    bool maskedByDynamicObject(const Eigen::Vector3d& point,
                               const ros::Time& now) const
    {
        const double radius_sq = dynamic_mask_radius_ * dynamic_mask_radius_;
        // Clear the complete recently observed trajectory.  Looking only behind
        // the latest velocity estimate leaves gaps whenever a track is briefly
        // lost or changes direction, which is perceived by A* as a false wall.
        for (const auto& item : dynamic_trails_)
        {
            for (const auto& sample : item.second)
            {
                if ((now - sample.stamp).toSec() > dynamic_history_seconds_)
                    continue;
                if (std::abs(point.z() - sample.center.z()) >
                    dynamic_mask_half_height_)
                    continue;
                if ((point.head<2>() - sample.center.head<2>()).squaredNorm() <=
                    radius_sq)
                    return true;
            }
        }

        if (!have_objects_ ||
            (now - objects_receive_time_).toSec() > object_timeout_)
            return false;

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
        const auto erase_cylinder = [&](const Eigen::Vector3d& center)
        {
            const Key key = keyFor(center);
            const int radius_cells = static_cast<int>(std::ceil(
                dynamic_mask_radius_ / voxel_size_)) + 1;
            const int height_cells = static_cast<int>(std::ceil(
                dynamic_mask_half_height_ / voxel_size_)) + 1;
            const double conservative_radius = dynamic_mask_radius_ + voxel_size_;
            for (int dz = -height_cells; dz <= height_cells; ++dz)
                for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    {
                        if (std::hypot(dx * voxel_size_, dy * voxel_size_) >
                            conservative_radius)
                            continue;
                        cells_.erase({key.x + dx, key.y + dy, key.z + dz});
                    }
        };

        for (const auto& item : dynamic_trails_)
            for (const auto& sample : item.second)
                if ((now - sample.stamp).toSec() <= dynamic_history_seconds_)
                    erase_cylinder(sample.center);

        if (!have_objects_ ||
            (now - objects_receive_time_).toSec() > object_timeout_)
            return;
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
                const double t = backward_trail_seconds_ *
                    static_cast<double>(i) / static_cast<double>(samples);
                erase_cylinder(center - velocity * t);
            }
        }
    }

    void objectsCallback(const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        latest_objects_ = *msg;
        objects_receive_time_ = ros::Time::now();
        have_objects_ = true;
        pruneDynamicTrails(objects_receive_time_);
        for (const auto& object : msg->objects)
        {
            // Predictions share the same track id but are future positions.  Do
            // not record them as already traversed space.
            if (object.predicted) continue;
            const Eigen::Vector3d center(object.pose.position.x,
                                         object.pose.position.y,
                                         object.pose.position.z);
            auto& trail = dynamic_trails_[object.id];
            if (trail.empty() ||
                (trail.back().center.head<2>() - center.head<2>()).norm() >=
                    trail_step_)
                trail.push_back({center, objects_receive_time_});
            else
            {
                trail.back().center = center;
                trail.back().stamp = objects_receive_time_;
            }
        }
        clearDynamicVoxels(objects_receive_time_);
    }

    void pruneDynamicTrails(const ros::Time& now)
    {
        for (auto it = dynamic_trails_.begin(); it != dynamic_trails_.end();)
        {
            auto& trail = it->second;
            while (!trail.empty() &&
                   (now - trail.front().stamp).toSec() > dynamic_history_seconds_)
                trail.pop_front();
            if (trail.empty())
                it = dynamic_trails_.erase(it);
            else
                ++it;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        uav_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
        have_odom_ = true;
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
        pruneDynamicTrails(now);
        clearDynamicVoxels(now);

        std::size_t accepted = 0;
        std::unordered_map<Key, FrameCell, KeyHash> frame_cells;
        frame_cells.reserve(cloud.size());
        for (const auto& p : cloud.points)
        {
            const Eigen::Vector3d point(p.x, p.y, p.z);
            if (!point.allFinite() || maskedByDynamicObject(point, now)) continue;
            FrameCell& frame_cell = frame_cells[keyFor(point)];
            frame_cell.sum += point;
            ++frame_cell.count;
            ++accepted;
        }

        // hits counts independent LiDAR frames, not raw points.  Otherwise a
        // dense pedestrian return can become "stable" in a single scan.
        for (const auto& item : frame_cells)
        {
            const Eigen::Vector3d observation =
                item.second.sum / static_cast<double>(item.second.count);
            Cell& cell = cells_[item.first];
            ++cell.hits;
            cell.last_frame = frame_index_;
            cell.free_misses = 0;
            const double alpha = 1.0 / static_cast<double>(
                std::min<std::uint32_t>(cell.hits, 20));
            cell.mean = (1.0 - alpha) * cell.mean + alpha * observation;
        }

        // Use current local rays as negative occupancy observations. A stale
        // body/vehicle point is removed after several scans see through it.
        updateFreeSpace(frame_cells);

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
        CloudT stable_map;
        CloudT local_map;
        CloudT free_space;
        map.reserve(cells_.size());
        stable_map.reserve(cells_.size());
        local_map.reserve(cells_.size());
        free_space.reserve(latest_free_cells_.size());
        const std::uint64_t oldest_local_frame =
            frame_index_ > static_cast<std::uint64_t>(local_max_age_frames_)
                ? frame_index_ - static_cast<std::uint64_t>(local_max_age_frames_)
                : 0;
        const double local_radius_sq = local_radius_ * local_radius_;
        for (const auto& item : cells_)
        {
            PointT p;
            p.x = static_cast<float>(item.second.mean.x());
            p.y = static_cast<float>(item.second.mean.y());
            p.z = static_cast<float>(item.second.mean.z());
            if (have_odom_ && item.second.free_misses == 0 &&
                item.second.last_frame >= oldest_local_frame &&
                item.second.hits >= static_cast<std::uint32_t>(local_minimum_hits_) &&
                (item.second.mean.head<2>() - uav_position_.head<2>()).squaredNorm() <=
                    local_radius_sq)
                local_map.push_back(p);

            if (item.second.hits < static_cast<std::uint32_t>(minimum_static_hits_))
                continue;
            map.push_back(p);
            // A second, conservative map is used for dynamic-object veto and
            // global planning.  Requiring many observations prevents a moving
            // pedestrian trail from being mistaken for permanent structure.
            if (item.second.hits >=
                static_cast<std::uint32_t>(stable_minimum_static_hits_))
                stable_map.push_back(p);
        }
        for (const auto& key : latest_free_cells_)
        {
            const Eigen::Vector3d center = centerFor(key);
            PointT p;
            p.x = static_cast<float>(center.x());
            p.y = static_cast<float>(center.y());
            p.z = static_cast<float>(center.z());
            free_space.push_back(p);
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(map, output);
        output.header = last_header_;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = world_frame_;
        map_pub_.publish(output);
        if (publish_legacy_map_) legacy_map_pub_.publish(output);

        sensor_msgs::PointCloud2 stable_output;
        pcl::toROSMsg(stable_map, stable_output);
        stable_output.header = output.header;
        stable_map_pub_.publish(stable_output);

        sensor_msgs::PointCloud2 local_output;
        pcl::toROSMsg(local_map, local_output);
        local_output.header = output.header;
        local_map_pub_.publish(local_output);

        sensor_msgs::PointCloud2 free_output;
        pcl::toROSMsg(free_space, free_output);
        free_output.header = output.header;
        free_space_pub_.publish(free_output);
    }

    bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        cells_.clear();
        dynamic_trails_.clear();
        latest_free_cells_.clear();
        frame_index_ = 0;
        ROS_WARN("[StaticMapBuilder] map cleared by service request.");
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher map_pub_, legacy_map_pub_, stable_map_pub_, local_map_pub_;
    ros::Publisher free_space_pub_;
    ros::ServiceServer clear_service_;
    ros::Timer timer_;
    std::string cloud_topic_, objects_topic_, odom_topic_, output_topic_;
    std::string legacy_output_topic_;
    std::string stable_output_topic_, local_output_topic_, world_frame_;
    std::string free_space_output_topic_;
    double voxel_size_{0.15}, dynamic_mask_radius_{0.65};
    double dynamic_mask_half_height_{1.0}, backward_trail_seconds_{1.5};
    double trail_step_{0.15}, dynamic_history_seconds_{20.0};
    double object_timeout_{0.40}, publish_rate_{2.0}, local_radius_{6.0};
    double free_space_endpoint_margin_{0.30}, free_space_max_range_{6.0};
    double free_space_ray_step_{0.15}, sensor_origin_z_offset_{0.0};
    int minimum_static_hits_{2}, stable_minimum_static_hits_{8};
    int local_minimum_hits_{1}, local_max_age_frames_{15};
    int free_space_miss_threshold_{3}, max_free_rays_{8000};
    int max_free_voxels_{200000};
    int max_voxels_{500000};
    std::uint64_t frame_index_{0};
    std_msgs::Header last_header_;
    std::unordered_map<Key, Cell, KeyHash> cells_;
    std::unordered_set<Key, KeyHash> latest_free_cells_;
    std::unordered_map<int, std::deque<TrailSample>> dynamic_trails_;
    fastlio_bridge::DynamicObjectArray latest_objects_;
    ros::Time objects_receive_time_;
    bool have_objects_{false};
    Eigen::Vector3d uav_position_{Eigen::Vector3d::Zero()};
    bool have_odom_{false};
    bool publish_legacy_map_{false};
    bool enable_free_space_clearing_{true};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_map_builder");
    StaticMapBuilder node;
    ros::spin();
    return 0;
}
