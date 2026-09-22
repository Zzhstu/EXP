#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>


class DynamicDetector
{
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    struct VoxelKey
    {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey& other) const
        {
            return x == other.x &&
                   y == other.y &&
                   z == other.z;
        }
    };

    struct VoxelHash
    {
        std::size_t operator()(const VoxelKey& key) const
        {
            const std::size_t h1 = std::hash<int>()(key.x);
            const std::size_t h2 = std::hash<int>()(key.y);
            const std::size_t h3 = std::hash<int>()(key.z);
            return h1 ^ (h2 << 1U) ^ (h3 << 2U);
        }
    };

    struct ForegroundState
    {
        int persistent_hits{0};
        std::uint64_t last_seen_frame{0};
    };

    DynamicDetector()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "cloud_topic",
            cloud_topic_,
            "/uav1/fastlio/cloud_map");

        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/uav1/fastlio/odom");

        pnh_.param<std::string>(
            "output_topic",
            output_topic_,
            "/uav1/foreground_points");

        pnh_.param<std::string>(
            "world_frame",
            world_frame_,
            "map");

        pnh_.param("voxel_size", voxel_size_, 0.20);
        pnh_.param("background_frames", background_frames_, 30);
        pnh_.param(
            "background_neighbor_range",
            background_neighbor_range_,
            1);

        pnh_.param(
            "online_static_confirm_frames",
            online_static_confirm_frames_,
            20);

        // 在线静态学习会把连续出现的前景体素永久写入背景。对动态障碍
        // 检测而言，慢速、暂停或反复经过同一路径的目标很容易因此消失，
        // 所以默认关闭；只有明确需要适应场景变化时才手动开启。
        pnh_.param(
            "enable_online_static_update",
            enable_online_static_update_,
            false);

        pnh_.param(
            "max_foreground_gap_frames",
            max_foreground_gap_frames_,
            3);

        // 高度门限必须相对无人机，而不能使用绝对 map.z；否则无人机
        // 改变飞行高度后，同高度的近距离障碍物可能被直接过滤掉。
        pnh_.param("min_relative_z", min_relative_z_, -0.60);
        pnh_.param("max_relative_z", max_relative_z_, 1.20);
        pnh_.param("max_range", max_range_, 15.0);

        // 近距离目标在视角变化下容易落到相邻体素。近场仅把完全相同的
        // 背景体素视为静态，远场仍保留较宽松的邻域抑制噪声。
        pnh_.param("near_field_range", near_field_range_, 3.0);
        pnh_.param(
            "near_field_background_neighbor_range",
            near_field_background_neighbor_range_,
            0);

        cloud_sub_ = nh_.subscribe(
            cloud_topic_,
            2,
            &DynamicDetector::cloudCallback,
            this);

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &DynamicDetector::odomCallback,
            this);

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            output_topic_,
            2);

        ROS_INFO(
            "[DynamicDetector] Started. cloud=%s odom=%s output=%s",
            cloud_topic_.c_str(),
            odom_topic_.c_str(),
            output_topic_.c_str());

        ROS_INFO(
            "[DynamicDetector] voxel=%.2f initial_background=%d "
            "online_update=%d online_static=%d neighbor=%d "
            "near_range=%.2f near_neighbor=%d "
            "relative_z=[%.2f, %.2f]",
            voxel_size_,
            background_frames_,
            static_cast<int>(enable_online_static_update_),
            online_static_confirm_frames_,
            background_neighbor_range_,
            near_field_range_,
            near_field_background_neighbor_range_,
            min_relative_z_,
            max_relative_z_);

        ROS_INFO(
            "[DynamicDetector] Building initial static background...");
    }

private:
    static std::string normalizedFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/')
        {
            frame.erase(frame.begin());
        }
        return frame;
    }

    VoxelKey pointToVoxel(const PointT& point) const
    {
        return {
            static_cast<int>(std::floor(point.x / voxel_size_)),
            static_cast<int>(std::floor(point.y / voxel_size_)),
            static_cast<int>(std::floor(point.z / voxel_size_))};
    }

    PointT voxelCenter(const VoxelKey& key) const
    {
        PointT point;
        point.x = static_cast<float>((key.x + 0.5) * voxel_size_);
        point.y = static_cast<float>((key.y + 0.5) * voxel_size_);
        point.z = static_cast<float>((key.z + 0.5) * voxel_size_);
        return point;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        uav_x_ = msg->pose.pose.position.x;
        uav_y_ = msg->pose.pose.position.y;
        uav_z_ = msg->pose.pose.position.z;
        have_odom_ = true;
    }

    bool isInsideDetectionRange(const PointT& point) const
    {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            return false;
        }

        const double relative_z = have_odom_
            ? point.z - uav_z_
            : point.z;

        if (have_odom_ &&
            (relative_z < min_relative_z_ ||
             relative_z > max_relative_z_))
        {
            return false;
        }

        // cloud_map 已经在 map 中，量程必须相对无人机计算。
        if (have_odom_ && max_range_ > 0.0)
        {
            const double dx = point.x - uav_x_;
            const double dy = point.y - uav_y_;
            const double dz = point.z - uav_z_;
            const double range =
                std::sqrt(dx * dx + dy * dy + dz * dz);

            if (range > max_range_)
            {
                return false;
            }
        }

        return true;
    }

    int backgroundNeighborRangeForKey(const VoxelKey& key) const
    {
        if (!have_odom_ || near_field_range_ <= 0.0)
        {
            return background_neighbor_range_;
        }

        const PointT center = voxelCenter(key);
        const double dx = center.x - uav_x_;
        const double dy = center.y - uav_y_;

        const double horizontal_range = std::sqrt(dx * dx + dy * dy);

        return horizontal_range < near_field_range_
            ? near_field_background_neighbor_range_
            : background_neighbor_range_;
    }

    bool isBackgroundVoxel(
        const VoxelKey& key,
        const int neighbor_range) const
    {
        for (int dx = -neighbor_range;
             dx <= neighbor_range;
             ++dx)
        {
            for (int dy = -neighbor_range;
                 dy <= neighbor_range;
                 ++dy)
            {
                for (int dz = -neighbor_range;
                     dz <= neighbor_range;
                     ++dz)
                {
                    const VoxelKey neighbor{
                        key.x + dx,
                        key.y + dy,
                        key.z + dz};

                    if (background_voxels_.find(neighbor) !=
                        background_voxels_.end())
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void publishCloud(
        const CloudT& cloud,
        const std_msgs::Header& source_header)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header = source_header;
        output.header.frame_id = world_frame_;
        pub_.publish(output);
    }

    void removeStaleForegroundStates()
    {
        for (auto it = foreground_states_.begin();
             it != foreground_states_.end();)
        {
            const std::uint64_t gap =
                frame_index_ - it->second.last_seen_frame;

            if (gap > static_cast<std::uint64_t>(
                          max_foreground_gap_frames_))
            {
                it = foreground_states_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void cloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ++frame_index_;

        if (normalizedFrame(msg->header.frame_id) !=
            normalizedFrame(world_frame_))
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[DynamicDetector] Input frame is '%s', expected '%s'. "
                "Transform the cloud instead of only relabeling it.",
                msg->header.frame_id.c_str(),
                world_frame_.c_str());
            return;
        }

        if (!have_odom_)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[DynamicDetector] Waiting for %s; "
                "range filtering is temporarily disabled.",
                odom_topic_.c_str());
        }

        CloudT::Ptr current(new CloudT);
        pcl::fromROSMsg(*msg, *current);

        if (current->empty())
        {
            CloudT empty;
            publishCloud(empty, msg->header);
            return;
        }

        CloudT::Ptr filtered(new CloudT);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(current);
        voxel_filter.setLeafSize(
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_));
        voxel_filter.filter(*filtered);

        std::unordered_set<VoxelKey, VoxelHash> current_voxels;
        current_voxels.reserve(filtered->size());

        for (const auto& point : filtered->points)
        {
            if (isInsideDetectionRange(point))
            {
                current_voxels.insert(pointToVoxel(point));
            }
        }

        if (!background_ready_)
        {
            background_building_voxels_.insert(
                current_voxels.begin(),
                current_voxels.end());

            ++background_frame_count_;

            ROS_INFO_THROTTLE(
                1.0,
                "[DynamicDetector] Initial background: %d / %d frames",
                background_frame_count_,
                background_frames_);

            if (background_frame_count_ >= background_frames_)
            {
                background_voxels_ = background_building_voxels_;
                background_ready_ = true;

                ROS_INFO(
                    "[DynamicDetector] Initial background ready: %zu voxels",
                    background_voxels_.size());
            }

            CloudT empty;
            publishCloud(empty, msg->header);
            return;
        }

        CloudT foreground;
        foreground.points.reserve(current_voxels.size());

        std::vector<VoxelKey> promote_to_background;
        promote_to_background.reserve(current_voxels.size());

        for (const auto& key : current_voxels)
        {
            if (isBackgroundVoxel(
                    key,
                    backgroundNeighborRangeForKey(key)))
            {
                foreground_states_.erase(key);
                continue;
            }

            // 未启用在线静态学习时，所有非初始背景体素都交给
            // dynamic_cluster 做时序运动确认，避免移动目标污染背景。
            if (!enable_online_static_update_)
            {
                foreground.push_back(voxelCenter(key));
                continue;
            }

            ForegroundState& state = foreground_states_[key];

            const bool continuous =
                state.last_seen_frame > 0 &&
                frame_index_ - state.last_seen_frame <=
                    static_cast<std::uint64_t>(
                        max_foreground_gap_frames_);

            state.persistent_hits =
                continuous ? state.persistent_hits + 1 : 1;
            state.last_seen_frame = frame_index_;

            if (state.persistent_hits >=
                online_static_confirm_frames_)
            {
                promote_to_background.push_back(key);
                continue;
            }

            // 这里是非背景前景候选。最终动态性由 dynamic_cluster
            // 根据整个点簇在 map 中的连续运动判断。
            foreground.push_back(voxelCenter(key));
        }

        for (const auto& key : promote_to_background)
        {
            background_voxels_.insert(key);
            foreground_states_.erase(key);
        }

        if (enable_online_static_update_)
        {
            removeStaleForegroundStates();
        }
        publishCloud(foreground, msg->header);

        ROS_INFO_THROTTLE(
            1.0,
            "[DynamicDetector] input=%zu voxels=%zu foreground=%zu "
            "promoted=%zu background=%zu pending=%zu",
            current->size(),
            current_voxels.size(),
            foreground.size(),
            promote_to_background.size(),
            background_voxels_.size(),
            foreground_states_.size());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher pub_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string output_topic_;
    std::string world_frame_;

    double voxel_size_{0.20};
    int background_frames_{30};
    int background_neighbor_range_{1};
    int online_static_confirm_frames_{20};
    bool enable_online_static_update_{false};
    int max_foreground_gap_frames_{3};

    double min_relative_z_{-0.60};
    double max_relative_z_{1.20};
    double max_range_{15.0};
    double near_field_range_{3.0};
    int near_field_background_neighbor_range_{0};

    bool background_ready_{false};
    int background_frame_count_{0};
    std::uint64_t frame_index_{0};

    std::unordered_set<VoxelKey, VoxelHash>
        background_building_voxels_;
    std::unordered_set<VoxelKey, VoxelHash>
        background_voxels_;
    std::unordered_map<VoxelKey, ForegroundState, VoxelHash>
        foreground_states_;

    bool have_odom_{false};
    double uav_x_{0.0};
    double uav_y_{0.0};
    double uav_z_{0.0};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_detector");
    DynamicDetector node;
    ros::spin();
    return 0;
}
