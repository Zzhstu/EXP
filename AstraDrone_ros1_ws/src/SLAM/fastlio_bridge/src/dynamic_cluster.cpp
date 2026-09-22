#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <vector>
#include <deque>
#include <utility>
#include <cmath>
#include <string>


class DynamicCluster
{
public:

    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;


    // =========================================================
    // Track
    // =========================================================

    struct Track
    {
        // 目标ID
        int id;

        // 当前目标位置
        // 正常情况下为滤波后的实际检测位置
        // 丢帧时为预测位置
        Eigen::Vector3d center;

        // 当前速度
        Eigen::Vector3d velocity;

        // 连续明显运动帧数
        int motion_count;

        // 连续不明显运动帧数
        int static_count;

        // 是否已经确认是动态目标
        bool dynamic;

        // 连续丢失帧数
        int missed_count;

        // 最近一次运动质量评估：直线一致性与匀速拟合残差
        double linearity;
        double fit_residual;
        double motion_speed;

        // 最近一次实际检测到目标的时间
        ros::Time stamp;

        // 最近若干帧的历史位置
        //
        // <时间戳, 滤波后的位置>
        //
        std::deque<
            std::pair<double, Eigen::Vector3d>
        > history;


        Track()
        {
            id = -1;

            center.setZero();

            velocity.setZero();

            motion_count = 0;

            static_count = 0;

            dynamic = false;

            missed_count = 0;

            linearity = 0.0;

            fit_residual = 1e9;

            motion_speed = 0.0;

            stamp = ros::Time(0);

            history.clear();
        }
    };


    // =========================================================
    // Constructor
    // =========================================================

    DynamicCluster()
        : nh_(),
          pnh_("~")
    {
        // -----------------------------------------------------
        // Subscriber
        // -----------------------------------------------------

        sub_ = nh_.subscribe(
            "/uav1/foreground_points",
            2,
            &DynamicCluster::callback,
            this
        );


        // -----------------------------------------------------
        // Publisher
        // -----------------------------------------------------

        pub_ = nh_.advertise<
            fastlio_bridge::DynamicObjectArray
        >(
            "/uav1/dynamic_objects",
            2
        );


        // 只发布已经通过整簇运动确认的动态点。
        // dynamic_detector 输出的是前景候选点，不能直接当成动态点。
        dynamic_points_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "/uav1/dynamic_points",
                2
            );


        // -----------------------------------------------------
        // 参数
        // -----------------------------------------------------

        pnh_.param<std::string>(
            "odom_topic", odom_topic_, "/uav1/fastlio/odom");
        pnh_.param("cluster_tolerance", cluster_tolerance_, 0.45);
        pnh_.param("min_cluster_size", min_cluster_size_, 4);
        pnh_.param("max_cluster_size", max_cluster_size_, 1000);
        pnh_.param("track_match_distance", track_match_distance_, 0.80);
        pnh_.param(
            "near_track_match_distance", near_track_match_distance_, 1.00);
        pnh_.param("association_position_tolerance",
                   association_position_tolerance_, 0.20);
        pnh_.param("near_association_position_tolerance",
                   near_association_position_tolerance_, 0.30);
        pnh_.param("max_association_speed", max_association_speed_, 3.00);
        pnh_.param("horizontal_motion_only",
                   horizontal_motion_only_, true);
        pnh_.param("velocity_history_size", velocity_history_size_, 12);
        pnh_.param("position_alpha", position_alpha_, 0.25);
        pnh_.param("velocity_alpha", velocity_alpha_, 0.20);
        pnh_.param(
            "motion_distance_threshold", motion_distance_threshold_, 0.10);
        pnh_.param(
            "motion_speed_threshold", motion_speed_threshold_, 0.25);
        pnh_.param("motion_history_time", motion_history_time_, 0.30);
        pnh_.param("motion_linearity_threshold",
                   motion_linearity_threshold_, 0.75);
        pnh_.param("max_velocity_fit_residual",
                   max_velocity_fit_residual_, 0.12);
        pnh_.param("dynamic_confirm_frames", dynamic_confirm_frames_, 4);
        pnh_.param("static_confirm_frames", static_confirm_frames_, 5);
        pnh_.param("max_velocity", max_velocity_, 3.0);
        pnh_.param("max_missed_frames", max_missed_frames_, 4);

        // A repeatedly observed static map provides stronger evidence than a
        // small apparent centroid shift.  This veto is aimed specifically at
        // non-repetitive LiDAR sampling on foliage, fences and thin edges.
        pnh_.param<std::string>("stable_static_map_topic",
            stable_static_map_topic_, "/uav1/stable_static_map");
        pnh_.param("static_support_radius", static_support_radius_, 0.35);
        pnh_.param("static_support_ratio", static_support_ratio_, 0.60);
        pnh_.param("static_support_min_map_points",
                   static_support_min_map_points_, 30);

        // 近场目标只需很短的历史即可确认，避免目标接近时仍卡在
        // "未确认" 状态；这里仍要求连续轨迹运动，不会把单帧杂点发布。
        pnh_.param("near_field_range", near_field_range_, 3.0);
        pnh_.param(
            "near_motion_history_time", near_motion_history_time_, 0.18);
        pnh_.param(
            "near_motion_distance_threshold",
            near_motion_distance_threshold_,
            0.07);
        pnh_.param(
            "near_motion_speed_threshold",
            near_motion_speed_threshold_,
            0.28);
        pnh_.param(
            "near_motion_linearity_threshold",
            near_motion_linearity_threshold_,
            0.65);
        pnh_.param(
            "near_max_velocity_fit_residual",
            near_max_velocity_fit_residual_,
            0.15);
        pnh_.param(
            "near_dynamic_confirm_frames",
            near_dynamic_confirm_frames_,
            4);

        odom_sub_ = nh_.subscribe(
            odom_topic_, 20, &DynamicCluster::odomCallback, this);
        stable_static_map_sub_ = nh_.subscribe(
            stable_static_map_topic_, 1,
            &DynamicCluster::stableStaticMapCallback, this);


        ROS_INFO(
            "[DynamicCluster] Started."
        );


        ROS_INFO(
            "[DynamicCluster] "
            "cluster_tolerance=%.2f "
            "min_cluster_size=%d "
            "track_match_distance=%.2f "
            "history=%d "
            "position_alpha=%.2f "
            "velocity_alpha=%.2f "
            "dynamic_confirm=%d "
            "static_confirm=%d near_range=%.2f near_match=%.2f "
            "near_confirm=%d horizontal_only=%d "
            "linearity=(%.2f/%.2f) residual=(%.2f/%.2f) "
            "max_missed=%d",
            cluster_tolerance_,
            min_cluster_size_,
            track_match_distance_,
            velocity_history_size_,
            position_alpha_,
            velocity_alpha_,
            dynamic_confirm_frames_,
            static_confirm_frames_,
            near_field_range_,
            near_track_match_distance_,
            near_dynamic_confirm_frames_,
            static_cast<int>(horizontal_motion_only_),
            motion_linearity_threshold_,
            near_motion_linearity_threshold_,
            max_velocity_fit_residual_,
            near_max_velocity_fit_residual_,
            max_missed_frames_
        );
    }


private:
    void stableStaticMapCallback(
        const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        CloudT::Ptr map(new CloudT);
        pcl::fromROSMsg(*msg, *map);
        stable_static_map_ = map;
        stable_static_tree_.reset(new pcl::search::KdTree<PointT>);
        if (!map->empty()) stable_static_tree_->setInputCloud(map);
    }

    bool hasStableStaticSupport(
        const CloudT& cloud,
        const pcl::PointIndices& cluster) const
    {
        if (!stable_static_tree_ || !stable_static_map_ ||
            stable_static_map_->size() <
                static_cast<std::size_t>(static_support_min_map_points_) ||
            cluster.indices.empty())
            return false;

        int supported = 0;
        std::vector<int> neighbors;
        std::vector<float> squared_distances;
        for (const int point_id : cluster.indices)
        {
            neighbors.clear();
            squared_distances.clear();
            if (stable_static_tree_->radiusSearch(
                    cloud.points[point_id], static_support_radius_,
                    neighbors, squared_distances, 1) > 0)
                ++supported;
        }
        const double ratio = static_cast<double>(supported) /
            static_cast<double>(cluster.indices.size());
        return ratio >= static_support_ratio_;
    }


    // =========================================================
    // 两个三维点之间的距离
    // =========================================================

    double distance(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b
    ) const
    {
        return (a - b).norm();
    }


    // 动态确认默认只使用水平XY运动。高度仍参与聚类、关联和最终
    // 三维速度输出，但不会让扫描表面的上下抖动触发动态状态。
    double motionDistance(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b
    ) const
    {
        const Eigen::Vector3d delta = a - b;

        if (horizontal_motion_only_)
        {
            return delta.head<2>().norm();
        }

        return delta.norm();
    }


    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg
    )
    {
        uav_position_ <<
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;

        have_odom_ = true;
    }


    bool isNearField(
        const Eigen::Vector3d& position
    ) const
    {
        if (!have_odom_ || near_field_range_ <= 0.0)
        {
            return false;
        }

        const Eigen::Vector2d horizontal_delta(
            position.x() - uav_position_.x(),
            position.y() - uav_position_.y());

        return horizontal_delta.norm() < near_field_range_;
    }


    // =========================================================
    // 多帧最小二乘拟合速度
    //
    // p(t) = v*t + p0
    // =========================================================

    Eigen::Vector3d estimateVelocity(
        const Track& track
    ) const
    {
        const size_t N =
            track.history.size();


        // 至少3个历史点
        if (N < 3)
        {
            return Eigen::Vector3d::Zero();
        }


        // -----------------------------------------------------
        // 以第一帧作为时间零点
        // -----------------------------------------------------

        const double t0 =
            track.history.front().first;


        double sum_t = 0.0;

        double sum_t2 = 0.0;


        Eigen::Vector3d sum_p =
            Eigen::Vector3d::Zero();


        Eigen::Vector3d sum_tp =
            Eigen::Vector3d::Zero();


        // -----------------------------------------------------
        // 累加
        // -----------------------------------------------------

        for (const auto& sample :
             track.history)
        {
            const double t =
                sample.first - t0;


            const Eigen::Vector3d& p =
                sample.second;


            sum_t += t;

            sum_t2 += t * t;

            sum_p += p;

            sum_tp += t * p;
        }


        const double n =
            static_cast<double>(N);


        const double denominator =
            n * sum_t2 -
            sum_t * sum_t;


        if (
            std::fabs(denominator) < 1e-8
        )
        {
            return Eigen::Vector3d::Zero();
        }


        Eigen::Vector3d velocity =
            (
                n * sum_tp -
                sum_t * sum_p
            )
            /
            denominator;


        return velocity;
    }


    // =========================================================
    // 轨迹直线一致性
    //
    // 真实短时运动通常具有稳定方向；扫描边界噪声往往来回跳动。
    // 返回值接近1表示路径接近直线，接近0表示主要是抖动。
    // =========================================================

    double trajectoryLinearity(const Track& track) const
    {
        if (track.history.size() < 3)
        {
            return 0.0;
        }

        double path_length = 0.0;

        for (size_t i = 1; i < track.history.size(); ++i)
        {
            path_length += motionDistance(
                track.history[i].second,
                track.history[i - 1].second);
        }

        if (path_length < 1e-6)
        {
            return 0.0;
        }

        const double net_displacement = motionDistance(
            track.history.back().second,
            track.history.front().second);

        return std::min(1.0, net_displacement / path_length);
    }


    // =========================================================
    // 匀速模型拟合残差
    //
    // p(t) = p0 + v*t。残差越小，越像真实连续运动；残差较大通常
    // 表示簇中心由不同扫描边界点拼接而成。
    // =========================================================

    double velocityFitResidual(
        const Track& track,
        const Eigen::Vector3d& velocity
    ) const
    {
        const size_t count = track.history.size();

        if (count < 3)
        {
            return 1e9;
        }

        const double t0 = track.history.front().first;
        double mean_t = 0.0;
        Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();

        for (const auto& sample : track.history)
        {
            mean_t += sample.first - t0;
            mean_position += sample.second;
        }

        mean_t /= static_cast<double>(count);
        mean_position /= static_cast<double>(count);

        const Eigen::Vector3d intercept =
            mean_position - velocity * mean_t;

        double squared_error_sum = 0.0;

        for (const auto& sample : track.history)
        {
            const double t = sample.first - t0;
            const Eigen::Vector3d predicted = intercept + velocity * t;
            Eigen::Vector3d error = sample.second - predicted;

            if (horizontal_motion_only_)
            {
                error.z() = 0.0;
            }

            squared_error_sum += error.squaredNorm();
        }

        return std::sqrt(
            squared_error_sum / static_cast<double>(count));
    }


    // =========================================================
    // 发布已经通过动态确认的点，而不是全部前景候选点
    // =========================================================

    void publishDynamicPoints(
        const CloudT& cloud,
        const std_msgs::Header& header
    )
    {
        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(
            cloud,
            output
        );

        output.header =
            header;

        output.header.frame_id =
            "map";

        dynamic_points_pub_.publish(
            output
        );
    }


    // =========================================================
    // 预测未来位置
    //
    // p_predict = p + v*dt
    // =========================================================

    Eigen::Vector3d predictPosition(
        const Track& track,
        const ros::Time& now
    ) const
    {
        double dt =
            (
                now -
                track.stamp
            ).toSec();


        if (dt < 0.0)
        {
            dt = 0.0;
        }


        return
            track.center +
            track.velocity * dt;
    }


    // =========================================================
    // Callback
    // =========================================================

    void callback(
        const sensor_msgs::PointCloud2::ConstPtr& msg
    )
    {
        // =====================================================
        // 1. ROS PointCloud2 -> PCL
        // =====================================================

        CloudT::Ptr cloud(
            new CloudT
        );


        pcl::fromROSMsg(
            *msg,
            *cloud
        );


        fastlio_bridge::DynamicObjectArray output;


        output.header =
            msg->header;


        output.header.frame_id =
            "map";


        CloudT confirmed_dynamic_points;


        // =====================================================
        // 2. 空点云
        // =====================================================

        if (cloud->empty())
        {
            handleNoDetection(
                msg,
                output
            );

            pub_.publish(
                output
            );


            publishDynamicPoints(
                confirmed_dynamic_points,
                msg->header
            );

            return;
        }


        // =====================================================
        // 3. 欧式聚类
        // =====================================================

        pcl::search::KdTree<PointT>::Ptr tree(
            new pcl::search::KdTree<PointT>
        );


        tree->setInputCloud(
            cloud
        );


        std::vector<pcl::PointIndices> clusters;


        pcl::EuclideanClusterExtraction<PointT> ec;


        ec.setClusterTolerance(
            cluster_tolerance_
        );


        ec.setMinClusterSize(
            min_cluster_size_
        );


        ec.setMaxClusterSize(
            max_cluster_size_
        );


        ec.setSearchMethod(
            tree
        );


        ec.setInputCloud(
            cloud
        );


        ec.extract(
            clusters
        );


        // =====================================================
        // 4. 计算cluster中心
        // =====================================================

        std::vector<Eigen::Vector3d> centers;


        // centers[i] 对应 clusters[center_cluster_indices[i]]。
        // 显式保存关系，后面才能只输出已确认动态簇的原始点。
        std::vector<size_t> center_cluster_indices;


        for (
            size_t cluster_index = 0;
            cluster_index < clusters.size();
            ++cluster_index
        )
        {
            const auto& indices =
                clusters[cluster_index];

            if (
                static_cast<int>(
                    indices.indices.size()
                )
                <
                min_cluster_size_
            )
            {
                continue;
            }


            Eigen::Vector3d raw_center =
                Eigen::Vector3d::Zero();


            for (
                const auto point_id :
                indices.indices
            )
            {
                const auto& p =
                    cloud->points[point_id];


                raw_center.x() += p.x;

                raw_center.y() += p.y;

                raw_center.z() += p.z;
            }


            raw_center /=
                static_cast<double>(
                    indices.indices.size()
                );


            centers.push_back(
                raw_center
            );


            center_cluster_indices.push_back(
                cluster_index
            );
        }


        // =====================================================
        // 5. 如果聚类失败
        // =====================================================

        if (centers.empty())
        {
            handleNoDetection(
                msg,
                output
            );


            pub_.publish(
                output
            );


            publishDynamicPoints(
                confirmed_dynamic_points,
                msg->header
            );


            ROS_INFO_THROTTLE(
                1.0,
                "[DynamicCluster] "
                "No cluster detected."
            );


            return;
        }


        // =====================================================
        // 6. 当前帧Track匹配状态
        // =====================================================

        std::vector<bool> matched(
            tracks_.size(),
            false
        );


        // =====================================================
        // 7. 当前检测到的clusters进行Track匹配
        // =====================================================

        for (
            size_t center_index = 0;
            center_index < centers.size();
            ++center_index
        )
        {
            const Eigen::Vector3d& raw_center =
                centers[center_index];


            const pcl::PointIndices& current_cluster =
                clusters[
                    center_cluster_indices[center_index]
                ];

            const bool stable_static_support =
                hasStableStaticSupport(*cloud, current_cluster);

            const bool near_field =
                isNearField(raw_center);

            int best =
                -1;


            double min_dist =
                near_field
                    ? near_track_match_distance_
                    : track_match_distance_;

            const double association_tolerance =
                near_field
                    ? near_association_position_tolerance_
                    : association_position_tolerance_;


            // -------------------------------------------------
            // 搜索最近Track
            // -------------------------------------------------

            for (
                size_t i = 0;
                i < tracks_.size();
                ++i
            )
            {
                if (
                    matched[i]
                )
                {
                    continue;
                }


                double association_dt =
                    (msg->header.stamp - tracks_[i].stamp).toSec();

                if (association_dt < 0.0)
                {
                    association_dt = 0.0;
                }

                const Eigen::Vector3d predicted_center =
                    predictPosition(tracks_[i], msg->header.stamp);

                const double association_gate = std::min(
                    min_dist,
                    association_tolerance +
                        max_association_speed_ * association_dt);

                const double d =
                    distance(raw_center, predicted_center);


                if (
                    d < association_gate
                )
                {
                    min_dist =
                        d;


                    best =
                        static_cast<int>(i);
                }
            }


            // =================================================
            // 8. 匹配到历史Track
            // =================================================

            if (best >= 0)
            {
                Track& track =
                    tracks_[best];


                matched[best] =
                    true;


                // -------------------------------------------------
                // 当前帧重新检测到
                // -------------------------------------------------

                track.missed_count =
                    0;


                // -------------------------------------------------
                // 原始cluster中心
                // -------------------------------------------------

                Eigen::Vector3d raw_position =
                    raw_center;


                // -------------------------------------------------
                // 位置低通滤波
                //
                // filtered =
                //     0.8 * old
                //   + 0.2 * measurement
                // -------------------------------------------------

                Eigen::Vector3d filtered_position =
                    (
                        1.0 -
                        position_alpha_
                    )
                    *
                    track.center
                    +
                    position_alpha_
                    *
                    raw_position;


                // -------------------------------------------------
                // 记录历史
                // -------------------------------------------------

                track.history.emplace_back(
                    msg->header.stamp.toSec(),
                    filtered_position
                );


                // -------------------------------------------------
                // 限制历史长度
                // -------------------------------------------------

                while (
                    static_cast<int>(
                        track.history.size()
                    )
                    >
                    velocity_history_size_
                )
                {
                    track.history.pop_front();
                }


                // -------------------------------------------------
                // 多帧拟合速度
                // -------------------------------------------------

                Eigen::Vector3d fitted_velocity =
                    estimateVelocity(track);


                double fitted_speed =
                    fitted_velocity.norm();

                const bool fitted_velocity_valid =
                    std::isfinite(fitted_speed) &&
                    fitted_speed < max_velocity_;

                track.motion_speed = horizontal_motion_only_
                    ? fitted_velocity.head<2>().norm()
                    : fitted_speed;


                // -------------------------------------------------
                // 速度异常过滤 + 低通
                // -------------------------------------------------

                if (
                    fitted_velocity_valid
                )
                {
                    track.velocity =
                        (
                            1.0 -
                            velocity_alpha_
                        )
                        *
                        track.velocity
                        +
                        velocity_alpha_
                        *
                        fitted_velocity;
                }
                else
                {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[DynamicCluster] "
                        "Abnormal fitted speed: %.2f",
                        fitted_speed
                    );
                }


                // -------------------------------------------------
                // 历史窗口总位移与持续时间
                // -------------------------------------------------

                double history_move = 0.0;

                double history_dt = 0.0;


                if (track.history.size() >= 3)
                {
                    history_move =
                        motionDistance(
                            track.history.back().second,
                            track.history.front().second
                        );


                    history_dt =
                        track.history.back().first -
                        track.history.front().first;
                }


                // -------------------------------------------------
                // 动态判断
                // -------------------------------------------------

                const double required_history_time =
                    near_field
                        ? near_motion_history_time_
                        : motion_history_time_;

                const double required_motion_distance =
                    near_field
                        ? near_motion_distance_threshold_
                        : motion_distance_threshold_;

                const double required_motion_speed =
                    near_field
                        ? near_motion_speed_threshold_
                        : motion_speed_threshold_;

                const int required_confirm_frames =
                    near_field
                        ? near_dynamic_confirm_frames_
                        : dynamic_confirm_frames_;

                const double required_linearity =
                    near_field
                        ? near_motion_linearity_threshold_
                        : motion_linearity_threshold_;

                const double allowed_fit_residual =
                    near_field
                        ? near_max_velocity_fit_residual_
                        : max_velocity_fit_residual_;

                track.linearity = trajectoryLinearity(track);
                track.fit_residual =
                    velocityFitResidual(track, fitted_velocity);

                bool moving =
                    (
                        !stable_static_support
                        &&
                        fitted_velocity_valid
                        &&
                        history_dt >= required_history_time
                        &&
                        history_move >
                        required_motion_distance
                        &&
                        track.motion_speed >
                        required_motion_speed
                        &&
                        track.linearity >= required_linearity
                        &&
                        track.fit_residual <= allowed_fit_residual
                    );


                if (moving)
                {
                    track.motion_count++;

                    track.static_count =
                        0;
                }
                else
                {
                    track.motion_count =
                        0;


                    if (track.dynamic)
                    {
                        track.static_count++;
                    }
                    else
                    {
                        track.static_count =
                            0;
                    }
                }


                // -------------------------------------------------
                // 连续运动确认动态
                // -------------------------------------------------

                if (
                    track.motion_count >=
                    required_confirm_frames
                )
                {
                    track.dynamic =
                        true;


                    track.static_count =
                        0;
                }


                // -------------------------------------------------
                // 连续10帧不明显运动 -> 静态
                // -------------------------------------------------

                if (
                    track.dynamic &&
                    track.static_count >=
                    static_confirm_frames_
                )
                {
                    track.dynamic =
                        false;


                    track.motion_count =
                        0;


                    track.static_count =
                        0;


                    ROS_INFO(
                        "[DynamicCluster] "
                        "Track id=%d became static",
                        track.id
                    );
                }


                // -------------------------------------------------
                // 更新实际位置
                // -------------------------------------------------

                track.center =
                    filtered_position;


                // -------------------------------------------------
                // 更新时间
                // -------------------------------------------------

                track.stamp =
                    msg->header.stamp;


                // =================================================
                // 9. 输出动态目标
                // =================================================

                if (
                    track.dynamic
                )
                {
                    publishObject(
                        output,
                        track,
                        false
                    );


                    for (
                        const int point_id :
                        current_cluster.indices
                    )
                    {
                        confirmed_dynamic_points.push_back(
                            cloud->points[point_id]
                        );
                    }
                }
            }


            // =================================================
            // 10. 没匹配到旧Track
            // =================================================

            else
{
    Track new_track;

    new_track.id =
        next_id_++;

    new_track.center =
        raw_center;

    new_track.velocity =
        Eigen::Vector3d::Zero();

    new_track.motion_count =
        0;

    new_track.static_count =
        0;

    new_track.dynamic =
        false;

    new_track.missed_count =
        0;

    new_track.stamp =
        msg->header.stamp;

    new_track.history.emplace_back(
        msg->header.stamp.toSec(),
        raw_center
    );

    tracks_.push_back(
        new_track
    );

    // =====================================================
    // 关键修复
    //
    // tracks_增加一个元素后，
    // matched也必须同步增加一个元素。
    //
    // 当前这个new track已经被当前cluster使用，
    // 所以标记为true。
    // =====================================================

    matched.push_back(true);

    ROS_INFO(
        "[DynamicCluster] "
        "New track id=%d "
        "center=(%.2f %.2f %.2f)",
        new_track.id,
        raw_center.x(),
        raw_center.y(),
        raw_center.z()
    );
}
        }


        // =====================================================
        // 11. 对没有匹配到的Track进行预测
        // =====================================================

        for (
            size_t i = 0;
            i < tracks_.size();
            ++i
        )
        {
            // 已经匹配
            if (
                i < matched.size() &&
                matched[i]
            )
            {
                continue;
            }


            Track& track =
                tracks_[i];


            // 还没有确认动态的Track
            // 不进行预测发布
            if (
                !track.dynamic
            )
            {
                track.missed_count++;

                continue;
            }


            // -------------------------------------------------
            // 丢失一帧
            // -------------------------------------------------

            track.missed_count++;


            // -------------------------------------------------
            // 预测当前时刻位置
            // -------------------------------------------------

            Eigen::Vector3d predicted_center =
                predictPosition(
                    track,
                    msg->header.stamp
                );


            // -------------------------------------------------
            // 不修改真实检测中心和时间戳
            //
            // center/stamp始终表示最后一次真实检测。若每次预测都把
            // center向前推进、stamp却不变，下一帧会重复累计整段dt。
            // -------------------------------------------------

            if (
                track.missed_count <=
                max_missed_frames_
            )
            {
                Track predicted_track = track;
                predicted_track.center = predicted_center;

                publishObject(output, predicted_track, true);


                ROS_INFO_THROTTLE(
                    0.5,
                    "[DynamicCluster] "
                    "Predict id=%d "
                    "missed=%d "
                    "v=%.2f "
                    "center=(%.2f %.2f %.2f)",
                    track.id,
                    track.missed_count,
                    track.velocity.norm(),
                    predicted_center.x(),
                    predicted_center.y(),
                    predicted_center.z()
                );
            }
        }


        // =====================================================
        // 12. 删除长期没有检测到的Track
        // =====================================================

        removeStaleTracks();


        // =====================================================
        // 13. 发布
        // =====================================================

        pub_.publish(
            output
        );


        publishDynamicPoints(
            confirmed_dynamic_points,
            msg->header
        );


        // =====================================================
        // 14. 调试信息
        // =====================================================

        ROS_INFO_THROTTLE(
            1.0,
            "[DynamicCluster] "
            "Input=%zu "
            "clusters=%zu "
            "centers=%zu "
            "tracks=%zu "
            "objects=%zu",
            cloud->size(),
            clusters.size(),
            centers.size(),
            tracks_.size(),
            output.objects.size()
        );
    }


    // =========================================================
    // 没有检测到cluster时
    // =========================================================

    void handleNoDetection(
        const sensor_msgs::PointCloud2::ConstPtr& msg,
        fastlio_bridge::DynamicObjectArray& output
    )
    {
        for (
            auto& track :
            tracks_
        )
        {
            track.missed_count++;


            // 只有确认动态目标才进行预测
            if (
                !track.dynamic
            )
            {
                continue;
            }


            // -------------------------------------------------
            // 超过最大允许丢失帧数
            // -------------------------------------------------

            if (
                track.missed_count >
                max_missed_frames_
            )
            {
                continue;
            }


            // -------------------------------------------------
            // 预测位置
            // -------------------------------------------------

            Eigen::Vector3d predicted_center =
                predictPosition(
                    track,
                    msg->header.stamp
                );


            // -------------------------------------------------
            // 发布预测目标
            // -------------------------------------------------

            Track predicted_track = track;
            predicted_track.center = predicted_center;

            publishObject(output, predicted_track, true);


            ROS_INFO_THROTTLE(
                0.5,
                "[DynamicCluster] "
                "Predict(no cluster) "
                "id=%d "
                "missed=%d "
                "v=%.2f "
                "center=(%.2f %.2f %.2f)",
                track.id,
                track.missed_count,
                track.velocity.norm(),
                predicted_center.x(),
                predicted_center.y(),
                predicted_center.z()
            );
        }


        removeStaleTracks();
    }


    // =========================================================
    // 发布 DynamicObject
    // =========================================================

    void publishObject(
        fastlio_bridge::DynamicObjectArray& output,
        const Track& track,
        bool predicted
    )
    {
        fastlio_bridge::DynamicObject obj;


        // -----------------------------------------------------
        // ID
        // -----------------------------------------------------

        obj.id =
            track.id;


        // -----------------------------------------------------
        // Position
        // -----------------------------------------------------

        obj.pose.position.x =
            track.center.x();


        obj.pose.position.y =
            track.center.y();


        obj.pose.position.z =
            track.center.z();


        // -----------------------------------------------------
        // Orientation
        // -----------------------------------------------------

        obj.pose.orientation.x =
            0.0;


        obj.pose.orientation.y =
            0.0;


        obj.pose.orientation.z =
            0.0;


        obj.pose.orientation.w =
            1.0;


        // -----------------------------------------------------
        // Velocity
        // -----------------------------------------------------

        obj.twist.linear.x =
            track.velocity.x();


        obj.twist.linear.y =
            track.velocity.y();


        obj.twist.linear.z =
            track.velocity.z();

        // V1 messages reserve size/semantic fields for downstream fusion.
        // The current clustering stage has no stable oriented box yet, so a
        // conservative default volume is used instead of inventing a class.
        obj.size.x = 0.60;
        obj.size.y = 0.60;
        obj.size.z = 1.20;
        obj.confidence = static_cast<float>(
            std::max(0.0, std::min(1.0, track.linearity)));
        obj.semantic_class = 0;
        obj.predicted = predicted;


        // -----------------------------------------------------
        // 加入输出
        // -----------------------------------------------------

        output.objects.push_back(
            obj
        );


        // -----------------------------------------------------
        // 调试
        // -----------------------------------------------------

        if (!predicted)
        {
            ROS_INFO(
                "[DynamicCluster] "
                "Dynamic id=%d "
                "v=%.2f motion_v=%.2f "
                "linearity=%.2f residual=%.3f "
                "motion=%d "
                "static=%d "
                "missed=%d "
                "center=(%.2f %.2f %.2f)",
                track.id,
                track.velocity.norm(),
                track.motion_speed,
                track.linearity,
                track.fit_residual,
                track.motion_count,
                track.static_count,
                track.missed_count,
                track.center.x(),
                track.center.y(),
                track.center.z()
            );
        }
    }


    // =========================================================
    // 删除长期没有检测到的Track
    // =========================================================

    void removeStaleTracks()
    {
        for (
            size_t i = 0;
            i < tracks_.size();
        )
        {
            if (
                tracks_[i].missed_count >
                max_missed_frames_
            )
            {
                ROS_INFO(
                    "[DynamicCluster] "
                    "Remove stale track id=%d "
                    "missed=%d",
                    tracks_[i].id,
                    tracks_[i].missed_count
                );


                tracks_.erase(
                    tracks_.begin() + i
                );
            }
            else
            {
                ++i;
            }
        }
    }


private:

    // =========================================================
    // ROS
    // =========================================================

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Subscriber sub_;

    ros::Subscriber odom_sub_;

    ros::Subscriber stable_static_map_sub_;

    ros::Publisher pub_;

    ros::Publisher dynamic_points_pub_;


    // =========================================================
    // Track
    // =========================================================

    std::vector<Track> tracks_;


    int next_id_ = 0;


    // =========================================================
    // Clustering parameters
    // =========================================================

    double cluster_tolerance_;

    int min_cluster_size_;

    int max_cluster_size_;


    // =========================================================
    // Tracking parameters
    // =========================================================

    std::string odom_topic_;

    std::string stable_static_map_topic_;

    double static_support_radius_{0.35};

    double static_support_ratio_{0.60};

    int static_support_min_map_points_{30};

    CloudT::Ptr stable_static_map_;

    pcl::search::KdTree<PointT>::Ptr stable_static_tree_;

    double track_match_distance_;

    double near_track_match_distance_;

    double association_position_tolerance_;

    double near_association_position_tolerance_;

    double max_association_speed_;

    bool horizontal_motion_only_;


    // =========================================================
    // Position / velocity
    // =========================================================

    int velocity_history_size_;

    double position_alpha_;

    double velocity_alpha_;


    // =========================================================
    // Dynamic state
    // =========================================================

    double motion_distance_threshold_;

    double motion_speed_threshold_;

    double motion_history_time_;

    double motion_linearity_threshold_;

    double max_velocity_fit_residual_;


    int dynamic_confirm_frames_;

    int static_confirm_frames_;


    // =========================================================
    // Velocity limit
    // =========================================================

    double max_velocity_;


    // =========================================================
    // Lost track
    // =========================================================

    int max_missed_frames_;


    // =========================================================
    // Near field / UAV state
    // =========================================================

    double near_field_range_;

    double near_motion_history_time_;

    double near_motion_distance_threshold_;

    double near_motion_speed_threshold_;

    double near_motion_linearity_threshold_;

    double near_max_velocity_fit_residual_;

    int near_dynamic_confirm_frames_;

    bool have_odom_{false};

    Eigen::Vector3d uav_position_{Eigen::Vector3d::Zero()};
};


// =============================================================
// Main
// =============================================================

int main(
    int argc,
    char** argv
)
{
    ros::init(
        argc,
        argv,
        "dynamic_cluster"
    );


    DynamicCluster node;


    ros::spin();


    return 0;
}
