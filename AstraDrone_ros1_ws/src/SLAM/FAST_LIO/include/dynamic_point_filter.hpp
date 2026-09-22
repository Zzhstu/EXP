#pragma once

// Geometry-only moving point filter inspired by LMNet's ego-motion compensated
// temporal residuals.  It deliberately has no neural-network dependency.
//
// IMPORTANT:
//   1. Input clouds must already be deskewed and transformed into one fixed
//      world frame. FAST-LIO calls this after the iterated Kalman update.
//   2. UNKNOWN points are intentionally not inserted into the map. A new wall
//      therefore needs several observations before it is accepted as static.
//   3. Pure motion reasoning cannot distinguish a permanently stationary
//      person from a wall. Start mapping with a mostly static scene whenever
//      possible.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace fast_lio_dynamic
{

enum class PointLabel : std::uint8_t
{
    STATIC = 0,
    DYNAMIC = 1,
    UNKNOWN = 2
};

struct DynamicFilterParams
{
    bool enabled = true;
    int history_frames = 5;
    int min_history_frames = 4;
    int min_static_observations = 3;
    int min_changed_observations = 3;
    double voxel_size = 0.20;
    double residual_min = 0.20;
    double residual_max = 0.60;
    double residual_range_ratio = 0.01;
    // Nearby-but-not-matching observations are motion evidence. This gate is
    // needed because MID360 uses a non-repetitive sampling pattern.
    double motion_gate = 0.90;
    double map_static_distance = 0.25;
    double cluster_tolerance = 0.60;
    int min_cluster_points = 3;
};

struct VoxelKey
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey &key) const
    {
        // Three large odd constants provide a stable spatial hash and work for
        // negative indices after conversion to unsigned integers.
        const std::size_t hx = std::hash<std::int64_t>()(key.x);
        const std::size_t hy = std::hash<std::int64_t>()(key.y);
        const std::size_t hz = std::hash<std::int64_t>()(key.z);
        return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6U) + (hx >> 2U))
                  ^ (hz + 0x9e3779b97f4a7c15ULL + (hy << 6U) + (hy >> 2U));
    }
};

template <typename PointT>
class GeometricDynamicFilter
{
public:
    using Cloud = pcl::PointCloud<PointT>;
    using CloudPtr = typename Cloud::Ptr;

    void configure(const DynamicFilterParams &params)
    {
        params_ = params;
        params_.history_frames = std::max(1, params_.history_frames);
        params_.min_history_frames =
            std::max(1, std::min(params_.min_history_frames, params_.history_frames));
        params_.min_static_observations =
            std::max(1, std::min(params_.min_static_observations, params_.history_frames));
        params_.min_changed_observations =
            std::max(1, std::min(params_.min_changed_observations, params_.history_frames));
        params_.voxel_size = std::max(0.02, params_.voxel_size);
        params_.residual_min = std::max(0.01, params_.residual_min);
        params_.residual_max = std::max(params_.residual_min, params_.residual_max);
        params_.motion_gate = std::max(params_.residual_max, params_.motion_gate);
        params_.cluster_tolerance = std::max(params_.voxel_size, params_.cluster_tolerance);
        params_.min_cluster_points = std::max(1, params_.min_cluster_points);
        reset();
    }

    void reset()
    {
        history_.clear();
    }

    bool enabled() const { return params_.enabled; }

    // During warm-up, incremental insertion should be frozen. The initial
    // ikd-tree is still available to FAST-LIO for pose estimation.
    bool ready() const
    {
        return !params_.enabled ||
               static_cast<int>(history_.size()) >= params_.min_history_frames;
    }

    double mapStaticDistance() const { return params_.map_static_distance; }

    void seed(const Cloud &world_cloud)
    {
        if (!params_.enabled)
            return;
        history_.clear();
        history_.push_back(buildGrid(world_cloud));
    }

    std::vector<PointLabel> classifyAndUpdate(
        const Cloud &world_cloud,
        const std::vector<bool> &map_supported,
        const Eigen::Vector3d &sensor_origin)
    {
        std::vector<PointLabel> labels(world_cloud.size(), PointLabel::STATIC);
        if (!params_.enabled || world_cloud.empty())
            return labels;

        std::vector<int> dynamic_candidates;
        dynamic_candidates.reserve(world_cloud.size() / 8U + 1U);

        for (std::size_t i = 0; i < world_cloud.size(); ++i)
        {
            const PointT &point = world_cloud.points[i];
            const Eigen::Vector3d p(point.x, point.y, point.z);
            const double range = (p - sensor_origin).norm();

            // The threshold grows slightly with range to tolerate angular
            // sampling sparsity and small pose errors at long distance.
            const double threshold = std::min(
                params_.residual_max,
                std::max(params_.residual_min, params_.residual_range_ratio * range));

            int support_count = 0;
            int motion_count = 0;
            for (const FrameGrid &frame : history_)
            {
                if (hasTemporalSupport(frame, p, threshold))
                    ++support_count;
                else if (hasNearbyNonMatchingPoint(frame, p, threshold))
                    ++motion_count;
            }

            const int history_count = static_cast<int>(history_.size());
            const bool enough_history = history_count >= params_.min_history_frames;

            const bool map_consistent =
                i < map_supported.size() && map_supported[i];

            // A moving object can still overlap the old map and can retain a
            // few exact temporal matches while it moves. Once the temporal
            // evidence is strong across several frames, it must override the
            // old map match; otherwise a person inserted in an early scan is
            // permanently accepted as static.
            const bool strong_temporal_motion =
                enough_history &&
                support_count < params_.min_static_observations &&
                motion_count >= params_.min_changed_observations;

            if (strong_temporal_motion)
            {
                labels[i] = PointLabel::DYNAMIC;
                dynamic_candidates.push_back(static_cast<int>(i));
            }
            else if (map_consistent ||
                     support_count >= params_.min_static_observations)
            {
                labels[i] = PointLabel::STATIC;
            }
            else
            {
                // New points, occlusion boundaries and scan-pattern changes
                // are not dynamic evidence by themselves. UNKNOWN points
                // never enter the map.
                labels[i] = PointLabel::UNKNOWN;
            }
        }

        rejectSmallDynamicClusters(world_cloud, dynamic_candidates, labels);
        history_.push_back(buildGrid(world_cloud));
        while (static_cast<int>(history_.size()) > params_.history_frames)
            history_.pop_front();

        return labels;
    }

private:
    struct VoxelAccumulator
    {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
    };

    using FrameGrid = std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash>;

    VoxelKey keyFor(const Eigen::Vector3d &p) const
    {
        return VoxelKey{
            static_cast<std::int64_t>(std::floor(p.x() / params_.voxel_size)),
            static_cast<std::int64_t>(std::floor(p.y() / params_.voxel_size)),
            static_cast<std::int64_t>(std::floor(p.z() / params_.voxel_size))};
    }

    FrameGrid buildGrid(const Cloud &cloud) const
    {
        std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> accumulators;
        accumulators.reserve(cloud.size());
        for (const PointT &point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            const Eigen::Vector3d p(point.x, point.y, point.z);
            VoxelAccumulator &acc = accumulators[keyFor(p)];
            acc.sum += p;
            ++acc.count;
        }

        FrameGrid grid;
        grid.reserve(accumulators.size());
        for (const auto &entry : accumulators)
            grid.emplace(entry.first, entry.second.sum / std::max(1, entry.second.count));
        return grid;
    }

    bool hasTemporalSupport(
        const FrameGrid &frame,
        const Eigen::Vector3d &point,
        double threshold) const
    {
        const VoxelKey center = keyFor(point);
        const int radius = std::max(1, static_cast<int>(std::ceil(threshold / params_.voxel_size)));
        const double threshold_sq = threshold * threshold;

        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
                    const auto it = frame.find(key);
                    if (it != frame.end() && (it->second - point).squaredNorm() <= threshold_sq)
                        return true;
                }
            }
        }
        return false;
    }

    bool hasNearbyNonMatchingPoint(
        const FrameGrid &frame,
        const Eigen::Vector3d &point,
        double same_point_threshold) const
    {
        const VoxelKey center = keyFor(point);
        const int radius = std::max(
            1, static_cast<int>(std::ceil(params_.motion_gate / params_.voxel_size)));
        const double same_sq = same_point_threshold * same_point_threshold;
        const double gate_sq = params_.motion_gate * params_.motion_gate;

        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
                    const auto it = frame.find(key);
                    if (it == frame.end())
                        continue;

                    const double distance_sq = (it->second - point).squaredNorm();
                    if (distance_sq > same_sq && distance_sq <= gate_sq)
                        return true;
                }
            }
        }
        return false;
    }

    void rejectSmallDynamicClusters(
        const Cloud &world_cloud,
        const std::vector<int> &candidate_indices,
        std::vector<PointLabel> &labels) const
    {
        if (candidate_indices.empty() || params_.min_cluster_points <= 1)
            return;

        if (static_cast<int>(candidate_indices.size()) < params_.min_cluster_points)
        {
            for (int index : candidate_indices)
                labels[static_cast<std::size_t>(index)] = PointLabel::UNKNOWN;
            return;
        }

        CloudPtr candidates(new Cloud());
        candidates->reserve(candidate_indices.size());
        for (int index : candidate_indices)
            candidates->push_back(world_cloud.points[static_cast<std::size_t>(index)]);
        candidates->width = static_cast<std::uint32_t>(candidates->size());
        candidates->height = 1;

        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(candidates);
        pcl::EuclideanClusterExtraction<PointT> extraction;
        extraction.setClusterTolerance(params_.cluster_tolerance);
        extraction.setMinClusterSize(params_.min_cluster_points);
        extraction.setMaxClusterSize(static_cast<int>(candidates->size()));
        extraction.setSearchMethod(tree);
        extraction.setInputCloud(candidates);

        std::vector<pcl::PointIndices> clusters;
        extraction.extract(clusters);
        std::vector<bool> accepted(candidate_indices.size(), false);
        for (const pcl::PointIndices &cluster : clusters)
            for (int local_index : cluster.indices)
                accepted[static_cast<std::size_t>(local_index)] = true;

        for (std::size_t i = 0; i < candidate_indices.size(); ++i)
        {
            if (!accepted[i])
                labels[static_cast<std::size_t>(candidate_indices[i])] = PointLabel::UNKNOWN;
        }
    }

    DynamicFilterParams params_;
    std::deque<FrameGrid> history_;
};

}  // namespace fast_lio_dynamic
