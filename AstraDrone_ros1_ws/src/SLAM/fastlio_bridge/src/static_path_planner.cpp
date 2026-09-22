#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>

// Lightweight map-aware planner for the demo controller.
//
// Hierarchical two-map planner:
//   1. a cleaned, persistent global map plans toward the final destination;
//   2. a short-lived rolling local map replans toward a point on that route.
// Transient returns can therefore protect the UAV nearby without becoming a
// permanent wall.  The controller still retains its fast reactive safety layer.
class StaticPathPlanner
{
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    StaticPathPlanner() : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("map_topic", map_topic_,
                                "/uav1/stable_static_map");
        pnh_.param<std::string>("local_map_topic", local_map_topic_,
                                "/uav1/local_static_map");
        pnh_.param<std::string>("local_free_space_topic",
                                local_free_space_topic_,
                                "/uav1/local_free_space");
        pnh_.param<std::string>("odom_topic", odom_topic_,
                                "/uav1/fastlio/odom");
        pnh_.param<std::string>("goal_topic", goal_topic_,
                                "/move_base_simple/goal");
        pnh_.param<std::string>("waypoint_topic", waypoint_topic_,
                                "/uav1/planner/waypoint");
        pnh_.param<std::string>("path_topic", path_topic_,
                                "/uav1/planner/path");
        pnh_.param<std::string>("local_path_topic", local_path_topic_,
                                "/uav1/planner/local_path");
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param("resolution", resolution_, 0.25);
        pnh_.param("inflation_radius", inflation_radius_, 0.95);
        pnh_.param("planning_margin", planning_margin_, 4.0);
        pnh_.param("local_planning_margin", local_planning_margin_, 2.5);
        pnh_.param("local_planning_horizon", local_planning_horizon_, 4.0);
        pnh_.param("local_replan_to_final_goal",
                   local_replan_to_final_goal_, true);
        pnh_.param("local_free_clear_radius",
                   local_free_clear_radius_, 0.15);
        pnh_.param("obstacle_min_relative_z", obstacle_min_relative_z_, -0.80);
        pnh_.param("obstacle_max_relative_z", obstacle_max_relative_z_, 0.80);
        pnh_.param("lookahead_distance", lookahead_distance_, 1.0);
        pnh_.param("goal_tolerance", goal_tolerance_, 0.30);
        pnh_.param("start_clearance_radius", start_clearance_radius_, 0.35);
        pnh_.param("goal_search_radius", goal_search_radius_, 1.5);
        pnh_.param("minimum_map_points", minimum_map_points_, 30);
        pnh_.param("replan_rate", replan_rate_, 2.0);
        pnh_.param("max_grid_cells", max_grid_cells_, 250000);

        resolution_ = std::max(0.10, resolution_);
        inflation_radius_ = std::max(0.0, inflation_radius_);
        planning_margin_ = std::max(1.0, planning_margin_);
        local_planning_margin_ = std::max(1.0, local_planning_margin_);
        lookahead_distance_ = std::max(resolution_, lookahead_distance_);
        local_planning_horizon_ = std::max(lookahead_distance_,
                                           local_planning_horizon_);
        local_free_clear_radius_ = std::max(0.0, local_free_clear_radius_);
        replan_rate_ = std::max(0.5, replan_rate_);

        map_sub_ = nh_.subscribe(map_topic_, 1,
            &StaticPathPlanner::globalMapCallback, this);
        local_map_sub_ = nh_.subscribe(local_map_topic_, 1,
            &StaticPathPlanner::localMapCallback, this);
        local_free_space_sub_ = nh_.subscribe(local_free_space_topic_, 1,
            &StaticPathPlanner::localFreeSpaceCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20,
            &StaticPathPlanner::odomCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 5,
            &StaticPathPlanner::goalCallback, this);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            waypoint_topic_, 5, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 2, true);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>(
            local_path_topic_, 2, true);
        timer_ = nh_.createTimer(ros::Duration(1.0 / replan_rate_),
            &StaticPathPlanner::timerCallback, this);

        ROS_INFO("[StaticPathPlanner] global=%s local=%s free=%s goal=%s "
                 "waypoint=%s resolution=%.2f inflation=%.2f "
                 "replan_final=%d horizon=%.2f lookahead=%.2f",
                 map_topic_.c_str(), local_map_topic_.c_str(),
                 local_free_space_topic_.c_str(), goal_topic_.c_str(),
                 waypoint_topic_.c_str(), resolution_, inflation_radius_,
                 static_cast<int>(local_replan_to_final_goal_),
                 local_planning_horizon_, lookahead_distance_);
    }

private:
    struct Grid
    {
        double origin_x{0.0};
        double origin_y{0.0};
        double resolution{0.25};
        int width{0};
        int height{0};
        std::vector<std::uint8_t> occupied;

        bool inside(int x, int y) const
        {
            return x >= 0 && y >= 0 && x < width && y < height;
        }

        int index(int x, int y) const { return y * width + x; }

        bool isOccupied(int x, int y) const
        {
            return !inside(x, y) || occupied[index(x, y)] != 0U;
        }

        bool worldToCell(double wx, double wy, int& x, int& y) const
        {
            x = static_cast<int>(std::floor((wx - origin_x) / resolution));
            y = static_cast<int>(std::floor((wy - origin_y) / resolution));
            return inside(x, y);
        }

        Eigen::Vector2d cellCenter(int x, int y) const
        {
            return Eigen::Vector2d(
                origin_x + (static_cast<double>(x) + 0.5) * resolution,
                origin_y + (static_cast<double>(y) + 0.5) * resolution);
        }
    };

    struct OpenNode
    {
        double f{0.0};
        int index{-1};
        bool operator<(const OpenNode& rhs) const { return f > rhs.f; }
    };

    static std::string normalizeFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
        return frame;
    }

    bool validMapFrame(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       const char* label) const
    {
        if (normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
        {
            ROS_ERROR_THROTTLE(1.0,
                "[StaticPathPlanner] %s map frame '%s' != '%s'.",
                label, msg->header.frame_id.c_str(), world_frame_.c_str());
            return false;
        }
        return true;
    }

    void globalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!validMapFrame(msg, "global")) return;
        pcl::fromROSMsg(*msg, global_map_);
        have_global_map_ = true;
    }

    void localMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!validMapFrame(msg, "local")) return;
        pcl::fromROSMsg(*msg, local_map_);
        have_local_map_ = true;
    }

    void localFreeSpaceCallback(
        const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!validMapFrame(msg, "local free-space")) return;
        pcl::fromROSMsg(*msg, local_free_space_);
        have_local_free_space_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_ = *msg;
        have_odom_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!msg->header.frame_id.empty() &&
            normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
        {
            ROS_ERROR("[StaticPathPlanner] rejected goal frame '%s'; expected '%s'.",
                      msg->header.frame_id.c_str(), world_frame_.c_str());
            return;
        }
        final_goal_ = *msg;
        final_goal_.header.frame_id = world_frame_;
        if (final_goal_.pose.position.z <= 0.1 && have_odom_)
            final_goal_.pose.position.z = odom_.pose.pose.position.z;
        have_goal_ = true;
        ROS_INFO("[StaticPathPlanner] final goal=(%.2f %.2f %.2f)",
                 final_goal_.pose.position.x, final_goal_.pose.position.y,
                 final_goal_.pose.position.z);
    }

    bool buildGrid(const Eigen::Vector2d& start,
                   const Eigen::Vector2d& goal,
                   const double flight_z,
                   const CloudT& primary_map,
                   const CloudT* overlay_map,
                   const CloudT* clearing_map,
                   const double margin,
                   Grid& grid) const
    {
        const double min_x = std::min(start.x(), goal.x()) - margin;
        const double min_y = std::min(start.y(), goal.y()) - margin;
        const double max_x = std::max(start.x(), goal.x()) + margin;
        const double max_y = std::max(start.y(), goal.y()) + margin;

        grid.origin_x = min_x;
        grid.origin_y = min_y;
        grid.resolution = resolution_;
        grid.width = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
        grid.height = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;

        const std::int64_t cell_count =
            static_cast<std::int64_t>(grid.width) * grid.height;
        if (cell_count <= 0 || cell_count > max_grid_cells_)
        {
            ROS_ERROR_THROTTLE(1.0,
                "[StaticPathPlanner] grid too large: %d x %d; reduce goal range "
                "or increase resolution.", grid.width, grid.height);
            return false;
        }
        grid.occupied.assign(static_cast<std::size_t>(cell_count), 0U);

        // Rasterization order is important:
        // global occupied -> locally observed free -> current local occupied.
        // Thus a current return always wins, while a ray that sees through an
        // old person can immediately remove that stale global obstacle.
        const auto rasterize = [&](const CloudT& cloud,
                                   const double radius,
                                   const std::uint8_t value)
        {
            const int radius_cells = static_cast<int>(
                std::ceil(radius / resolution_));
            const double radius_sq = radius * radius;
            for (const auto& point : cloud.points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) continue;
                const double relative_z = point.z - flight_z;
                if (relative_z < obstacle_min_relative_z_ ||
                    relative_z > obstacle_max_relative_z_) continue;
                int x = 0;
                int y = 0;
                if (!grid.worldToCell(point.x, point.y, x, y)) continue;
                for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    {
                        if ((dx * resolution_) * (dx * resolution_) +
                            (dy * resolution_) * (dy * resolution_) > radius_sq)
                            continue;
                        if (grid.inside(x + dx, y + dy))
                            grid.occupied[grid.index(x + dx, y + dy)] = value;
                    }
                }
            }
        };
        rasterize(primary_map, inflation_radius_, 1U);
        if (clearing_map != nullptr)
        {
            // Clear both the stale obstacle cell and the inflation halo it had
            // created. Real surfaces visible now are restored by local_map.
            rasterize(*clearing_map,
                      inflation_radius_ + local_free_clear_radius_, 0U);
        }
        if (overlay_map != nullptr)
            rasterize(*overlay_map, inflation_radius_, 1U);

        /*
         * Do not clear the whole local-radius disk: unobserved/occluded space
         * remains governed by the global map. Only explicit LiDAR free rays
         * are allowed to invalidate a global obstacle.
         */

        // The current vehicle footprint naturally appears in accumulated maps.
        // Clear only a small start disk; local collision protection remains active.
        int sx = 0;
        int sy = 0;
        if (grid.worldToCell(start.x(), start.y(), sx, sy))
        {
            const int clear_cells = static_cast<int>(
                std::ceil(start_clearance_radius_ / resolution_));
            for (int dy = -clear_cells; dy <= clear_cells; ++dy)
                for (int dx = -clear_cells; dx <= clear_cells; ++dx)
                    if (grid.inside(sx + dx, sy + dy) &&
                        std::hypot(dx * resolution_, dy * resolution_) <=
                            start_clearance_radius_)
                        grid.occupied[grid.index(sx + dx, sy + dy)] = 0U;
        }
        return true;
    }

    bool nearestFreeCell(const Grid& grid, int& x, int& y) const
    {
        if (!grid.isOccupied(x, y)) return true;
        const int radius = static_cast<int>(
            std::ceil(goal_search_radius_ / grid.resolution));
        int best_x = x;
        int best_y = y;
        double best_distance = std::numeric_limits<double>::infinity();
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if (!grid.inside(x + dx, y + dy) ||
                    grid.isOccupied(x + dx, y + dy)) continue;
                const double distance = std::hypot(dx, dy);
                if (distance < best_distance)
                {
                    best_distance = distance;
                    best_x = x + dx;
                    best_y = y + dy;
                }
            }
        }
        if (!std::isfinite(best_distance)) return false;
        x = best_x;
        y = best_y;
        return true;
    }

    bool runAStar(const Grid& grid,
                  const Eigen::Vector2d& start,
                  const Eigen::Vector2d& goal,
                  std::vector<Eigen::Vector2d>& path) const
    {
        int sx = 0, sy = 0, gx = 0, gy = 0;
        if (!grid.worldToCell(start.x(), start.y(), sx, sy) ||
            !grid.worldToCell(goal.x(), goal.y(), gx, gy)) return false;
        const bool original_goal_free = !grid.isOccupied(gx, gy);
        if (!nearestFreeCell(grid, gx, gy)) return false;

        const int count = grid.width * grid.height;
        const int start_index = grid.index(sx, sy);
        const int goal_index = grid.index(gx, gy);
        std::vector<double> g_score(count,
            std::numeric_limits<double>::infinity());
        std::vector<int> parent(count, -1);
        std::vector<std::uint8_t> closed(count, 0U);
        std::priority_queue<OpenNode> open;
        g_score[start_index] = 0.0;
        open.push({std::hypot(gx - sx, gy - sy), start_index});

        const int offsets[8][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        while (!open.empty())
        {
            const int current = open.top().index;
            open.pop();
            if (closed[current] != 0U) continue;
            closed[current] = 1U;
            if (current == goal_index) break;
            const int cx = current % grid.width;
            const int cy = current / grid.width;
            for (const auto& offset : offsets)
            {
                const int nx = cx + offset[0];
                const int ny = cy + offset[1];
                if (grid.isOccupied(nx, ny)) continue;
                if (offset[0] != 0 && offset[1] != 0 &&
                    (grid.isOccupied(cx + offset[0], cy) ||
                     grid.isOccupied(cx, cy + offset[1]))) continue;
                const int next = grid.index(nx, ny);
                const double step = offset[0] != 0 && offset[1] != 0
                    ? std::sqrt(2.0) : 1.0;
                const double candidate = g_score[current] + step;
                if (candidate >= g_score[next]) continue;
                g_score[next] = candidate;
                parent[next] = current;
                open.push({candidate + std::hypot(gx - nx, gy - ny), next});
            }
        }

        if (start_index != goal_index && parent[goal_index] < 0) return false;
        std::vector<Eigen::Vector2d> reverse_path;
        for (int current = goal_index; current >= 0; current = parent[current])
        {
            reverse_path.push_back(
                grid.cellCenter(current % grid.width, current / grid.width));
            if (current == start_index) break;
        }
        if (reverse_path.empty()) return false;
        path.assign(reverse_path.rbegin(), reverse_path.rend());
        path.front() = start;
        if (original_goal_free) path.back() = goal;
        return true;
    }

    bool lineIsFree(const Grid& grid,
                    const Eigen::Vector2d& from,
                    const Eigen::Vector2d& to) const
    {
        const double distance = (to - from).norm();
        const int samples = std::max(1, static_cast<int>(
            std::ceil(distance / (0.5 * grid.resolution))));
        for (int i = 1; i <= samples; ++i)
        {
            const double t = static_cast<double>(i) / samples;
            const Eigen::Vector2d point = (1.0 - t) * from + t * to;
            int x = 0, y = 0;
            if (!grid.worldToCell(point.x(), point.y(), x, y) ||
                grid.isOccupied(x, y)) return false;
        }
        return true;
    }

    std::vector<Eigen::Vector2d> simplifyPath(
        const Grid& grid, const std::vector<Eigen::Vector2d>& path) const
    {
        if (path.size() <= 2) return path;
        std::vector<Eigen::Vector2d> simplified;
        simplified.push_back(path.front());
        std::size_t current = 0;
        while (current + 1 < path.size())
        {
            std::size_t best = current + 1;
            for (std::size_t candidate = path.size() - 1;
                 candidate > current + 1; --candidate)
            {
                if (lineIsFree(grid, path[current], path[candidate]))
                {
                    best = candidate;
                    break;
                }
            }
            simplified.push_back(path[best]);
            current = best;
        }
        return simplified;
    }

    Eigen::Vector2d pointAtDistance(
        const std::vector<Eigen::Vector2d>& path, const double distance) const
    {
        if (path.empty()) return Eigen::Vector2d::Zero();
        double remaining = std::max(0.0, distance);
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const Eigen::Vector2d segment = path[i] - path[i - 1];
            const double length = segment.norm();
            if (length >= remaining && length > 1e-6)
                return path[i - 1] + (remaining / length) * segment;
            remaining -= length;
        }
        return path.back();
    }

    void publishPath(const std::vector<Eigen::Vector2d>& path,
                     const ros::Time& stamp,
                     ros::Publisher& publisher)
    {
        nav_msgs::Path message;
        message.header.stamp = stamp;
        message.header.frame_id = world_frame_;
        for (const auto& point : path)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = message.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = final_goal_.pose.position.z;
            pose.pose.orientation.w = 1.0;
            message.poses.push_back(pose);
        }
        publisher.publish(message);
    }

    void publishWaypoint(const Eigen::Vector2d& point, const ros::Time& stamp)
    {
        geometry_msgs::PoseStamped waypoint = final_goal_;
        waypoint.header.stamp = stamp;
        waypoint.header.frame_id = world_frame_;
        waypoint.pose.position.x = point.x();
        waypoint.pose.position.y = point.y();
        waypoint_pub_.publish(waypoint);
    }

    void publishHold(const ros::Time& stamp)
    {
        if (!have_odom_) return;
        const Eigen::Vector2d current(
            odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        publishWaypoint(current, stamp);
        publishPath({current}, stamp, path_pub_);
        publishPath({current}, stamp, local_path_pub_);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!have_goal_ || !have_odom_) return;
        const ros::Time now = ros::Time::now();
        // RViz 2D Nav Goal carries z=0. If it was clicked before takeoff,
        // resolve its height lazily from the current flight altitude.
        if (final_goal_.pose.position.z <= 0.1 &&
            odom_.pose.pose.position.z > 0.1)
            final_goal_.pose.position.z = odom_.pose.pose.position.z;
        const Eigen::Vector2d start(
            odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        const Eigen::Vector2d goal(
            final_goal_.pose.position.x, final_goal_.pose.position.y);

        if ((goal - start).norm() <= goal_tolerance_)
        {
            publishWaypoint(goal, now);
            publishPath({start, goal}, now, path_pub_);
            publishPath({start, goal}, now, local_path_pub_);
            return;
        }
        if (!have_global_map_ ||
            global_map_.size() < static_cast<std::size_t>(minimum_map_points_))
        {
            ROS_WARN_THROTTLE(1.0,
                "[StaticPathPlanner] stable map not ready (%zu/%d); holding.",
                global_map_.size(), minimum_map_points_);
            publishHold(now);
            return;
        }

        // Stage 1: persistent global route to the final goal.  This route is
        // insensitive to short-lived objects because dynamic trajectories are
        // removed from the stable map by StaticMapBuilder.
        Grid global_grid;
        if (!buildGrid(start, goal, final_goal_.pose.position.z, global_map_,
                       nullptr, nullptr, planning_margin_, global_grid))
        {
            publishHold(now);
            return;
        }
        std::vector<Eigen::Vector2d> global_path;
        if (!runAStar(global_grid, start, goal, global_path))
        {
            ROS_WARN_THROTTLE(1.0,
                "[StaticPathPlanner] no global path; holding and replanning.");
            publishHold(now);
            return;
        }
        const std::vector<Eigen::Vector2d> global_simplified =
            simplifyPath(global_grid, global_path);
        publishPath(global_simplified, now, path_pub_);

        // Stage 2: by default recompute the complete route after applying local
        // occupied and free-space evidence. This can immediately shortcut a
        // stale global detour; horizon mode remains available as a parameter.
        const Eigen::Vector2d local_goal = local_replan_to_final_goal_
            ? goal
            : pointAtDistance(global_simplified, local_planning_horizon_);
        Grid local_grid;
        const CloudT* local_overlay = have_local_map_ ? &local_map_ : nullptr;
        const CloudT* local_clearing = have_local_free_space_
            ? &local_free_space_ : nullptr;
        const double local_margin = local_replan_to_final_goal_
            ? planning_margin_ : local_planning_margin_;
        if (!buildGrid(start, local_goal, final_goal_.pose.position.z,
                       global_map_, local_overlay, local_clearing, local_margin,
                       local_grid))
        {
            publishHold(now);
            return;
        }
        std::vector<Eigen::Vector2d> local_path;
        if (!runAStar(local_grid, start, local_goal, local_path))
        {
            ROS_WARN_THROTTLE(1.0,
                "[StaticPathPlanner] local route blocked; holding while the "
                "rolling map expires and replanning.");
            publishHold(now);
            return;
        }

        const std::vector<Eigen::Vector2d> local_simplified =
            simplifyPath(local_grid, local_path);
        const Eigen::Vector2d waypoint = pointAtDistance(
            local_simplified, lookahead_distance_);
        publishPath(local_simplified, now, local_path_pub_);
        publishWaypoint(waypoint, now);
        ROS_INFO_THROTTLE(1.0,
            "[StaticPathPlanner] global=%zu/%zu local=%zu/%zu "
            "waypoint=(%.2f %.2f) remaining=%.2f",
            global_path.size(), global_simplified.size(), local_path.size(),
            local_simplified.size(), waypoint.x(), waypoint.y(),
            (goal - start).norm());
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber map_sub_, local_map_sub_, local_free_space_sub_;
    ros::Subscriber odom_sub_, goal_sub_;
    ros::Publisher waypoint_pub_, path_pub_, local_path_pub_;
    ros::Timer timer_;
    std::string map_topic_, local_map_topic_, local_free_space_topic_;
    std::string odom_topic_, goal_topic_;
    std::string waypoint_topic_, path_topic_, local_path_topic_, world_frame_;
    CloudT global_map_, local_map_, local_free_space_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped final_goal_;
    double resolution_{0.25}, inflation_radius_{0.95}, planning_margin_{4.0};
    double local_planning_margin_{2.5}, local_planning_horizon_{4.0};
    double local_free_clear_radius_{0.15};
    double obstacle_min_relative_z_{-0.80}, obstacle_max_relative_z_{0.80};
    double lookahead_distance_{1.0}, goal_tolerance_{0.30};
    double start_clearance_radius_{0.35}, goal_search_radius_{1.5};
    double replan_rate_{2.0};
    int minimum_map_points_{30}, max_grid_cells_{250000};
    bool have_global_map_{false}, have_local_map_{false};
    bool have_local_free_space_{false}, local_replan_to_final_goal_{true};
    bool have_odom_{false}, have_goal_{false};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_path_planner");
    StaticPathPlanner planner;
    ros::spin();
    return 0;
}
