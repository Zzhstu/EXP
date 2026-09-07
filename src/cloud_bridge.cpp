#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>


class FastLIOCloudBridge
{
public:
    //hello:test for function1
    using Cloud = sensor_msgs::PointCloud2;
    using LioOdom = nav_msgs::Odometry;
    using Px4Pose = geometry_msgs::PoseStamped;

    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            LioOdom,
            Px4Pose>;


    FastLIOCloudBridge()
        : nh_(),
          pnh_("~"),
          lio_sub_(
              nh_,
              "/Odometry",
              10),
          px4_sub_(
              nh_,
              "/uav1/mavros/local_position/pose",
              10),
          sync_(
              SyncPolicy(20),
              lio_sub_,
              px4_sub_)
    {
        pnh_.param(
            "use_px4_initial_alignment",
            use_px4_initial_alignment_,
            false);

        // =====================================================
        // 订阅已经完成运动补偿的 Fast-LIO 点云
        // =====================================================

        cloud_sub_ =
            nh_.subscribe(
                "/cloud_registered",
                10,
                &FastLIOCloudBridge::cloudCallback,
                this);


        // =====================================================
        // 发布转换到 Prometheus map 的点云
        // =====================================================

        cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "/uav1/fastlio/cloud_map",
                2);


        // =====================================================
        // 用 Fast-LIO + PX4 只计算一次固定坐标变换
        // =====================================================

        if (use_px4_initial_alignment_)
        {
            sync_.registerCallback(
                boost::bind(
                    &FastLIOCloudBridge::alignmentCallback,
                    this,
                    _1,
                    _2));
        }
        else
        {
            // 必须与fastlio_bridge保持一致：GNSS拒止模式下直接定义
            // camera_init为map，不再等待不存在的PX4本地位姿。
            T_map_camera_init_ = Eigen::Isometry3d::Identity();
            initialized_ = true;
        }


        ROS_INFO(
            "[FastLIOCloudBridge] Started.");

        if (use_px4_initial_alignment_)
        {
            ROS_INFO(
                "[FastLIOCloudBridge] Waiting for initial PX4 alignment...");
        }
        else
        {
            ROS_INFO(
                "[FastLIOCloudBridge] GNSS-denied mode: "
                "map == camera_init; PX4 local pose is not required.");
        }
    }


private:

    // =========================================================
    // nav_msgs/Odometry → Eigen Isometry
    // =========================================================

    Eigen::Isometry3d odomToTransform(
        const nav_msgs::Odometry& msg)
    {
        Eigen::Isometry3d T =
            Eigen::Isometry3d::Identity();

        Eigen::Quaterniond q(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z);

        q.normalize();

        T.linear() =
            q.toRotationMatrix();

        T.translation() =
            Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z);

        return T;
    }


    // =========================================================
    // PoseStamped → Eigen Isometry
    // =========================================================

    Eigen::Isometry3d poseToTransform(
        const geometry_msgs::PoseStamped& msg)
    {
        Eigen::Isometry3d T =
            Eigen::Isometry3d::Identity();

        Eigen::Quaterniond q(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z);

        q.normalize();

        T.linear() =
            q.toRotationMatrix();

        T.translation() =
            Eigen::Vector3d(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z);

        return T;
    }


    // =========================================================
    // 第一次对齐
    //
    // T_map_camera_init =
    //      T_map_body(PX4)
    //      *
    //      inverse(T_camera_init_body(Fast-LIO))
    // =========================================================

    void alignmentCallback(
        const LioOdom::ConstPtr& lio_msg,
        const Px4Pose::ConstPtr& px4_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 已经初始化，不再重复计算
        if (initialized_)
            return;


        Eigen::Isometry3d
            T_camera_init_body =
            odomToTransform(*lio_msg);


        Eigen::Isometry3d
            T_map_body_px4 =
            poseToTransform(*px4_msg);


        T_map_camera_init_ =
            T_map_body_px4 *
            T_camera_init_body.inverse();


        initialized_ = true;


        Eigen::Vector3d t =
            T_map_camera_init_.translation();


        Eigen::Quaterniond q(
            T_map_camera_init_.rotation());

        q.normalize();


        ROS_INFO(
            "[FastLIOCloudBridge] "
            "Initial alignment completed.");


        ROS_INFO(
            "[FastLIOCloudBridge] "
            "T_map_camera_init translation: "
            "(%.4f, %.4f, %.4f)",
            t.x(),
            t.y(),
            t.z());


        ROS_INFO(
            "[FastLIOCloudBridge] "
            "T_map_camera_init quaternion: "
            "(%.4f, %.4f, %.4f, %.4f)",
            q.x(),
            q.y(),
            q.z(),
            q.w());
    }


    // =========================================================
    // 点云回调
    //
    // /cloud_registered 已经经过 Fast-LIO 的运动补偿，
    // 位于 camera_init 全局坐标系。
    //
    // 所以这里只做：
    //
    // p_map =
    //      T_map_camera_init * p_camera_init
    // =========================================================

    void cloudCallback(
        const Cloud::ConstPtr& cloud_msg)
    {
        Eigen::Isometry3d T_map_camera_init;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!initialized_)
            {
                ROS_WARN_THROTTLE(
                    2.0,
                    "[FastLIOCloudBridge] "
                    "Waiting for initial alignment...");
                return;
            }

            T_map_camera_init =
                T_map_camera_init_;
        }


        // =====================================================
        // ROS PointCloud2 → PCL
        // =====================================================

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(
            new pcl::PointCloud<pcl::PointXYZI>);


        pcl::fromROSMsg(
            *cloud_msg,
            *cloud_in);


        if (cloud_in->empty())
            return;


        // =====================================================
        // 固定 camera_init → map
        // =====================================================

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
            new pcl::PointCloud<pcl::PointXYZI>);


        Eigen::Matrix4f transform =
            T_map_camera_init.matrix()
                .cast<float>();


        pcl::transformPointCloud(
            *cloud_in,
            *cloud_out,
            transform);


        // =====================================================
        // PCL → ROS
        // =====================================================

        sensor_msgs::PointCloud2 output;


        pcl::toROSMsg(
            *cloud_out,
            output);


        output.header.stamp =
            cloud_msg->header.stamp;


        output.header.frame_id =
            "map";


        cloud_pub_.publish(
            output);
    }


private:

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;


    // =========================================================
    // Fast-LIO / PX4
    // =========================================================

    message_filters::Subscriber<
        nav_msgs::Odometry>
        lio_sub_;


    message_filters::Subscriber<
        geometry_msgs::PoseStamped>
        px4_sub_;


    message_filters::Synchronizer<
        SyncPolicy>
        sync_;


    // =========================================================
    // Fast-LIO 点云
    // =========================================================

    ros::Subscriber cloud_sub_;


    ros::Publisher cloud_pub_;


    // =========================================================
    // 固定 camera_init → map
    // =========================================================

    Eigen::Isometry3d
        T_map_camera_init_ =
        Eigen::Isometry3d::Identity();


    bool initialized_{false};

    bool use_px4_initial_alignment_{false};


    std::mutex mutex_;
};


int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "fastlio_cloud_bridge");


    FastLIOCloudBridge bridge;


    ros::spin();


    return 0;
}
