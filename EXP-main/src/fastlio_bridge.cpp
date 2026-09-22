#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


class FastLIOBridge
{
public:

    using LioOdom = nav_msgs::Odometry;
    using Px4Pose = geometry_msgs::PoseStamped;

    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            LioOdom,
            Px4Pose>;

    FastLIOBridge()
        : nh_(),
          pnh_("~"),
          lio_sub_(nh_, "/Odometry", 10),
          px4_sub_(nh_, "/uav1/mavros/local_position/pose", 10),
          sync_(SyncPolicy(20), lio_sub_, px4_sub_)
    {
        pnh_.param(
            "use_px4_initial_alignment",
            use_px4_initial_alignment_,
            false);

        pnh_.param(
            "publish_mavros_vision_pose",
            publish_mavros_vision_pose_,
            false);

        if (publish_mavros_vision_pose_ &&
            use_px4_initial_alignment_)
        {
            ROS_WARN(
                "[FastLIOBridge] publish_mavros_vision_pose requires "
                "GNSS-denied alignment; disabling PX4 initial alignment.");
            use_px4_initial_alignment_ = false;
        }

        odom_pub_ =
            nh_.advertise<nav_msgs::Odometry>(
                "/uav1/fastlio/odom",
                10);

        if (publish_mavros_vision_pose_)
        {
            vision_pose_pub_ =
                nh_.advertise<geometry_msgs::PoseStamped>(
                    "/uav1/mavros/vision_pose/pose",
                    20);
        }

        // 无论是否使用PX4初始对齐，完成初始化后都由每一帧Fast-LIO
        // 里程计独立驱动输出，不能继续依赖两个话题逐帧同步。
        direct_lio_sub_ = nh_.subscribe(
            "/Odometry",
            20,
            &FastLIOBridge::lioCallback,
            this);

        if (use_px4_initial_alignment_)
        {
            sync_.registerCallback(
                boost::bind(
                    &FastLIOBridge::syncCallback,
                    this,
                    _1,
                    _2));
        }
        else
        {
            // GNSS拒止模式：直接把Fast-LIO的camera_init定义为map。
            T_map_camera_init_ = Eigen::Isometry3d::Identity();
            initialized_ = true;
        }

        ROS_INFO(
            "[FastLIOBridge] Started.");

        if (use_px4_initial_alignment_)
        {
            ROS_INFO(
                "[FastLIOBridge] Waiting for initial synchronized "
                "Fast-LIO2 + PX4 pose...");
        }
        else
        {
            ROS_INFO(
                "[FastLIOBridge] GNSS-denied mode: map == camera_init; "
                "PX4 local pose is not required.");
        }

        ROS_INFO(
            "[FastLIOBridge] MAVROS vision pose output: %s",
            publish_mavros_vision_pose_ ? "enabled" : "disabled");
    }

private:

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

        T.linear() = q.toRotationMatrix();

        T.translation() =
            Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z);

        return T;
    }


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

        T.linear() = q.toRotationMatrix();

        T.translation() =
            Eigen::Vector3d(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z);

        return T;
    }


    geometry_msgs::Quaternion
    rotationToMsg(
        const Eigen::Matrix3d& R)
    {
        Eigen::Quaterniond q(R);
        q.normalize();

        geometry_msgs::Quaternion msg;

        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();

        return msg;
    }


    void syncCallback(
        const LioOdom::ConstPtr& lio_msg,
        const Px4Pose::ConstPtr& px4_msg)
    {
        if (initialized_)
        {
            return;
        }

        Eigen::Isometry3d
            T_camera_init_body =
                odomToTransform(*lio_msg);

        Eigen::Isometry3d
            T_map_body_px4 =
                poseToTransform(*px4_msg);


        // =====================================================
        // 第一次同步：
        // 建立固定的 camera_init -> map 变换
        //
        // T_map_camera_init
        //     =
        // T_map_body_px4
        //     *
        // inverse(T_camera_init_body)
        // =====================================================

        T_map_camera_init_ =
            T_map_body_px4 *
            T_camera_init_body.inverse();

        initialized_ = true;

        Eigen::Vector3d t =
            T_map_camera_init_.translation();

        Eigen::Matrix3d R =
            T_map_camera_init_.rotation();

        Eigen::Quaterniond q(R);

        ROS_INFO(
            "[FastLIOBridge] Initial alignment completed.");

        ROS_INFO(
            "[FastLIOBridge] T_map_camera_init translation: "
            "(%.4f, %.4f, %.4f)",
            t.x(),
            t.y(),
            t.z());

        ROS_INFO(
            "[FastLIOBridge] T_map_camera_init quaternion: "
            "(%.4f, %.4f, %.4f, %.4f)",
            q.x(),
            q.y(),
            q.z(),
            q.w());
    }


    void lioCallback(
        const LioOdom::ConstPtr& lio_msg)
    {
        if (!initialized_)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[FastLIOBridge] Waiting for initial PX4 alignment; "
                "no odometry is published.");
            return;
        }

        const Eigen::Isometry3d T_camera_init_body =
            odomToTransform(*lio_msg);


        // =====================================================
        // 当前 Fast-LIO 位姿转换到 map
        //
        // T_map_body =
        // T_map_camera_init *
        // T_camera_init_body
        // =====================================================

        Eigen::Isometry3d
            T_map_body =
            T_map_camera_init_ *
            T_camera_init_body;


        nav_msgs::Odometry out = *lio_msg;

        out.header.frame_id = "map";
        out.child_frame_id = "uav1/base_link";

        out.pose.pose.position.x =
            T_map_body.translation().x();

        out.pose.pose.position.y =
            T_map_body.translation().y();

        out.pose.pose.position.z =
            T_map_body.translation().z();

        out.pose.pose.orientation =
            rotationToMsg(
                T_map_body.rotation());

        odom_pub_.publish(out);

        if (publish_mavros_vision_pose_)
        {
            geometry_msgs::PoseStamped vision_pose;
            vision_pose.header = out.header;
            vision_pose.pose = out.pose.pose;
            vision_pose_pub_.publish(vision_pose);
        }
    }


private:

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Subscriber direct_lio_sub_;

    message_filters::Subscriber<
        nav_msgs::Odometry>
        lio_sub_;

    message_filters::Subscriber<
        geometry_msgs::PoseStamped>
        px4_sub_;

    message_filters::Synchronizer<
        SyncPolicy>
        sync_;

    ros::Publisher odom_pub_;

    ros::Publisher vision_pose_pub_;

    bool initialized_{false};

    bool use_px4_initial_alignment_{false};

    bool publish_mavros_vision_pose_{false};

    Eigen::Isometry3d
        T_map_camera_init_ =
        Eigen::Isometry3d::Identity();
};


int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "fastlio_bridge");

    FastLIOBridge bridge;

    ros::spin();

    return 0;
}
