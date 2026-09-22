#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>
#include <fastlio_bridge/SemanticDetection2DArray.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <string>

class SemanticFusion
{
public:
    SemanticFusion()
        : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
    {
        pnh_.param<std::string>("objects_topic", objects_topic_, "/uav1/dynamic_objects");
        pnh_.param<std::string>("detections_topic", detections_topic_, "/uav1/semantic/detections");
        pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
        pnh_.param<std::string>("output_topic", output_topic_, "/uav1/semantic_dynamic_objects");
        pnh_.param("enabled", enabled_, false);
        pnh_.param("detection_timeout", detection_timeout_, 0.30);
        pnh_.param("minimum_score", minimum_score_, 0.35);
        pnh_.param("box_margin_px", box_margin_px_, 12.0);

        objects_sub_ = nh_.subscribe(objects_topic_, 10, &SemanticFusion::objectsCallback, this);
        detections_sub_ = nh_.subscribe(detections_topic_, 5, &SemanticFusion::detectionsCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 2, &SemanticFusion::cameraInfoCallback, this);
        pub_ = nh_.advertise<fastlio_bridge::DynamicObjectArray>(output_topic_, 10);
    }

private:
    void detectionsCallback(const fastlio_bridge::SemanticDetection2DArray::ConstPtr& msg)
    {
        detections_ = *msg;
        detection_receive_time_ = ros::Time::now();
        have_detections_ = true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        camera_info_ = *msg;
        have_camera_info_ = true;
    }

    void objectsCallback(const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        fastlio_bridge::DynamicObjectArray output = *msg;
        const ros::Time now = ros::Time::now();
        const bool semantic_fresh = enabled_ && have_detections_ && have_camera_info_ &&
            (now - detection_receive_time_).toSec() <= detection_timeout_;
        if (!semantic_fresh)
        {
            pub_.publish(output);
            return;
        }

        for (auto& object : output.objects)
        {
            geometry_msgs::PointStamped world_point, camera_point;
            world_point.header = msg->header;
            world_point.point = object.pose.position;
            try
            {
                camera_point = tf_buffer_.transform(
                    world_point, camera_info_.header.frame_id, ros::Duration(0.02));
            }
            catch (const tf2::TransformException& error)
            {
                ROS_WARN_THROTTLE(1.0, "[SemanticFusion] TF unavailable: %s", error.what());
                break;
            }
            if (camera_point.point.z <= 0.05) continue;
            const double u = camera_info_.K[0] * camera_point.point.x /
                             camera_point.point.z + camera_info_.K[2];
            const double v = camera_info_.K[4] * camera_point.point.y /
                             camera_point.point.z + camera_info_.K[5];

            double best_score = minimum_score_;
            std::uint16_t best_class = 0;
            for (const auto& detection : detections_.detections)
            {
                const double half_x = 0.5 * detection.size_x + box_margin_px_;
                const double half_y = 0.5 * detection.size_y + box_margin_px_;
                if (std::abs(u - detection.center_x) > half_x ||
                    std::abs(v - detection.center_y) > half_y)
                    continue;
                if (detection.score > best_score)
                {
                    best_score = detection.score;
                    best_class = detection.class_id;
                }
            }
            object.semantic_class = best_class;
            object.confidence = static_cast<float>(best_score);
        }
        pub_.publish(output);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber objects_sub_, detections_sub_, camera_info_sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string objects_topic_, detections_topic_, camera_info_topic_, output_topic_;
    fastlio_bridge::SemanticDetection2DArray detections_;
    sensor_msgs::CameraInfo camera_info_;
    ros::Time detection_receive_time_;
    double detection_timeout_{0.30}, minimum_score_{0.35}, box_margin_px_{12.0};
    bool have_detections_{false}, have_camera_info_{false};
    bool enabled_{false};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_fusion");
    SemanticFusion node;
    ros::spin();
    return 0;
}
