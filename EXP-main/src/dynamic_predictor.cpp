#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <Eigen/Dense>

#include <algorithm>
#include <string>


class DynamicPredictor
{
public:

    DynamicPredictor()
        : pnh_("~")
    {
        ros::NodeHandle nh;


        // =====================================================
        // Subscriber
        // =====================================================

        pnh_.param<std::string>("input_topic", input_topic_,
                               "/uav1/dynamic_objects");
        pnh_.param<std::string>("output_topic", output_topic_,
                               "/uav1/predicted_objects");
        pnh_.param("prediction_time", prediction_time_, 0.5);
        prediction_time_ = std::max(0.0, std::min(3.0, prediction_time_));

        sub_ = nh.subscribe(
            input_topic_,
            10,
            &DynamicPredictor::callback,
            this
        );


        // =====================================================
        // Publisher
        // =====================================================

        pub_ = nh.advertise<
            fastlio_bridge::DynamicObjectArray
        >(
            output_topic_,
            10
        );


        // =====================================================
        // Prediction horizon
        //
        // 当前先预测0.5秒后
        // =====================================================

        ROS_INFO(
            "[DynamicPredictor] Started."
        );


        ROS_INFO(
            "[DynamicPredictor] "
            "prediction_time=%.2f s",
            prediction_time_
        );
    }


private:

    // =========================================================
    // Callback
    // =========================================================

    void callback(
        const fastlio_bridge::DynamicObjectArray::ConstPtr& msg
    )
    {
        fastlio_bridge::DynamicObjectArray output;


        // -----------------------------------------------------
        // Header保持map坐标系
        // -----------------------------------------------------

        output.header =
            msg->header;


        output.header.frame_id =
            "map";


        // -----------------------------------------------------
        // 遍历所有动态目标
        // -----------------------------------------------------

        for (
            const auto& object :
            msg->objects
        )
        {
            // =================================================
            // 当前状态
            // =================================================

            Eigen::Vector3d position(
                object.pose.position.x,
                object.pose.position.y,
                object.pose.position.z
            );


            Eigen::Vector3d velocity(
                object.twist.linear.x,
                object.twist.linear.y,
                object.twist.linear.z
            );


            // =================================================
            // 匀速模型
            //
            // p_future = p + v * dt
            // =================================================

            Eigen::Vector3d predicted_position =
                position +
                velocity * prediction_time_;


            // =================================================
            // 构造预测目标
            // =================================================

            fastlio_bridge::DynamicObject predicted;


            predicted.id =
                object.id;


            predicted.pose.position.x =
                predicted_position.x();


            predicted.pose.position.y =
                predicted_position.y();


            predicted.pose.position.z =
                predicted_position.z();


            // 保持原来的姿态
            predicted.pose.orientation =
                object.pose.orientation;


            // 预测目标的速度暂时保持当前速度
            predicted.twist =
                object.twist;

            predicted.size = object.size;
            predicted.confidence = object.confidence;
            predicted.semantic_class = object.semantic_class;
            predicted.predicted = true;


            output.objects.push_back(
                predicted
            );


            // =================================================
            // Debug
            // =================================================

            ROS_INFO_THROTTLE(
                0.5,
                "[DynamicPredictor] "
                "id=%d "
                "current=(%.2f %.2f %.2f) "
                "velocity=(%.2f %.2f %.2f) "
                "predicted=(%.2f %.2f %.2f)",
                object.id,

                position.x(),
                position.y(),
                position.z(),

                velocity.x(),
                velocity.y(),
                velocity.z(),

                predicted_position.x(),
                predicted_position.y(),
                predicted_position.z()
            );
        }


        // =====================================================
        // 发布
        // =====================================================

        pub_.publish(
            output
        );
    }


private:

    ros::Subscriber sub_;

    ros::Publisher pub_;

    ros::NodeHandle pnh_;
    std::string input_topic_;
    std::string output_topic_;


    // 预测时间
    double prediction_time_;
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
        "dynamic_predictor"
    );


    DynamicPredictor node;


    ros::spin();


    return 0;
}
