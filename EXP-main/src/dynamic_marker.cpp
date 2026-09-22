#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "fastlio_bridge/DynamicObjectArray.h"


class DynamicMarker
{

public:

DynamicMarker()
{
    ros::NodeHandle nh;

    sub_ =
        nh.subscribe(
            "/uav1/dynamic_objects",
            10,
            &DynamicMarker::callback,
            this
        );


    pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(
            "/uav1/dynamic_markers",
            10
        );


    ROS_INFO("[DynamicMarker] Started");
}


void callback(
const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
{

    visualization_msgs::MarkerArray array;


    int id=0;


    for(auto& obj:msg->objects)
    {

        visualization_msgs::Marker marker;


        marker.header = msg->header;

        marker.ns="dynamic_object";

        marker.id=id++;


        marker.type =
            visualization_msgs::Marker::SPHERE;


        marker.action =
            visualization_msgs::Marker::ADD;



        marker.pose =
            obj.pose;



        marker.scale.x=0.3;
        marker.scale.y=0.3;
        marker.scale.z=0.3;


        marker.color.a=1.0;
        marker.color.r=1.0;


        array.markers.push_back(marker);

    }


    pub_.publish(array);

}



private:

ros::Subscriber sub_;

ros::Publisher pub_;

};


int main(int argc,char** argv)
{

ros::init(
argc,
argv,
"dynamic_marker"
);


DynamicMarker node;


ros::spin();

return 0;

}
