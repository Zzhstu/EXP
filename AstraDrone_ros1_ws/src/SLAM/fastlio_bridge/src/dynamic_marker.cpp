#include <ros/ros.h>
#include <set>

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
            1,
            &DynamicMarker::callback,
            this
        );


    pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(
            "/uav1/dynamic_markers",
            1
        );


    ROS_INFO("[DynamicMarker] Started");
}


void callback(
const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
{

    visualization_msgs::MarkerArray array;


    std::set<int> current;


    for(auto& obj:msg->objects)
    {

        visualization_msgs::Marker marker;


        marker.header = msg->header;

        marker.ns="dynamic_object";

        marker.id=obj.id; // Use stable track ID rather than array index.
        current.insert(obj.id);
        marker.lifetime=ros::Duration(0.5); // Expire even if upstream stops.


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
        marker.color.g=obj.predicted ? 0.6 : 0.0;


        array.markers.push_back(marker);

    }


    for (int id : previous_) {
        if (current.count(id)) continue;
        visualization_msgs::Marker marker;
        marker.header=msg->header;
        marker.ns="dynamic_object";
        marker.id=id;
        marker.action=visualization_msgs::Marker::DELETE;
        array.markers.push_back(marker);
    }
    previous_=current;
    pub_.publish(array);

}



private:
std::set<int> previous_;

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
