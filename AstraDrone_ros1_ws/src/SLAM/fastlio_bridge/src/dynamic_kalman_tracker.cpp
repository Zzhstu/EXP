#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <Eigen/Dense>

#include <vector>
#include <cmath>


class DynamicKalmanTracker
{

public:


DynamicKalmanTracker()
{

    ros::NodeHandle nh;


    sub_ =
    nh.subscribe(
        "/uav1/dynamic_objects",
        10,
        &DynamicKalmanTracker::callback,
        this
    );


    pub_ =
    nh.advertise
    <fastlio_bridge::DynamicObjectArray>
    (
        "/uav1/tracked_objects",
        10
    );


    ROS_INFO("[KalmanTracker] Started.");

}




private:



struct Track
{

    int id;


    // x y z vx vy vz

    Eigen::VectorXd state;


    Eigen::MatrixXd P;


    ros::Time stamp;


    int lost_count;


    int hit_count;



    Track()
    {

        state =
        Eigen::VectorXd::Zero(6);


        P =
        Eigen::MatrixXd::Identity(6,6)
        *0.1;


        lost_count=0;

        hit_count=0;

    }

};





//======================
// Kalman预测
//======================


void predict(
Track& t,
double dt
)
{


Eigen::MatrixXd F =
Eigen::MatrixXd::Identity(6,6);



F(0,3)=dt;

F(1,4)=dt;

F(2,5)=dt;



t.state =
F*t.state;



Eigen::MatrixXd Q =
Eigen::MatrixXd::Identity(6,6)
*0.005;



t.P =
F*t.P*F.transpose()
+
Q;



//速度限制

Eigen::Vector3d v;


v<<
t.state(3),
t.state(4),
t.state(5);



double speed =
v.norm();



double max_speed=3.0;



if(speed>max_speed)
{

    v =
    v/speed*max_speed;


    t.state(3)=v.x();

    t.state(4)=v.y();

    t.state(5)=v.z();

}


}





//======================
// Kalman更新
//======================


void update(
Track& t,
const Eigen::Vector3d& z
)
{


Eigen::MatrixXd H =
Eigen::MatrixXd::Zero(3,6);



H(0,0)=1;

H(1,1)=1;

H(2,2)=1;



Eigen::MatrixXd R =
Eigen::MatrixXd::Identity(3,3)
*0.05;



Eigen::Vector3d y;


y =
z-H*t.state;



Eigen::MatrixXd S;


S =
H*t.P*H.transpose()
+
R;



Eigen::MatrixXd K;


K =
t.P*
H.transpose()
*
S.inverse();



t.state += K*y;



Eigen::MatrixXd I =
Eigen::MatrixXd::Identity(6,6);



t.P =
(I-K*H)*t.P;


}





double distance(
Eigen::Vector3d a,
Eigen::Vector3d b
)
{

return (a-b).norm();

}






void callback(
const fastlio_bridge::DynamicObjectArray::ConstPtr& msg
)
{


ros::Time now =
msg->header.stamp;



fastlio_bridge::DynamicObjectArray output;


output.header =
msg->header;





//======================
// prediction
//======================


for(auto& t:tracks_)
{

    double dt =
    (now-t.stamp).toSec();



    if(dt>0)
        predict(t,dt);



    t.lost_count++;

}






std::vector<bool> matched(
tracks_.size(),
false
);






//======================
// data association
//======================


for(auto& obj:msg->objects)
{


Eigen::Vector3d measurement;


measurement
<<
obj.pose.position.x,

obj.pose.position.y,

obj.pose.position.z;




int best=-1;


double min_dist=999;





for(size_t i=0;i<tracks_.size();i++)
{


    if(matched[i])
        continue;



    Eigen::Vector3d pred;


    pred
    <<
    tracks_[i].state(0),

    tracks_[i].state(1),

    tracks_[i].state(2);



    double d =
    distance(
        measurement,
        pred
    );



    if(d<min_dist)
    {

        min_dist=d;

        best=i;

    }


}






Track* current=nullptr;





//======================
// update existing
//======================


if(best>=0 && min_dist<2.0)
{


Track& t =
tracks_[best];


update(
t,
measurement
);



t.stamp =
now;


t.lost_count=0;


t.hit_count++;


matched[best]=true;


current=&t;


}






//======================
// create new
//======================


else
{


Track t;



t.id =
next_id_++;



t.state(0)=measurement.x();

t.state(1)=measurement.y();

t.state(2)=measurement.z();



t.state(3)=0;

t.state(4)=0;

t.state(5)=0;



t.stamp=now;


t.hit_count=1;



tracks_.push_back(t);



current=&tracks_.back();


}






//======================
// publish
//======================


if(current->hit_count>=3)
{


fastlio_bridge::DynamicObject tracked;



tracked.id =
current->id;



tracked.pose.position.x =
current->state(0);


tracked.pose.position.y =
current->state(1);


tracked.pose.position.z =
current->state(2);



tracked.pose.orientation.w=1.0;



tracked.twist.linear.x =
current->state(3);


tracked.twist.linear.y =
current->state(4);


tracked.twist.linear.z =
current->state(5);



output.objects.push_back(
tracked
);



}



}






//======================
// remove lost tracks
//======================


for(auto it=tracks_.begin();
it!=tracks_.end();)
{


if(it->lost_count>10)
{

    it =
    tracks_.erase(it);

}
else
{

    ++it;

}


}





pub_.publish(output);



ROS_INFO_THROTTLE(
1.0,
"[KalmanTracker] tracks=%zu",
tracks_.size()
);



}






private:


ros::Subscriber sub_;


ros::Publisher pub_;


std::vector<Track> tracks_;


int next_id_=0;


};






int main(
int argc,
char** argv
)
{


ros::init(
argc,
argv,
"dynamic_kalman_tracker"
);



DynamicKalmanTracker node;



ros::spin();



return 0;

}
