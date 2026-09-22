#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>

#include <vector>
#include <cmath>
#include <algorithm>


class DynamicTracker
{

public:

    DynamicTracker()
    {
        ros::NodeHandle nh;


        sub_ =
            nh.subscribe(
                "/uav1/dynamic_objects",
                10,
                &DynamicTracker::callback,
                this
            );


        pub_ =
            nh.advertise
            <fastlio_bridge::DynamicObjectArray>
            (
                "/uav1/tracked_objects",
                10
            );


        ROS_INFO("[DynamicTracker] Started.");
    }



private:


    struct Track
    {
        int id;


        geometry_msgs::Point position;


        geometry_msgs::Vector3 velocity;


        ros::Time stamp;


        ros::Time last_seen;
    };



    double distance(
        const geometry_msgs::Point& a,
        const geometry_msgs::Point& b
    )
    {

        return sqrt(
            pow(a.x-b.x,2)
            +
            pow(a.y-b.y,2)
            +
            pow(a.z-b.z,2)
        );

    }




    void callback(
        const fastlio_bridge::DynamicObjectArray::ConstPtr& msg
    )
    {


        fastlio_bridge::DynamicObjectArray output;


        output.header =
            msg->header;



        ros::Time now =
            msg->header.stamp;



        std::vector<bool> matched(
            tracks_.size(),
            false
        );



        for(
            const auto& obj:
            msg->objects
        )
        {


            double min_dist =
                999.0;


            int best =
                -1;



            // ==========================
            // 最近邻匹配
            // ==========================

            for(
                size_t i=0;
                i<tracks_.size();
                i++
            )
            {

                if(matched[i])
                    continue;


                double d =
                    distance(
                        obj.pose.position,
                        tracks_[i].position
                    );


                if(d < min_dist)
                {
                    min_dist = d;
                    best = i;
                }

            }




            fastlio_bridge::DynamicObject tracked;



            // ==========================
            // 匹配成功
            // ==========================

            if(
                best >=0
                &&
                min_dist < 1.0
            )
            {


                Track& t =
                    tracks_[best];



                double dt =
                    (now-t.stamp)
                    .toSec();



                tracked.id =
                    t.id;



                tracked.pose =
                    obj.pose;




                if(dt > 0.001)
                {


                    double vx =
                    (
                        obj.pose.position.x
                        -
                        t.position.x
                    )
                    /
                    dt;



                    double vy =
                    (
                        obj.pose.position.y
                        -
                        t.position.y
                    )
                    /
                    dt;



                    double vz =
                    (
                        obj.pose.position.z
                        -
                        t.position.z
                    )
                    /
                    dt;



                    // ====================
                    // 速度限制
                    // ====================

                    double max_v = 5.0;


                    double norm =
                        sqrt(
                            vx*vx+
                            vy*vy+
                            vz*vz
                        );



                    if(norm > max_v)
                    {

                        vx =
                        vx/norm*max_v;


                        vy =
                        vy/norm*max_v;


                        vz =
                        vz/norm*max_v;

                    }




                    // ====================
                    // 低通滤波
                    // ====================

                    double alpha = 0.2;



                    t.velocity.x =
                        alpha*vx
                        +
                        (1-alpha)
                        *
                        t.velocity.x;



                    t.velocity.y =
                        alpha*vy
                        +
                        (1-alpha)
                        *
                        t.velocity.y;



                    t.velocity.z =
                        alpha*vz
                        +
                        (1-alpha)
                        *
                        t.velocity.z;



                }



                tracked.twist.linear =
                    t.velocity;



                t.position =
                    obj.pose.position;



                t.stamp =
                    now;



                t.last_seen =
                    now;



                matched[best]=true;


            }


            // ==========================
            // 新目标
            // ==========================

            else
            {


                Track t;


                t.id =
                    next_id_++;



                t.position =
                    obj.pose.position;



                t.velocity.x = 0.0;
                t.velocity.y = 0.0;
                t.velocity.z = 0.0;



                t.stamp =
                    now;



                t.last_seen =
                    now;



                tracks_.push_back(t);



                tracked.id =
                    t.id;


                tracked.pose =
                    obj.pose;



                tracked.twist.linear.x = 0.0;
                tracked.twist.linear.y = 0.0;
                tracked.twist.linear.z = 0.0;


            }



            tracked.pose.orientation.x = 0.0;
            tracked.pose.orientation.y = 0.0;
            tracked.pose.orientation.z = 0.0;
            tracked.pose.orientation.w = 1.0;



            output.objects.push_back(
                tracked
            );


        }





        // ==========================
        // 删除消失目标
        // ==========================

        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),

                [&](const Track& t)
                {

                    return
                    (now-t.last_seen)
                    .toSec()
                    >
                    1.0;

                }),

            tracks_.end()
        );




        pub_.publish(output);



        ROS_INFO_THROTTLE(
            1.0,
            "[DynamicTracker] tracks=%zu",
            tracks_.size()
        );

    }



private:


    ros::Subscriber sub_;

    ros::Publisher pub_;


    std::vector<Track> tracks_;


    int next_id_{0};

};



int main(
    int argc,
    char** argv
)
{

    ros::init(
        argc,
        argv,
        "dynamic_tracker"
    );


    DynamicTracker node;


    ros::spin();


    return 0;
}
