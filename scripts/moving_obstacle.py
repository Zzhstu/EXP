#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def main():
    rospy.init_node("moving_obstacle")

    rospy.wait_for_service("/gazebo/set_model_state")

    set_model_state = rospy.ServiceProxy(
        "/gazebo/set_model_state",
        SetModelState
    )

    rate = rospy.Rate(100.0)

    model_name = "cylinder3_clone"

    x_min = -5
    x_max = -3

    y = 0.0
    z = 0.1

    velocity = 0.5
    direction = 1.0

    x = x_min

    while not rospy.is_shutdown():

        dt = 1.0 / 100.0

        x += direction * velocity * dt

        if x >= x_max:
            x = x_max
            direction = -1.0

        elif x <= x_min:
            x = x_min
            direction = 1.0

        state = ModelState()

        state.model_name = model_name
        state.reference_frame = "world"

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0

        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        try:
            result = set_model_state(state)

            if not result.success:
                rospy.logwarn(
                    "Gazebo set_model_state failed: %s",
                    result.status_message
                )

        except rospy.ServiceException as e:
            rospy.logerr(
                "Service call failed: %s",
                e
            )

        rate.sleep()


if __name__ == "__main__":
    main()
