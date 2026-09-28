#!/usr/bin/env python3
"""One optional walking episode. Truth is used ONLY to control the actor.

Wait until UAV approaches; allow stationary person to be mapped, walk along
left aisle, then release cmd_vel for keyboard. Warehouse relay retains guards.
"""
import math
import time
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    rospy.init_node('exploration_walk_once')
    state = {}
    def receive(msg):
        state.update({name:msg.pose[i].position for i,name in enumerate(msg.name)
                      if name in ('0','iris_mid360')})
    rospy.Subscriber('/gazebo/model_states',ModelStates,receive,queue_size=1)
    pub = rospy.Publisher('/demo/pedestrian/0/cmd_vel',Twist,queue_size=1)
    status = rospy.Publisher('/demo/pedestrian/walk_status',String,queue_size=1,latch=True)
    close_since = None
    start_y = None
    status.publish('waiting_for_uav')
    while not rospy.is_shutdown():
        if all(n in state for n in ('0','iris_mid360')):
            p,u = state['0'],state['iris_mid360']
            if math.hypot(p.x-u.x,p.y-u.y) < 6. and u.z > .9:
                close_since = close_since or rospy.Time.now().to_sec()
                if rospy.Time.now().to_sec()-close_since > 8.:
                    start_y = p.y
                    break
            else:
                close_since = None
        time.sleep(.1)
    try:
        if rospy.is_shutdown() or start_y is None:
            return
        status.publish('walking')
        # Left aisle only, away from shelves; relay still checks each segment.
        for x,y in ((-8.,start_y-3.),(-8.,start_y+3.)):
            deadline = time.monotonic()+90.
            while not rospy.is_shutdown():
                p = state['0']
                dx,dy = x-p.x,y-p.y
                distance = math.hypot(dx,dy)
                if distance < .1:
                    break
                if time.monotonic()>deadline:
                    raise RuntimeError('Pedestrian movement timed out; inspect collision guard')
                msg = Twist()
                speed = min(.5,distance)
                msg.linear.x,msg.linear.y = speed*dx/distance,speed*dy/distance
                pub.publish(msg)
                time.sleep(.05)
        status.publish('finished_keyboard_ready')
        rospy.loginfo('Automatic walk done; keyboard now owns human cmd_vel.')
        # Retain latched status; never writes velocity again.
        rospy.spin()
    finally:
        pub.publish(Twist())


if __name__ == '__main__':
    main()
