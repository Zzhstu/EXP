#!/usr/bin/env python3
"""Keyboard-driven ONE Gazebo pedestrian, with deadman and shelf/wall guards.

No PedSim simulator or ActorPosesPlugin runs in cangku: this node is the only
pose writer. Movement translates/turns the existing standing mesh, not skeletal
walking animation. Never feeds ground-truth geometry to the UAV algorithm.
"""
import json
import math
import threading
import time
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from warehouse_scene import blocked_segment


class WarehousePedestrian:
    def __init__(self):
        self.lock = threading.RLock()
        self.command = (0., 0., 0.)
        with open(rospy.get_param('~scene_file'), encoding='utf8') as stream:
            self.boxes = json.load(stream)['boxes']
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('0', 'world')
        if not state.success:
            raise RuntimeError('The generated cangku world must contain model 0')
        self.model = ModelState(model_name='0', reference_frame='world', pose=state.pose)
        self.model.pose.orientation.x = self.model.pose.orientation.y = 0
        self.model.pose.orientation.z, self.model.pose.orientation.w = 0, 1
        self.model.pose.position.z = 0
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, persistent=True)
        self.last_time = rospy.Time.now().to_sec()
        self.ready = rospy.Publisher('/demo/pedestrian/ready', Bool, queue_size=1, latch=True)
        self.blocked_pub = rospy.Publisher('/demo/pedestrian/blocked', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/demo/pedestrian/0/cmd_vel', Twist, self.receive, queue_size=1)
        self.ready.publish(True)
        self.blocked_pub.publish(False)
        self.timer = rospy.Timer(rospy.Duration(.04), self.tick)
        rospy.loginfo('Warehouse actor 0 ready. WASD in Gazebo world XY; .4s deadman; shelf/wall guard active.')

    def receive(self, msg):
        vx, vy = msg.linear.x, msg.linear.y
        if not math.isfinite(vx) or not math.isfinite(vy):
            return
        scale = min(1., 1./max(math.hypot(vx, vy), 1e-9))
        with self.lock:
            self.command = (vx*scale, vy*scale, time.monotonic())

    def tick(self, event):
        now = rospy.Time.now().to_sec()
        dt = max(0., min(.1, now-self.last_time))
        self.last_time = now
        with self.lock:
            vx, vy, received = self.command
        if time.monotonic()-received > .4:
            vx = vy = 0.
        p = self.model.pose.position
        x, y = p.x+vx*dt, p.y+vy*dt
        collision = blocked_segment(p.x, p.y, x, y, self.boxes)
        self.blocked_pub.publish(collision)
        if not collision:
            p.x, p.y = x, y
            if math.hypot(vx, vy) > 0:
                yaw = math.atan2(vy, vx)
                self.model.pose.orientation.z = math.sin(yaw/2)
                self.model.pose.orientation.w = math.cos(yaw/2)
        # Set zero physical velocity: the next commanded pose alone defines
        # movement; do not integrate the same velocity again in Gazebo physics.
        try:
            response = self.set_state(self.model)
            if not response.success:
                raise RuntimeError(response.status_message)
        except Exception as exc:
            self.ready.publish(False)
            rospy.logerr('Pedestrian update failed: %s', exc)
            rospy.signal_shutdown(str(exc))


if __name__ == '__main__':
    rospy.init_node('warehouse_pedestrian')
    node = WarehousePedestrian()
    rospy.spin()
