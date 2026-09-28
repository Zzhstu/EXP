#!/usr/bin/env python3
"""Single owner of Gazebo actor poses: scripted input, then optional keyboard.

No model spawning or competing SetModelState calls. Manual motion is world XY,
not collision-aware path planning; do not drive actors through solid geometry.
"""
import copy
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class PedestrianRelay:
    def __init__(self):
        from pedsim_msgs.msg import AgentStates
        self.lock = threading.RLock()
        self.source = None
        self.manual = {}
        self.commands = {}
        self.ready = False
        self.last_time = None
        self.destination_since = None
        self.source_time = 0.0
        self.pub = rospy.Publisher('/pedsim_simulator/simulated_agents', AgentStates, queue_size=1)
        self.ready_pub = rospy.Publisher('/demo/pedestrian/ready', Bool, queue_size=1, latch=True)
        self.ready_pub.publish(False)
        rospy.Subscriber('/demo/pedestrian/scripted_agents', AgentStates, self.receive, queue_size=1)
        # Restrict to models present in example.world; never spawn missing IDs.
        for actor in (0, 1):
            rospy.Subscriber('/demo/pedestrian/%d/cmd_vel' % actor, Twist,
                             self.command, callback_args=actor, queue_size=1)
        rospy.Timer(rospy.Duration(0.04), self.tick)

    def receive(self, msg):
        with self.lock:
            self.source = msg
            self.source_time = time.monotonic()
            positions = {a.id: (a.pose.position.x, a.pose.position.y) for a in msg.agent_states}
            # Crowd repulsion need not converge to zero speed at a shared
            # waypoint. Require BOTH actors to dwell in the destination region
            # for 3 simulation seconds, then explicitly finish scripted motion.
            at_destination = len(positions) >= 2 and all(
                math.hypot(x-3.8, y-2.5) < .95 for x, y in positions.values())
            now = rospy.Time.now().to_sec()
            self.destination_since = (self.destination_since if self.destination_since is not None else now) \
                if at_destination else None
            if not self.ready and self.destination_since is not None and now-self.destination_since > 3.0:
                # PedSim social forces can keep nudging agents at a reached
                # waypoint. Freeze ALL final poses once; only the selected
                # actor moves thereafter. Never keep resetting a manual pose.
                self.manual = {a.id: copy.deepcopy(a) for a in msg.agent_states}
                self.ready = True
                self.ready_pub.publish(True)
                rospy.loginfo('Pedestrian keyboard ready: scripted crossing finished.')

    def command(self, msg, actor):
        with self.lock:
            if not self.ready or self.source is None:
                rospy.logwarn_throttle(2, 'Keyboard locked until both pedestrians finish the crossing.')
                return
            vx, vy = msg.linear.x, msg.linear.y
            if not math.isfinite(vx) or not math.isfinite(vy):
                return
            # Clamp independently of the client; zero command means hold, not return
            # to the stale scripted endpoint (which would teleport the actor).
            speed = math.hypot(vx, vy)
            scale = min(1.0, 1.0 / max(speed, 1e-9))
            if actor not in self.manual:
                agent = next((a for a in self.source.agent_states if a.id == actor), None)
                if agent is None:
                    return
                self.manual[actor] = copy.deepcopy(agent)
            self.commands[actor] = (vx*scale, vy*scale, time.monotonic())

    def tick(self, event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            dt = 0 if self.last_time is None else max(0, min(0.1, now-self.last_time))
            self.last_time = now
            if self.source is None:
                return
            out = copy.deepcopy(self.source)
            out.header.stamp = rospy.Time.now()
            for i, agent in enumerate(out.agent_states):
                if agent.id not in self.manual:
                    continue
                a = self.manual[agent.id]
                vx, vy, received = self.commands.get(agent.id, (0, 0, 0))
                # Wall-clock deadman also stops motion after a frozen terminal.
                if time.monotonic()-received > 0.4 or time.monotonic()-self.source_time > 1.0:
                    vx = vy = 0
                a.pose.position.x += vx*dt
                a.pose.position.y += vy*dt
                a.twist.linear.x, a.twist.linear.y = vx, vy
                if math.hypot(vx, vy) > 0:
                    yaw = math.atan2(vy, vx)
                    a.pose.orientation.x = a.pose.orientation.y = 0
                    a.pose.orientation.z = math.sin(yaw/2)
                    a.pose.orientation.w = math.cos(yaw/2)
                out.agent_states[i] = copy.deepcopy(a)
            self.pub.publish(out)


def keyboard():
    if not sys.stdin.isatty():
        raise RuntimeError('需要真实终端；自动测试请向 /demo/pedestrian/0/cmd_vel 发布 Twist')
    pubs = {i: rospy.Publisher('/demo/pedestrian/%d/cmd_vel' % i, Twist, queue_size=1) for i in (0, 1)}
    rospy.Subscriber('/demo/pedestrian/ready', Bool,
                     lambda m: print('行人控制：' + ('已解锁' if m.data else '等待初期横穿结束'), flush=True))
    print('0/1选人；W/S=世界+Y/-Y，A/D=世界-X/+X；按住移动；空格停止；Q退出。速度0.6m/s。')
    actor = 0
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            # TextIOWrapper can prefetch multiple keys, hiding buffered keys
            # from select(). Read the terminal fd directly (e.g. "0w q").
            key = os.read(sys.stdin.fileno(), 1).decode('ascii', errors='ignore').lower()
            if key in ('q', '\x03'):
                break
            if key in ('0', '1'):
                pubs[actor].publish(Twist())
                actor = int(key)
                print('选择行人', actor, flush=True)
            elif key in ('w', 'a', 's', 'd', ' '):
                x, y = {'w': (0, .6), 's': (0, -.6), 'a': (-.6, 0),
                        'd': (.6, 0), ' ': (0, 0)}[key]
                cmd = Twist()
                cmd.linear.x, cmd.linear.y = x, y
                pubs[actor].publish(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        for pub in pubs.values():
            pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    is_keyboard = '--keyboard' in sys.argv
    rospy.init_node('pedestrian_keyboard' if is_keyboard else 'pedestrian_relay')
    if is_keyboard:
        keyboard()
    else:
        relay = PedestrianRelay()
        rospy.spin()
