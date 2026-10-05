"""Ground-only startup checks. Never publishes flight commands or uses truth."""
import math
import time
import threading
from collections import deque
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState


class GroundPoseWindow:
    def __init__(self, seconds=4., z_range=.12, xy_range=.25):
        self.seconds, self.z_range, self.xy_range = seconds, z_range, xy_range
        self.samples = deque()

    def update(self, stamp, xyz, on_ground=True):
        if not on_ground or stamp <= 0 or not all(math.isfinite(v) for v in (stamp,*xyz)):
            self.samples.clear()
            return False
        if self.samples and stamp==self.samples[-1][0]:
            return False # no new evidence, but preserve genuine prior samples
        if self.samples and (stamp<self.samples[-1][0] or stamp-self.samples[-1][0]>.5):
            self.samples.clear()
        self.samples.append((stamp,*xyz))
        while len(self.samples)>1 and self.samples[1][0]<=stamp-self.seconds:
            self.samples.popleft()
        spans=[max(s[k] for s in self.samples)-min(s[k] for s in self.samples) for k in (1,2,3)]
        return (stamp-self.samples[0][0]>=self.seconds and spans[2]<=self.z_range
                and math.hypot(*spans[:2])<=self.xy_range)


def ground_state_fresh(ext, wall_now, sim_now):
    # PX4 publishes this around 1 Hz in SIM time. A slow Gazebo can take >2
    # wall seconds per message; retain both acquisition and wall-stop guards.
    return bool(ext and wall_now-ext[1]<5. and
                -.1<=sim_now-ext[0].header.stamp.to_sec()<2. and
                ext[0].landed_state==ExtendedState.LANDED_STATE_ON_GROUND)


def wait_ground_pose(process, timeout=90.):
    # Gate FAST-LIO/bridge startup as well as takeoff: first-sample alignment
    # must not lock an EKF transient into every later map/scan coordinate.
    window=GroundPoseWindow()
    latest={}
    lock=threading.Lock()
    def receive(msg):
        with lock:
            latest['extended']=(msg,time.monotonic())
    def receive_pose(msg):
        with lock:
            ext=latest.get('extended')
            ground=ground_state_fresh(ext,time.monotonic(),rospy.Time.now().to_sec())
            age=(rospy.Time.now()-msg.header.stamp).to_sec()
            p=msg.pose.position
            latest['ready']=window.update(msg.header.stamp.to_sec(),(p.x,p.y,p.z),
                                           ground and -.1<=age<=.35)
            latest['pose']=(msg,time.monotonic())
    sub=rospy.Subscriber('/mavros/extended_state',ExtendedState,receive,queue_size=1)
    # Persistent subscription matters: repeatedly creating wait_for_message
    # subscriptions can replay a cached sample and reset the timestamp window.
    pose_sub=rospy.Subscriber('/mavros/local_position/pose',PoseStamped,receive_pose,queue_size=1)
    deadline=time.monotonic()+timeout
    try:
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if process.poll() is not None:
                raise RuntimeError('Gazebo/PX4 exited while waiting for stable ground pose')
            with lock:
                pose=latest.get('pose')
                ext=latest.get('extended')
                if latest.get('ready') and pose and time.monotonic()-pose[1]<.5 and ground_state_fresh(
                        ext,time.monotonic(),rospy.Time.now().to_sec()):
                    return pose[0]
            time.sleep(.1)
        raise RuntimeError('PX4地面位姿未稳定：未启动自动飞行；检查MAVROS/PX4日志')
    finally:
        sub.unregister()
        pose_sub.unregister()
