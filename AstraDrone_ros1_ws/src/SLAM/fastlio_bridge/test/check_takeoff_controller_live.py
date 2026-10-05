#!/usr/bin/env python3
"""Real compiled controller, isolated fake FCU; never connects to physical PX4."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import argparse


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    folder=Path(args.output); folder.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11359'
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from mavros_msgs.msg import State,ExtendedState
    from mavros_msgs.srv import CommandBool,CommandBoolResponse,SetMode,SetModeResponse
    from std_msgs.msg import Bool,UInt8,String
    if rosgraph.is_master_online(): raise RuntimeError('Isolated master occupied')
    processes=[]; streams=[]
    def start(name,cmd):
        stream=(folder/(name+'.log')).open('w'); streams.append(stream)
        p=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p)
        return p
    try:
        start('core',['roscore','-p','11359'])
        until=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>until: raise RuntimeError('master timeout')
            time.sleep(.1)
        rospy.init_node('fake_ground_fcu',disable_signals=True)
        state=State(connected=True,armed=False,mode='MANUAL')
        ext=ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        calls=[]; observations={}
        def arm(req):
            calls.append(time.monotonic()); state.armed=req.value
            return CommandBoolResponse(success=True,result=0)
        def mode(req):
            state.mode=req.custom_mode
            return SetModeResponse(mode_sent=True)
        rospy.Service('/mavros/cmd/arming',CommandBool,arm)
        rospy.Service('/mavros/set_mode',SetMode,mode)
        pub=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1)
        sp=rospy.Publisher('/mavros/state',State,queue_size=1)
        ep=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1)
        odom_pub=rospy.Publisher('/uav1/fastlio/odom',Odometry,queue_size=1)
        goal_pub=rospy.Publisher('/uav1/planner/waypoint',PoseStamped,queue_size=1)
        risk_pub=rospy.Publisher('/uav1/fused_collision_risk_level',UInt8,queue_size=1)
        avoid_pub=rospy.Publisher('/uav1/fused_avoidance_velocity',TwistStamped,queue_size=1)
        scan_pub=rospy.Publisher('/uav1/fastlio/registered_scan',PointCloud2,queue_size=1)
        recovery_pub=rospy.Publisher('/uav1/exploration/recovery_waypoint',PoseStamped,queue_size=1)
        source_pub=rospy.Publisher('/uav1/fused_avoidance_source',String,queue_size=1)
        dynamic_risk_pub=rospy.Publisher('/uav1/collision_risk_level',UInt8,queue_size=1)
        rospy.Subscriber('/test_controller/command_preview',TwistStamped,
                         lambda m:observations.update(command=m),queue_size=1)
        rospy.Subscriber('/test_controller/takeoff_complete',Bool,
                         lambda m:observations.update(ready=m.data),queue_size=1)
        node=start('controller',['rosrun','fastlio_bridge','mavros_avoidance_controller_node',
            '__name:=test_controller','_enable_control:=true','_auto_arm:=true',
            '_auto_goal:=false','_takeoff_height:=1.2',
            '_hold_on_level2:=true','_require_fresh_scan:=true',
            '_static_recovery_speed:=0.0'])
        def feed(z,seconds,publish_pose=True,pose_age=0.,navigation_risk=None,
                 publish_scan=True,recovery=False,scan_x=3.,dynamic_risk=0,
                 mismatch_recovery=False):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                if node.poll() is not None: raise RuntimeError('controller exited')
                stamp=rospy.Time.now()
                pose=PoseStamped(); pose.header.stamp=stamp-rospy.Duration(pose_age); pose.pose.orientation.w=1.
                pose.pose.position.z=z
                state.header.stamp=ext.header.stamp=stamp
                if publish_pose: pub.publish(pose)
                sp.publish(state); ep.publish(ext)
                if navigation_risk is not None:
                    odom=Odometry(); odom.header.stamp=stamp
                    odom.header.frame_id='map'; odom.pose.pose.position.z=z
                    odom_pub.publish(odom)
                    goal=PoseStamped(); goal.header.stamp=stamp
                    goal.header.frame_id='map'; goal.pose.position.x=-.2 if recovery and not mismatch_recovery else 2.
                    goal.pose.position.z=z; goal.pose.orientation.w=1.
                    goal_pub.publish(goal)
                    risk_pub.publish(UInt8(data=navigation_risk))
                    avoid=TwistStamped(); avoid.header.stamp=stamp
                    avoid.twist.linear.x=.6; avoid.twist.linear.y=.2
                    avoid_pub.publish(avoid)
                    source_pub.publish(String(data='static_avoid'))
                    dynamic_risk_pub.publish(UInt8(data=dynamic_risk))
                    if recovery:
                        escape=PoseStamped(); escape.header.stamp=stamp; escape.header.frame_id='map'
                        escape.pose.position.x=-.2; escape.pose.position.z=z; escape.pose.orientation.w=1.
                        recovery_pub.publish(escape)
                    if publish_scan:
                        header=odom.header
                        scan_pub.publish(point_cloud2.create_cloud_xyz32(header,[(scan_x,0.,z)]))
                time.sleep(.04)
        feed(-1.87,1.)
        for i in range(10): feed(-1.87+(i+1)*.187,.1)
        assert not calls,'armed while origin unstable'
        feed(0.,6.)
        assert calls and state.armed,'did not arm after stable ground'
        assert observations['command'].twist.linear.z>0,'wrong takeoff direction'
        for i in range(12): feed((i+1)*.1,.15)
        feed(1.2,1.5)
        assert not observations.get('ready',False),'ground flag falsely accepted as airborne'
        ext.landed_state=ExtendedState.LANDED_STATE_IN_AIR
        feed(1.2,1.5)
        assert observations.get('ready',False),'airborne altitude dwell failed'
        feed(1.2,.5,navigation_risk=0)
        assert observations['command'].twist.linear.x>.05,'fresh clear route did not navigate'
        feed(1.2,.5,navigation_risk=2)
        assert abs(observations['command'].twist.linear.x)<1e-6 and abs(observations['command'].twist.linear.y)<1e-6, 'level2 emergency still commanded horizontal escape'
        feed(1.2,.5,navigation_risk=2,recovery=True,scan_x=.55)
        assert -.081<observations['command'].twist.linear.x<-.02,'checked escape did not leave level2 shelf'
        feed(1.2,.5,navigation_risk=2,recovery=True,scan_x=.43)
        assert abs(observations['command'].twist.linear.x)<1e-6,'escape crossed rotor margin'
        feed(1.2,.5,navigation_risk=2,recovery=True,scan_x=.55,mismatch_recovery=True)
        assert abs(observations['command'].twist.linear.x)<1e-6,'mismatched escape goal authorized flight'
        feed(1.2,.5,navigation_risk=2,recovery=True,scan_x=.55,dynamic_risk=2)
        assert abs(observations['command'].twist.linear.x)<1e-6,'moving person did not block static escape'
        feed(1.2,.5,navigation_risk=0,scan_x=.75)
        assert abs(observations['command'].twist.linear.x)<1e-6,'forward braking did not stop before shelf'
        feed(1.2,.6,navigation_risk=0,publish_scan=False)
        assert abs(observations['command'].twist.linear.x)<1e-6, 'stale scan allowed navigation'
        feed(1.2,.5,navigation_risk=0)
        assert observations['command'].twist.linear.x>.05,'fresh scan did not resume navigation'
        arm_count=len(calls)
        feed(1.2,.8,publish_pose=False)
        assert not observations['ready'] and observations['command'].twist.linear.z==0,'stale pose did not hold'
        feed(1.2,.5)
        assert observations['ready'] and len(calls)==arm_count,'fresh airborne pose failed to recover without rearm'
        feed(1.2,.3,pose_age=.30)
        feed(1.2,.18,publish_pose=False)
        assert not observations['ready'],'recent receipt hid stale acquisition stamp'
        feed(1.2,.5)
        assert observations['ready'] and len(calls)==arm_count
        feed(2.2,.5) # coordinate jump must not become a new climb
        assert not observations['ready'] and observations['command'].twist.linear.z==0
        state.armed=False
        feed(2.2,2.5)
        assert len(calls)==arm_count,'automatically rearmed after flight fault'
        result=dict(passed=True,checks=['no_early_arm','positive_takeoff_velocity',
                    'ground_not_airborne','airborne_dwell','stale_pose_hold',
                    'fresh_pose_recovery_without_rearm','acquisition_stale_hold',
                    'reset_hold','no_fault_rearm','fresh_scan_navigation',
                    'level2_xy_hold','level2_guarded_escape','rotor_margin_hold',
                    'mismatched_escape_hold','dynamic_escape_hold','forward_brake',
                    'stale_scan_hold','fresh_scan_recovery'])
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
        for s in streams: s.close()


if __name__=='__main__': main()
