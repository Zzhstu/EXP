#!/usr/bin/env python3
"""Real C++ node regression on an isolated ROS master, no Gazebo/truth injection.

Runs legacy and evidence-only nodes on identical synthetic measured returns.
Logs and quantitative outcomes remain in the requested output directory.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--port',type=int,default=11339)
    args=parser.parse_args()
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    import rosgraph
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from std_srvs.srv import Empty
    from fastlio_bridge.msg import DynamicObjectArray,DynamicObject
    if rosgraph.is_master_online():
        raise RuntimeError('Isolated master port already in use; refusing to touch it')
    processes=[]
    handles=[]
    result={}
    def start(name,cmd):
        f=(output/(name+'.log')).open('w')
        handles.append(f)
        processes.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        start('core',['roscore','-p',str(args.port)])
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline: raise RuntimeError('master timeout')
            time.sleep(.1)
        rospy.init_node('map_evidence_test',disable_signals=True)
        for name in ('legacy','evidence'):
            start(name,['rosrun','fastlio_bridge','static_map_builder_node','__name:='+name,
                '_cloud_topic:=/fixture/scan','_odom_topic:=/fixture/odom','_objects_topic:=/fixture/objects',
                '_output_topic:=/'+name+'/map','_legacy_output_topic:=/'+name+'/old',
                '_stable_output_topic:=/'+name+'/stable','_local_output_topic:=/'+name+'/local',
                '_free_space_output_topic:=/'+name+'/free','_minimum_static_hits:=1',
                '_stable_minimum_static_hits:=3','_stable_free_miss_threshold:=3',
                '_free_space_miss_threshold:=3','_publish_rate:=20',
                '_evidence_only_clearing:='+('true' if name=='evidence' else 'false')])
            rospy.wait_for_service('/'+name+'/clear',10)
        scans=rospy.Publisher('/fixture/scan',PointCloud2,queue_size=1)
        odom=rospy.Publisher('/fixture/odom',Odometry,queue_size=1)
        objects=rospy.Publisher('/fixture/objects',DynamicObjectArray,queue_size=1)
        maps={}
        def receive(msg,name):
            maps[name]=list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True))
        for name in ('legacy','evidence'):
            rospy.Subscriber('/'+name+'/map',PointCloud2,receive,name,queue_size=1)
        time.sleep(.5)
        def send(points,repeat=1,dynamic=False,stale=False):
            for _ in range(repeat):
                stamp=rospy.Time.now()
                o=Odometry(header=Header(stamp=stamp-rospy.Duration(5 if stale else 0),frame_id='map'))
                o.pose.pose.position.z=1.02
                o.pose.pose.orientation.w=1.
                odom.publish(o)
                time.sleep(.015)
                m=DynamicObjectArray(header=Header(stamp=stamp,frame_id='map'))
                if dynamic:
                    obj=DynamicObject(id=1)
                    obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=2.1,.03,1.02
                    obj.twist.linear.x=.3
                    m.objects.append(obj)
                objects.publish(m)
                scans.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='map'),points))
                time.sleep(.12)
        def present(name,p):
            return any(sum((a-b)**2 for a,b in zip(q,p))<.001 for q in maps.get(name,[]))
        def reset():
            for name in ('legacy','evidence'):
                rospy.ServiceProxy('/'+name+'/clear',Empty)()
            time.sleep(.15)
        rack=(2.1,.03,1.02)
        send([rack],10)
        send([rack],8,dynamic=True)
        result['false_track_static_rack_preserved']=present('evidence',rack)
        result['legacy_false_track_erased_rack']=not present('legacy',rack)
        reset()
        person=(1.05,.015,1.02)
        send([person,rack],10)
        result['stationary_person_initially_in_map']=present('evidence',person)
        send([rack],22)
        result['vacated_person_cleared']=not present('evidence',person)
        result['background_rack_preserved']=present('evidence',rack)
        reset()
        # Same voxel traversal but 8cm away from the stored thin surface.
        edge=(1.06,.13,1.02)
        send([edge],10)
        send([(2.1,.1,1.02)],22)
        result['off_ray_thin_structure_preserved']=present('evidence',edge)
        reset()
        send([person,rack],10)
        send([rack],22,stale=True)
        result['stale_pose_does_not_clear']=present('evidence',person)
        reset()
        send([person,rack],10)
        send([(0.,2.,1.02)],6)  # wait for minimum observation age
        for _ in range(4):
            send([rack])
            send([(0.,2.,1.02)],9)  # no evidence here for >1 second
        result['separated_misses_do_not_accumulate']=present('evidence',person)
        send([rack],22)
        result['fresh_reobservation_still_clears']=not present('evidence',person)
        result['all_passed']=all(result.values())
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        return 0 if result['all_passed'] else 1
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait()
        for f in handles: f.close()


if __name__=='__main__':
    raise SystemExit(main())
