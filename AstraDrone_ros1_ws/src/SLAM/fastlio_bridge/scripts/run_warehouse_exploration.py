#!/usr/bin/env python3
"""Independent warehouse demo. Reuses lifecycle helpers, never edits old demo."""
import argparse
import os
from pathlib import Path
import signal
import sys
import subprocess
import time
import json
import math
import xml.etree.ElementTree as ET
from datetime import datetime
import rosgraph
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String, Empty
from std_srvs.srv import Trigger
from run_dynamic_comparison import CaseRunner, wait_topic, process_running, cleanup_interrupt
from warehouse_scene import prepare, blocked
from startup_checks import wait_ground_pose
from analyze_dynamic_demo_bag import rotation
import numpy as np


def mission_deadline(start_wall, test_seconds, max_wall_seconds):
    """Independent wall-clock fail-safe, even if /clock/data stops.

    Budget termination is PARTIAL, never coverage or return success. Startup
    has its own bounded waits. --keep-open only bypasses this AFTER success.
    """
    if not math.isfinite(test_seconds) or test_seconds<0 or not math.isfinite(max_wall_seconds) or max_wall_seconds<10:
        raise ValueError('Finite nonnegative test time and wall budget >=10s required')
    if test_seconds and test_seconds<max_wall_seconds:
        return start_wall+test_seconds,'explicit_test_timeout_partial'
    return start_wall+max_wall_seconds,'wall_guard_timeout_partial'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--no-gazebo-gui',action='store_true',help='keep RViz, omit expensive Gazebo GUI')
    parser.add_argument('--no-rviz',action='store_true',help='omit RViz but keep Gazebo GUI unless headless')
    parser.add_argument('--manual-pedestrian',action='store_true',help='disable one automatic walking episode')
    parser.add_argument('--test-seconds',type=float,default=0.,help='TEST ONLY: force stop after N wall seconds; not mission success')
    parser.add_argument('--max-wall-seconds',type=float,default=4800.,help='SITL mission hard wall budget incl. return; timeout saves PARTIAL map even if clock/data stalls')
    parser.add_argument('--return-after-goals',type=int,default=0,help='return after N reached viewpoints (0: normal exploration + inspection)')
    parser.add_argument('--keep-open',action='store_true',help='hold simulation open after return and save; default exits')
    parser.add_argument('--freedom-baseline',action='store_true',help='run FreeDOM in parallel on the same FAST-LIO scans; no planner/map-topic replacement')
    args = parser.parse_args()
    try:
        mission_deadline(0.,args.test_seconds,args.max_wall_seconds)
    except ValueError as exc:
        parser.error(str(exc))
    if args.return_after_goals < 0:
        parser.error('return-after-goals must be nonnegative')
    if rosgraph.is_master_online() or any(process_running(n) for n in ('gzserver','px4','fastlio_mapping')):
        parser.error('请先退出旧仿真；不会杀死用户已有进程')
    if not args.headless and not (args.no_gazebo_gui and args.no_rviz) and not os.environ.get('DISPLAY'):
        parser.error('没有DISPLAY，请使用 --headless')
    root,px4,ped = (Path(os.environ[k]) for k in ('ASTRA_ROOT','PX4_DIR','PEDSIM_WS'))
    output = root/'exploration_demo_runs'
    output.mkdir(exist_ok=True)
    prefix = output/('warehouse_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    scripts = root/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'
    env = os.environ.copy()
    for key in ('GAZEBO_PLUGIN_PATH','LD_LIBRARY_PATH'):
        env[key] = ':'.join([str(ped/'devel/lib'),str(root/'simulation/sim_workspace/devel/lib'),env.get(key,'')])
    env['GAZEBO_MODEL_PATH'] = ':'.join([str(root/'simulation/astra_gazebo_models'),
        str(ped/'src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models'),env.get('GAZEBO_MODEL_PATH','')])
    run = CaseRunner(root,prefix,env)
    saver = False
    failed = False
    mission = {}
    termination = 'interrupted'
    audit_alignment = None
    freedom_alignment = None
    truth_snapshot = None
    alignment_subscribers = []
    try:
        core = run.start('core',['roscore'])
        deadline = time.monotonic()+30
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('roscore failed')
            time.sleep(.3)
        # rospy chooses its clock at init_node. Gazebo starts later: setting
        # this only in Gazebo's launch leaves this runner on wall time forever,
        # incorrectly treating every MAVROS simulation timestamp as stale.
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('exploration_runner',anonymous=True,disable_signals=True)
        def status(msg):
            nonlocal mission
            try:
                mission=json.loads(msg.data)
            except (ValueError,TypeError):
                pass
        rospy.Subscriber('/uav1/exploration/status',String,status,queue_size=1)
        # Evaluator only. Never send geometry/truth to Explorer or its goals.
        def truth(msg):
            nonlocal truth_snapshot
            truth_snapshot=(msg,time.monotonic())
        def capture_alignment(msg):
            nonlocal audit_alignment
            snapshot=truth_snapshot
            if audit_alignment is not None or not snapshot or time.monotonic()-snapshot[1]>.05:
                return
            try:
                body=snapshot[0].pose[snapshot[0].name.index('iris_mid360')]
            except ValueError:
                return
            estimate=msg.pose.pose
            r=rotation(estimate.orientation)@rotation(body.orientation).T
            t=np.asarray([estimate.position.x,estimate.position.y,estimate.position.z])-r@np.asarray(
                [body.position.x,body.position.y,body.position.z])
            audit_alignment=dict(map_from_world_rotation=r.tolist(),map_from_world_translation=t.tolist(),
                odom_stamp=msg.header.stamp.to_sec(),truth_receipt_age_wall=time.monotonic()-snapshot[1],
                note='Evaluator-only initial pose alignment; ModelStates has no stamp, receipt pairing <=50ms; '
                     'no long-term SLAM drift correction, not planner input')
            Path(str(prefix)+'_alignment.json').write_text(json.dumps(audit_alignment,indent=2),encoding='utf8')
            # Gazebo Classic publishes ModelStates every physics iteration
            # while subscribed. A permanent audit subscription can itself slow
            # the simulation. We only need this initial alignment snapshot.
            if not args.freedom_baseline or freedom_alignment is not None:
                for subscriber in alignment_subscribers:
                    subscriber.unregister()
        def capture_freedom_alignment(msg):
            nonlocal freedom_alignment
            snapshot=truth_snapshot
            if freedom_alignment is not None or not snapshot or time.monotonic()-snapshot[1]>.05:
                return
            try:
                body=snapshot[0].pose[snapshot[0].name.index('iris_mid360')]
            except ValueError:
                return
            estimate=msg.pose.pose
            r=rotation(estimate.orientation)@rotation(body.orientation).T
            t=np.asarray([estimate.position.x,estimate.position.y,estimate.position.z])-r@np.asarray(
                [body.position.x,body.position.y,body.position.z])
            freedom_alignment=dict(map_from_world_rotation=r.tolist(),map_from_world_translation=t.tolist(),
                odom_stamp=msg.header.stamp.to_sec(),truth_receipt_age_wall=time.monotonic()-snapshot[1],
                note='Evaluator-only world to camera_init, using raw FAST-LIO odometry; not planner input')
            Path(str(prefix)+'_freedom_alignment.json').write_text(json.dumps(freedom_alignment,indent=2),encoding='utf8')
            if audit_alignment is not None:
                for subscriber in alignment_subscribers:
                    subscriber.unregister()
        alignment_subscribers.append(rospy.Subscriber('/gazebo/model_states',ModelStates,truth,queue_size=1))
        alignment_subscribers.append(rospy.Subscriber('/uav1/fastlio/odom',Odometry,capture_alignment,queue_size=1))
        if args.freedom_baseline:
            alignment_subscribers.append(rospy.Subscriber('/Odometry',Odometry,capture_freedom_alignment,queue_size=1))
        world,geometry = prepare(root,prefix)
        meta = json.loads(geometry.read_text())
        if blocked(-8.,-4.,meta['boxes']):
            raise RuntimeError('新demo行人出生点被障碍占据，请核对修改后的world')
        # New demo only: start within view of takeoff, so stationary-person
        # insertion AND later removal are observable without prescribing flight.
        # This edits the generated copy, never cangku.world or the old demo.
        tree = ET.parse(world)
        # Keep the original 1ms/1000Hz physics. The tested 2ms/500Hz
        # lockstep pair is accepted by the plugin but NOT stable for this
        # warehouse vehicle: PX4 ground initialization failed. Do not bypass
        # arming/pose gates or turn that failed experiment into a speed option.
        for actor in tree.getroot().find('world').findall('include'):
            if actor.findtext('name') == '0':
                actor.find('pose').text = '-8 -4 0 0 0 0'
        tree.write(str(world),encoding='utf-8',xml_declaration=True)
        meta['pedestrian_world'] = [-8,-4,0]
        geometry.write_text(json.dumps(meta,indent=2),encoding='utf8')
        setup = px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        launch = root/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch'
        cmd = "source '{}' '{}' '{}'; exec roslaunch '{}' world:='{}' gui:={} interactive:=false x:=-6 y:=-4".format(
            setup,px4,px4/'build/px4_sitl_default',launch,world,'false' if args.headless or args.no_gazebo_gui else 'true')
        gazebo = run.start('gazebo',['bash','-lc',cmd])
        wait_topic('/livox/lidar',PointCloud2,120,gazebo)
        print('等待PX4地面位姿稳定，防止初始化高度变化被当成起飞。',flush=True)
        stable_pose=wait_ground_pose(gazebo)
        print('地面位姿已稳定，MAVROS Z={:.3f}m；开始SLAM和自动起飞。'.format(
            stable_pose.pose.position.z),flush=True)
        lio = run.start('fastlio',['roslaunch','fast_lio','mapping_mid360.launch',
                                 'rviz:=false','sim_lidar:=true','dynamic_filter:=false'])
        wait_topic('/Odometry',Odometry,90,lio)
        if args.freedom_baseline:
            # Independent mapping baseline: uses FAST-LIO's body-frame deskewed
            # scan and time-matched TF. It never publishes to our planner map.
            run.start('freedom',['roslaunch','freedom','astra_mid360_baseline.launch',
                                 'output:='+str(prefix)])
        # Start the human before navigation, so its static initial points really
        # enter the maintained map before it moves. No truth reaches Explorer.
        human = run.start('pedestrian',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                                       '_scene_file:='+str(geometry)])
        wait_topic('/demo/pedestrian/ready',Bool,20,human)
        run.start('exploration',['roslaunch','fastlio_bridge','warehouse_exploration.launch',
                                'rviz:='+('false' if args.headless or args.no_rviz else 'true'),'output:='+str(prefix),
                                'return_after_goals:='+str(args.return_after_goals)])
        rospy.wait_for_service('/demo/save_map',timeout=30)
        saver = True
        if not args.manual_pedestrian:
            run.start('walk_once',[sys.executable,str(scripts/'exploration_pedestrian_walk.py')])
        print('自主探索已启动：探索 → 近距补扫 → 返航 → 稳定5秒 → 保存地图并结束。',flush=True)
        print('返航不等于三维完整；在线报告可见增益/多视点表面质量，退出后独立审计货架侧面。',flush=True)
        print('地图/状态/日志前缀：'+str(prefix),flush=True)
        print('暂停：rosservice call /uav1/exploration/set_enabled "data: false"；改true恢复。',flush=True)
        deadline,timeout_reason = mission_deadline(time.monotonic(),args.test_seconds,args.max_wall_seconds)
        while time.monotonic()<deadline or (args.keep_open and mission.get('state')=='COMPLETE' and mission.get('home_confirmed')):
            for name,process in run.processes:
                if process.poll() is not None:
                    if name == 'walk_once' and process.returncode == 0:
                        continue
                    raise RuntimeError(name+' 提前退出，请检查对应日志')
            if mission.get('state') == 'RETURN_FAILED':
                termination='return_failed'
                raise RuntimeError('返航超时，无法安全到达起点；保存部分地图，不报告成功')
            if mission.get('state') == 'STALLED_UNSAFE':
                termination='stalled_unsafe'
                raise RuntimeError('当前机位连续30秒无法安全脱离；已停止本轮探索并保存部分地图')
            if mission.get('state') == 'COMPLETE' and mission.get('home_confirmed') and not args.keep_open:
                termination='returned_home_and_saved'
                print('已返回起点并稳定，地图保存成功，结束本次仿真。',flush=True)
                break
            time.sleep(.5)
        else:
            termination=timeout_reason
            failed |= timeout_reason=='wall_guard_timeout_partial'
            print('墙钟时限到达（{}）；仅保存部分结果，不算任务完成。'.format(timeout_reason),flush=True)
    except KeyboardInterrupt:
        print('收到Ctrl-C，先保存维护地图，再关闭本次进程。',flush=True)
    except Exception as exc:
        failed = True
        if termination == 'interrupted':
            termination='runtime_error'
        print('探索演示失败：'+str(exc),file=sys.stderr,flush=True)
    finally:
        signal.signal(signal.SIGINT,cleanup_interrupt)
        if saver and rosgraph.is_master_online():
            try:
                result = rospy.ServiceProxy('/demo/save_map',Trigger)()
                print('最终地图：{} {}'.format(result.success,result.message),flush=True)
                failed |= not result.success
            except Exception as exc:
                failed = True
                print('地图保存失败：'+str(exc),flush=True)
        if args.freedom_baseline and rosgraph.is_master_online():
            # Upstream FreeDOM saves two PCDs on Empty; poll the generated file
            # rather than assuming publish() means the mapping thread finished.
            freedom_point = Path(str(prefix)+'_freedom_static_map_point.pcd')
            try:
                pub = rospy.Publisher('/freedom/save_map',Empty,queue_size=1)
                until = time.monotonic()+10
                while pub.get_num_connections()==0 and time.monotonic()<until:
                    time.sleep(.1)
                if pub.get_num_connections()==0:
                    raise RuntimeError('FreeDOM save subscriber unavailable')
                pub.publish(Empty())
                done = rospy.wait_for_message('/freedom/save_map_done',Bool,timeout=30)
                if not done.data or not freedom_point.is_file():
                    raise RuntimeError('FreeDOM save incomplete')
                print('FreeDOM独立基线地图：'+str(freedom_point),flush=True)
                pub.unregister()
            except Exception as exc:
                failed=True
                print('FreeDOM基线地图保存失败：'+str(exc),file=sys.stderr,flush=True)
        # Keep the completion reason separate from map completeness and exit
        # code. Stopping this SITL world is NOT an actual-aircraft landing API.
        summary=dict(termination=termination,failed=bool(failed),mission=mission,
                     freedom_baseline=bool(args.freedom_baseline),
                     freedom_point_map=str(prefix)+'_freedom_static_map_point.pcd' if args.freedom_baseline else None,
                     note='SITL only: save then stop owned simulation processes; no physical landing claim')
        Path(str(prefix)+'_result.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
        run.stop()
        # Audit AFTER simulation shutdown: no contention with flight/sensors.
        # A metric failure must not erase an otherwise valid saved map/result.
        map_path=Path(str(prefix)+'_final_map.pcd')
        if audit_alignment is not None and map_path.is_file():
            try:
                with Path(str(prefix)+'_coverage.log').open('w') as stream:
                    subprocess.run([sys.executable,str(scripts/'warehouse_map_audit.py'),
                        '--scene',str(geometry),'--pcd',str(map_path),
                        '--alignment',str(prefix)+'_alignment.json',
                        '--output',str(prefix)+'_coverage.json'],stdout=stream,stderr=subprocess.STDOUT,
                        check=True,timeout=120)
                print('货架侧面覆盖审计：'+str(prefix)+'_coverage.json；缺口样本：_coverage_missing.pcd',flush=True)
            except (subprocess.SubprocessError,OSError) as exc:
                print('离线覆盖审计失败（已保存地图不受影响）：'+str(exc),flush=True)
        elif saver:
            print('缺少可靠初始world/map对齐或PCD；未伪造覆盖率。',flush=True)
        freedom_map=Path(str(prefix)+'_freedom_static_map_point.pcd')
        if args.freedom_baseline and freedom_alignment is not None and freedom_map.is_file():
            try:
                with Path(str(prefix)+'_freedom_coverage.log').open('w') as stream:
                    subprocess.run([sys.executable,str(scripts/'warehouse_map_audit.py'),
                        '--scene',str(geometry),'--pcd',str(freedom_map),
                        '--alignment',str(prefix)+'_freedom_alignment.json',
                        '--output',str(prefix)+'_freedom_coverage.json'],stdout=stream,stderr=subprocess.STDOUT,
                        check=True,timeout=120)
                print('FreeDOM货架侧面覆盖审计：'+str(prefix)+'_freedom_coverage.json',flush=True)
            except (subprocess.SubprocessError,OSError) as exc:
                print('FreeDOM覆盖审计失败：'+str(exc),file=sys.stderr,flush=True)
        elif args.freedom_baseline:
            print('FreeDOM缺少初始world/camera_init对齐或PCD；未伪造覆盖率。',flush=True)
    return int(failed)


if __name__ == '__main__':
    sys.exit(main())
