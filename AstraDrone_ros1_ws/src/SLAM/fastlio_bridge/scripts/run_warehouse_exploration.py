#!/usr/bin/env python3
"""Independent warehouse demo. Reuses lifecycle helpers, never edits old demo."""
import argparse
import os
from pathlib import Path
import signal
import sys
import time
import json
import xml.etree.ElementTree as ET
from datetime import datetime
import rosgraph
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from run_dynamic_comparison import CaseRunner, wait_topic, process_running, cleanup_interrupt
from warehouse_scene import prepare, blocked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--manual-pedestrian',action='store_true',help='disable one automatic walking episode')
    parser.add_argument('--test-seconds',type=float,default=0.,help='TEST ONLY: force stop after N wall seconds; not mission success')
    parser.add_argument('--return-after-goals',type=int,default=0,help='return after N reached viewpoints (0: normal exploration + inspection)')
    parser.add_argument('--keep-open',action='store_true',help='hold simulation open after return and save; default exits')
    args = parser.parse_args()
    if args.test_seconds < 0:
        parser.error('test-seconds must be nonnegative')
    if args.return_after_goals < 0:
        parser.error('return-after-goals must be nonnegative')
    if rosgraph.is_master_online() or any(process_running(n) for n in ('gzserver','px4','fastlio_mapping')):
        parser.error('请先退出旧仿真；不会杀死用户已有进程')
    if not args.headless and not os.environ.get('DISPLAY'):
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
    try:
        core = run.start('core',['roscore'])
        deadline = time.monotonic()+30
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('roscore failed')
            time.sleep(.3)
        rospy.init_node('exploration_runner',anonymous=True,disable_signals=True)
        def status(msg):
            nonlocal mission
            try:
                mission=json.loads(msg.data)
            except (ValueError,TypeError):
                pass
        rospy.Subscriber('/uav1/exploration/status',String,status,queue_size=1)
        world,geometry = prepare(root,prefix)
        meta = json.loads(geometry.read_text())
        if blocked(-8.,-4.,meta['boxes']):
            raise RuntimeError('新demo行人出生点被障碍占据，请核对修改后的world')
        # New demo only: start within view of takeoff, so stationary-person
        # insertion AND later removal are observable without prescribing flight.
        # This edits the generated copy, never cangku.world or the old demo.
        tree = ET.parse(world)
        for actor in tree.getroot().find('world').findall('include'):
            if actor.findtext('name') == '0':
                actor.find('pose').text = '-8 -4 0 0 0 0'
        tree.write(str(world),encoding='utf-8',xml_declaration=True)
        meta['pedestrian_world'] = [-8,-4,0]
        geometry.write_text(json.dumps(meta,indent=2),encoding='utf8')
        setup = px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        launch = root/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch'
        cmd = "source '{}' '{}' '{}'; exec roslaunch '{}' world:='{}' gui:={} interactive:=false x:=-6 y:=-4".format(
            setup,px4,px4/'build/px4_sitl_default',launch,world,'false' if args.headless else 'true')
        gazebo = run.start('gazebo',['bash','-lc',cmd])
        wait_topic('/livox/lidar',PointCloud2,120,gazebo)
        lio = run.start('fastlio',['roslaunch','fast_lio','mapping_mid360.launch',
                                 'rviz:=false','sim_lidar:=true','dynamic_filter:=false'])
        wait_topic('/Odometry',Odometry,90,lio)
        # Start the human before navigation, so its static initial points really
        # enter the maintained map before it moves. No truth reaches Explorer.
        human = run.start('pedestrian',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                                       '_scene_file:='+str(geometry)])
        wait_topic('/demo/pedestrian/ready',Bool,20,human)
        run.start('exploration',['roslaunch','fastlio_bridge','warehouse_exploration.launch',
                                'rviz:='+('false' if args.headless else 'true'),'output:='+str(prefix),
                                'return_after_goals:='+str(args.return_after_goals)])
        rospy.wait_for_service('/demo/save_map',timeout=30)
        saver = True
        if not args.manual_pedestrian:
            run.start('walk_once',[sys.executable,str(scripts/'exploration_pedestrian_walk.py')])
        print('自主探索已启动：探索 → 近距补扫 → 返航 → 稳定5秒 → 保存地图并结束。',flush=True)
        print('返航结束不代表所有遮挡表面完整；状态文件会记录结束原因。',flush=True)
        print('地图/状态/日志前缀：'+str(prefix),flush=True)
        print('暂停：rosservice call /uav1/exploration/set_enabled "data: false"；改true恢复。',flush=True)
        deadline = time.monotonic()+args.test_seconds if args.test_seconds else float('inf')
        while time.monotonic()<deadline:
            for name,process in run.processes:
                if process.poll() is not None:
                    if name == 'walk_once' and process.returncode == 0:
                        continue
                    raise RuntimeError(name+' 提前退出，请检查对应日志')
            if mission.get('state') == 'RETURN_FAILED':
                termination='return_failed'
                raise RuntimeError('返航超时，无法安全到达起点；保存部分地图，不报告成功')
            if mission.get('state') == 'COMPLETE' and mission.get('home_confirmed') and not args.keep_open:
                termination='returned_home_and_saved'
                print('已返回起点并稳定，地图保存成功，结束本次仿真。',flush=True)
                break
            time.sleep(.5)
        else:
            termination='explicit_test_timeout_partial'
            print('显式测试时限到达；仅保存部分结果，不算任务完成。',flush=True)
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
        # Keep the completion reason separate from map completeness and exit
        # code. Stopping this SITL world is NOT an actual-aircraft landing API.
        summary=dict(termination=termination,failed=bool(failed),mission=mission,
                     note='SITL only: save then stop owned simulation processes; no physical landing claim')
        Path(str(prefix)+'_result.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
        run.stop()
    return int(failed)


if __name__ == '__main__':
    sys.exit(main())
