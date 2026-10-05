#!/usr/bin/env python3
"""Autonomous frontier exploration on observed space; fixed-altitude simulator.

Never subscribes to Gazebo truth. The maintained map clears historical people;
CURRENT registered returns remain safety obstacles even when dynamic-masked
out of the persistent map. No semantic claim is made by this planner.
"""
import json
import math
import os
from pathlib import Path
import threading
import time
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
# catkin devel relays execute into a separate context and cannot be imported as
# library modules. Prefer this actual source/install directory over relay dir.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from exploration_grid import ExplorationGrid
from exploration_native import SEARCH
from exploration_coverage import (SurfaceEvidence, ProgressWindow, suppress_disk,
                                  representative_cells, visible_unknown_gain)


def xyz(msg):
    """Zero-copy structured PointCloud2 view, including organized row padding."""
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ('x','y','z')):
        raise ValueError('Expected FLOAT32 XYZ cloud')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[k].offset for k in ('x','y','z')], itemsize=msg.point_step))
    view = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step,msg.point_step))
    points = np.stack([view[k].ravel() for k in ('x','y','z')], axis=1)
    return points[np.isfinite(points).all(axis=1)]


class Explorer:
    def __init__(self):
        self.p = lambda k, default: rospy.get_param('~'+k, default)
        # Fail explicitly on edits which would fake evidence / divide by zero.
        # Do not silently accept e.g. quality_target=0 or min_views=0.
        for key,default,lower,upper in (
                ('planning_hz',2.,.1,30.),('visualization_hz',.5,.1,30.),
                ('surface_voxel',.4,.15,1.),('surface_min_hits',3,2,100),
                ('surface_min_views',2,1,8),('surface_view_baseline',.75,.3,10.),
                ('surface_quality_target',.95,.5,1.),('convergence_seconds',90.,1.,3600.),
                ('convergence_free_area',1.,.01,1000.),('convergence_new_surfaces',25,1,100000),
                ('frontier_min_gain_area',.5,.01,100.),('inspection_budget_seconds',600.,1.,3600.),
                ('clearance_cost_weight',1.5,0,100),('recovery_command_horizon',.2,.03,.25)):
            value=float(self.p(key,default))
            if not math.isfinite(value) or not lower<=value<=upper:
                raise ValueError('%s must be finite in [%s,%s]' % (key,lower,upper))
        radius = float(self.p('initial_window_radius',12.0))
        if not math.isfinite(radius) or radius < 4.0:
            raise ValueError('initial_window_radius must be finite and >= 4 m')
        self.grid = ExplorationGrid([-radius,-radius,radius,radius],
                                    self.p('resolution',.2),self.p('clearance',.8),
                                    self.p('unknown_margin',.4),self.p('max_grid_cells',200000))
        self.lock = threading.Lock()
        self.inputs = {}
        self.takeoff_state = None
        self.paused = False
        self.target = None
        self.blacklist = []
        self.visited = []
        self.inspection_visited = []
        self.home = self.altitude = None
        self.home_xy = None
        self.phase = 'EXPLORING'
        self.finish_reason = ''
        self.mission_started = self.return_started = self.home_since = None
        self.home_confirmed = False
        self.return_requested = False
        self.inspected = np.zeros(self.grid.shape,dtype=bool)
        self.failed_visits = {}
        self.target_kind = 'frontier'
        self.ready_since = self.no_frontier_since = None
        self.settle_until = 0.
        self.last_progress = 0.
        self.progress_position = None
        self.unsafe_since = None
        self.unsafe_position = None
        self.last_save = time.monotonic()
        self.last_free = None
        self.cache = {}
        self.goals_selected = self.goals_reached = self.failures = 0
        self.start_wall = time.monotonic()
        self.travel = 0.
        self.previous = None
        self.surface = SurfaceEvidence(self.p('surface_voxel',.4),
            self.p('surface_min_hits',3),self.p('surface_min_views',2),
            self.p('surface_view_baseline',.75),self.p('surface_max_voxels',150000))
        self.progress = ProgressWindow(self.p('convergence_seconds',90.),
            self.p('convergence_free_area',1.),self.p('convergence_new_surfaces',25))
        self.surface_report = {}
        self.progress_report = {}
        self.exploration_complete = False
        self.grid_capacity_limited = False
        self.raw_frontier_count = self.useful_frontier_count = 0
        self.remaining_inspection_count = 0
        self.last_visualization = -float('inf')
        self.last_written_state = None
        self.tick_times = []
        self.last_compute_ms = 0.
        self.active_wall = self.active_sim = None
        self.inspection_started = None
        self.inspection_last_plan = -float('inf')
        self.inspection_plan = None
        self.report_file = Path(self.p('report','/tmp/warehouse_exploration.json'))
        for name, topic, cls in (
            ('odom','/uav1/fastlio/odom',Odometry),
            ('mavros','/mavros/local_position/pose',PoseStamped),
            ('map','/uav1/fastlio/cloud_map',PointCloud2),
            ('scan','/uav1/fastlio/registered_scan',PointCloud2),
            ('free','/uav1/local_free_space',PointCloud2)):
            rospy.Subscriber(topic,cls,self.receive,name,queue_size=1,buff_size=16000000)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
        rospy.Subscriber('/mavros_avoidance_controller/takeoff_complete',Bool,
                         self.receive_takeoff,queue_size=1)
        self.waypoint_pub = rospy.Publisher('/uav1/planner/waypoint',PoseStamped,queue_size=1)
        # Dedicated, non-latched authorization. The flight controller accepts
        # level-1/2 motion only while this short escape goal is fresh and also
        # matches the ordinary waypoint; a stale escape cannot move the UAV.
        self.recovery_pub = rospy.Publisher('/uav1/exploration/recovery_waypoint',PoseStamped,queue_size=1)
        self.path_pub = rospy.Publisher('/uav1/planner/local_path',RosPath,queue_size=1,latch=True)
        self.grid_pub = rospy.Publisher('/uav1/exploration/grid',OccupancyGrid,queue_size=1,latch=True)
        self.status_pub = rospy.Publisher('/uav1/exploration/status',String,queue_size=1,latch=True)
        self.markers_pub = rospy.Publisher('/uav1/exploration/markers',MarkerArray,queue_size=1,latch=True)
        rospy.Service('/uav1/exploration/set_enabled',SetBool,self.enable)
        rospy.Service('/uav1/exploration/return_home',Trigger,self.request_return)

    def request_return(self, req):
        # Service callbacks only request a transition; tick owns mission state.
        self.return_requested = True
        return TriggerResponse(True,'Return requested; observed-space planning and safety guards remain active')

    def begin_return(self, reason, now):
        self.phase = 'RETURNING'
        self.finish_reason = reason
        self.return_started = now
        self.home_since = None
        self.target = None
        self.settle_until = 0.

    def mark_inspected(self, position):
        # Close-range *viewpoint* coverage, not a claim of 3D surface coverage.
        # Limit marking to line-of-sight in the occupied grid: walls must not
        # make the opposite aisle appear inspected simply by Euclidean distance.
        start = self.grid.cell(position)
        r = int(math.ceil(self.p('inspection_radius',2.0)/self.grid.resolution))
        for y in range(max(0,start[0]-r),min(self.grid.shape[0],start[0]+r+1)):
            for x in range(max(0,start[1]-r),min(self.grid.shape[1],start[1]+r+1)):
                if (y-start[0])**2+(x-start[1])**2<=r*r and self.grid.safe[y,x] and not self.inspected[y,x]:
                    if self.grid.segment_safe(position,self.grid.xy((y,x))):
                        self.inspected[y,x]=True

    def return_tick(self, now, position, start, distances, parent):
        home = self.grid.cell(self.home_xy)
        error = math.dist(position,self.home_xy)
        if error <= self.p('home_tolerance',.30):
            self.home_since = now if self.home_since is None else self.home_since
            if now-self.home_since >= self.p('home_settle_seconds',5.):
                self.home_confirmed = True
                # Save confirmation is part of completion. Failed save is
                # retried while holding at home, never reported as success.
                if self.save():
                    self.phase = 'COMPLETE'
                    self.publish('COMPLETE',position)
                else:
                    self.publish('HOME_SAVE_FAILED',position)
            else:
                self.publish('HOME_SETTLE',position)
            return
        self.home_since = None
        if now-self.return_started > self.p('return_timeout',1200.):
            self.phase = 'RETURN_FAILED'
            self.publish('RETURN_FAILED',position)
            return
        if not self.grid.inside(home) or not np.isfinite(distances[home]):
            self.publish('RETURN_BLOCKED',position)
            return
        self.target = home
        path = self.grid.path(parent,start,home)
        self.publish('RETURNING',position,path)

    def receive(self, msg, name):
        if name != 'mavros' and msg.header.frame_id.lstrip('/') != 'map':
            rospy.logerr_throttle(5,'Exploration rejects non-map frame on %s',name)
            return
        with self.lock:
            self.inputs[name] = (msg,time.monotonic())

    def receive_takeoff(self, msg):
        with self.lock:
            self.takeoff_state = (bool(msg.data),time.monotonic())

    def enable(self, req):
        self.paused = not req.data
        return SetBoolResponse(True,'exploration enabled' if req.data else 'hover requested')

    def pose(self, xy):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = 'map',rospy.Time.now()
        msg.pose.position.x,msg.pose.position.y = xy
        msg.pose.position.z = self.altitude or 0.
        msg.pose.orientation.w = 1.
        return msg

    def cloud(self, name, inputs):
        msg = inputs[name][0]
        if name not in self.cache or self.cache[name][0] is not msg:
            self.cache[name] = (msg,xyz(msg))
        return self.cache[name][1]

    def slice(self, points, below=None, above=None):
        lower = self.p('height_below',.45) if below is None else below
        upper = self.p('height_above',.55) if above is None else above
        return points[(points[:,2] >= self.altitude-lower) & (points[:,2] <= self.altitude+upper)]

    def shift_grid_indices(self, pads):
        low, high, left, right = pads
        if not any(pads):
            return
        if self.target is not None:
            self.target = (self.target[0]+low, self.target[1]+left)
        self.visited = [((c[0]+low,c[1]+left),t) for c,t in self.visited]
        self.inspection_visited = [((c[0]+low,c[1]+left),t) for c,t in getattr(self,'inspection_visited',[])]
        self.blacklist = [((c[0]+low,c[1]+left),t) for c,t in self.blacklist]
        self.failed_visits = {(c[0]+low,c[1]+left):n for c,n in self.failed_visits.items()}
        self.inspected = np.pad(self.inspected, ((low,high),(left,right)), constant_values=False)
        self.inspection_plan = None

    def update_observed_window(self, position, free_points):
        """Grow the local planning window only from pose and fresh lidar freespace."""
        growth_points = np.vstack((np.asarray(position,dtype=float).reshape(1,2),
                                  free_points[:,:2]))
        pads = self.grid.grow_to_include(growth_points,
                    self.p('growth_margin',4.0),self.p('growth_chunk',8.0))
        if pads is None:
            return False
        self.shift_grid_indices(pads)
        if len(free_points):
            self.grid.observe_free(free_points)
        return True

    def publish(self, state, position=None, path=None, candidates=None, recovery=None):
        now = rospy.Time.now()
        route = RosPath()
        route.header.frame_id,route.header.stamp = 'map',now
        if position is not None:
            waypoint = self.grid.waypoint(path,position,self.p('lookahead',.6)) if path else position
            if recovery is not None:
                waypoint = recovery
                self.recovery_pub.publish(self.pose(recovery))
            self.waypoint_pub.publish(self.pose(waypoint))
            route.poses = [self.pose(self.grid.xy(c)) for c in path] if path else [self.pose(position)]
            if recovery is not None:
                route.poses.append(self.pose(recovery))
        self.path_pub.publish(route)
        visualize = (time.monotonic()-self.last_visualization >= 1./self.p('visualization_hz',.5) or
                     (state in ('COMPLETE','RETURN_FAILED') and state!=self.last_written_state))
        if visualize:
            cells = np.full(self.grid.shape,-1,dtype=np.int8)
            cells[self.grid.known] = 0
            cells[self.grid.occupied] = 100
            grid_msg = OccupancyGrid()
            grid_msg.header = route.header
            grid_msg.info.resolution = self.grid.resolution
            grid_msg.info.height,grid_msg.info.width = self.grid.shape
            grid_msg.info.origin.position.x,grid_msg.info.origin.position.y = self.grid.x0,self.grid.y0
            grid_msg.info.origin.orientation.w = 1.
            grid_msg.data = cells.ravel().tolist()
            self.grid_pub.publish(grid_msg)
        self.report = dict(state=state,stamp=now.to_sec(),wall_seconds=time.monotonic()-self.start_wall,
                           known_free_area_m2=float((self.grid.known & ~self.grid.occupied).sum()*self.grid.resolution**2),
                           known_cells=int(self.grid.known.sum()),map_points=len(self.cache.get('map',(None,[]))[1]),
                           goals_selected=self.goals_selected,goals_reached=self.goals_reached,
                           stalled_goals=self.failures,travel_xy_m=self.travel,
                           mission_phase=self.phase,finish_reason=self.finish_reason,
                           home_map_xy=self.home_xy,home_confirmed=self.home_confirmed,
                           home_error_xy_m=math.dist(position,self.home_xy) if position is not None and self.home_xy is not None else None,
                           inspection_cells=int(self.inspected.sum()),
                           coverage_complete=False,
                           exploration_complete=self.exploration_complete,
                           grid_capacity_limited=self.grid_capacity_limited,
                           completion_scope='observed reachable fixed-altitude space; NOT all 3D surfaces',
                           raw_frontier_candidates=self.raw_frontier_count,
                           useful_frontier_candidates=self.useful_frontier_count,
                           remaining_inspection_candidates=self.remaining_inspection_count,
                           surface_quality=self.surface_report,progress=self.progress_report,
                           compute_ms=self.last_compute_ms,
                           compute_p95_ms=float(np.percentile(self.tick_times,95)) if self.tick_times else 0.,
                           search_backend='native' if SEARCH is not None else 'python',
                           mission_sim_seconds=now.to_sec()-self.mission_started if self.mission_started is not None else 0.,
                           estimated_rtf=(now.to_sec()-self.active_sim)/(time.monotonic()-self.active_wall)
                               if self.active_wall is not None and time.monotonic()>self.active_wall else None,
                           target_map_xy=self.grid.xy(self.target) if self.target else None,
                           frontier_candidates=int(candidates.sum()) if candidates is not None else 0,
                           note='Observed 2D area, not 3D completeness. No backend. Ghost removal needs reobservation.')
        self.status_pub.publish(json.dumps(self.report))
        if not visualize:
            return  # waypoint/status stay full rate, only heavy visual layers slow
        self.last_visualization = time.monotonic()
        markers = MarkerArray()
        text = Marker(header=route.header,ns='exploration',id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
        text.pose.orientation.w = 1.
        if position is not None:
            text.pose.position.x,text.pose.position.y = position
        text.pose.position.z = (self.altitude or 1.2)+1.5
        text.scale.z = .4
        text.color.r,text.color.g,text.color.a = 1.,1.,1.
        text.text = '%s | %.1f m2 | goals %d | useful %d | surface %s' % (
            state,self.report['known_free_area_m2'],self.goals_reached,self.useful_frontier_count,
            ('%.0f%% observed' % (100*self.surface_report['observed_surface_quality_ratio']))
            if self.surface_report.get('observed_surface_quality_ratio') is not None else 'N/A')
        markers.markers.append(text)
        frontier = Marker(header=route.header,ns='exploration',id=1,type=Marker.POINTS,action=Marker.ADD)
        frontier.pose.orientation.w = 1.
        frontier.scale.x = frontier.scale.y = .15
        frontier.color.b = frontier.color.g = frontier.color.a = 1.
        if candidates is not None:
            for c in np.argwhere(candidates)[::3]:
                x,y = self.grid.xy(c)
                frontier.points.append(Point(x=x,y=y,z=self.altitude))
        markers.markers.append(frontier)
        self.markers_pub.publish(markers)
        self.report_file.parent.mkdir(parents=True,exist_ok=True)
        tmp = self.report_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.report,indent=2),encoding='utf8')
        os.replace(str(tmp),str(self.report_file))
        self.last_written_state = state
        rospy.loginfo_throttle(10,'Exploration %s area=%.1f m2 reached=%d target=%s',state,
                              self.report['known_free_area_m2'],self.goals_reached,self.report['target_map_xy'])

    def tick(self):
        wall, now = time.monotonic(),rospy.Time.now().to_sec()
        with self.lock:
            inputs = self.inputs.copy()
            takeoff_state = getattr(self,'takeoff_state',None)
        if any(name not in inputs or wall-inputs[name][1] > self.p('input_timeout_wall',3.)
               for name in ('odom','map','scan','free','mavros')):
            # Stop publishing waypoints: existing controller watchdog holds.
            self.home_since = None
            self.progress.reset()
            self.publish('WAIT_FRESH_DATA')
            return
        # A publisher's timer can keep receiving fresh while its source scan
        # has stopped. Check acquisition stamps, not receipt time alone.
        if any(now-inputs[name][0].header.stamp.to_sec()>self.p('input_timeout_sim',1.5)
               or inputs[name][0].header.stamp.to_sec()>now+.1
               for name in ('odom','map','scan','free')):
            self.home_since = None
            self.progress.reset()
            self.publish('WAIT_FRESH_DATA')
            return
        odom = inputs['odom'][0]
        p = odom.pose.pose.position
        position = (p.x,p.y)
        if self.home_xy is None:
            self.home_xy = position
        # Default is fail-closed. Only an explicit synthetic fixture may turn
        # this off; a first MAVROS sample is not a valid ground reference.
        if self.p('require_takeoff_confirmation',True) and not (
                takeoff_state and takeoff_state[0] and wall-takeoff_state[1]<=3.):
            self.home_since = self.ready_since = None
            self.progress.reset()
            self.publish('WAIT_TAKEOFF_CONFIRMATION')
            return
        if self.altitude is None:
            # The controller already checks armed/OFFBOARD, airborne and
            # continuous altitude dwell. Use the CURRENT map altitude.
            self.altitude = p.z
            self.settle_until = now+4.
            self.publish('OBSERVING',position)
            return
        if self.previous is not None:
            self.travel += math.dist(position,self.previous)
        self.previous = position
        free_msg = inputs['free'][0]
        free_points = np.empty((0,3),dtype=float)
        if free_msg is not self.last_free:
            # Only near-flight-height rays establish navigable space, not
            # floor/roof rays which could pass under/over shelving.
            free_points = self.slice(self.cloud('free',inputs),.2,.2)
            self.last_free = free_msg
        # The bounded grid is only a memory/computation window, not a known
        # warehouse outline. Expand it using this vehicle pose and fresh lidar
        # free-space evidence only; never use Gazebo collision boxes or map
        # extrema as a prior. Expansion leaves new cells unknown.
        if not self.update_observed_window(position,free_points):
            self.home_since = None
            self.grid_capacity_limited = True
            if self.phase not in ('RETURNING','COMPLETE','RETURN_FAILED'):
                self.begin_return('map_capacity_partial',now)
            # Retain old grid atomically and use only in-window fresh evidence.
            # Attempt a SAFE return; outside unknown cells are still forbidden.
            self.grid.observe_free(free_points)
            rospy.logerr_throttle(5,'Exploration grid capacity exhausted; attempting safe return, PARTIAL')
        points = np.concatenate([self.slice(self.cloud('map',inputs)),self.slice(self.cloud('scan',inputs))])
        self.grid.update_obstacles(points)
        if self.active_wall is None:
            self.active_wall,self.active_sim = wall,now
        def surface_slice(cloud):
            return cloud[(cloud[:,2]>=self.altitude-self.p('surface_height_below',.95)) &
                         (cloud[:,2]<=self.altitude+self.p('surface_height_above',2.))]
        self.surface.observe(surface_slice(self.cloud('scan',inputs)),(p.x,p.y,p.z),
            inputs['scan'][0].header.stamp.to_sec(),self.p('surface_observation_range',6.))
        deficits,self.surface_report = self.surface.deficits(surface_slice(self.cloud('map',inputs)))
        start = self.grid.cell(position)
        if self.mission_started is None:
            self.mission_started = now
        if self.phase in ('COMPLETE','RETURN_FAILED','STALLED_UNSAFE'):
            self.publish(self.phase,position)
            return
        if self.phase != 'RETURNING':
            limit = self.p('max_mission_seconds',3600.)
            goal_limit = self.p('return_after_goals',0)
            if self.return_requested:
                self.begin_return('operator_request',now)
            elif goal_limit>0 and self.goals_reached>=goal_limit:
                self.begin_return('explicit_goal_limit_partial',now)
            elif limit>0 and now-self.mission_started>=limit:
                self.begin_return('mission_budget_partial',now)
            elif self.p('max_mission_wall_seconds',3600.)>0 and wall-self.active_wall>=self.p('max_mission_wall_seconds',3600.):
                self.begin_return('wall_budget_partial',now)
        elif now-self.return_started>self.p('return_timeout',1200.):
            self.phase='RETURN_FAILED'
            self.publish('RETURN_FAILED',position)
            return
        if self.paused or now < self.settle_until:
            if self.paused:
                self.progress.reset()  # paused time cannot certify convergence
            self.home_since = None
            self.last_progress,self.progress_position = now,position
            self.publish('PAUSED' if self.paused else 'OBSERVING',position)
            return
        if not self.grid.inside(start) or not self.grid.safe[start]:
            self.progress.reset()
            self.home_since = None
            if self.unsafe_since is None or (self.unsafe_position is not None and
                    math.dist(position,self.unsafe_position) >= .20):
                self.unsafe_since,self.unsafe_position = now,position
            # A permanently unsafe start is an explicit partial-map failure,
            # not an endless hover disguised as ongoing exploration.
            if now-self.unsafe_since > self.p('unsafe_stall_seconds',30.):
                self.phase='STALLED_UNSAFE'
                self.finish_reason='unsafe_start_no_progress'
                self.publish('STALLED_UNSAFE',position)
                return
            recovery = self.grid.reconnect(position,points,self.p('recovery_minimum_clearance',.45),
                                           command_horizon=self.p('recovery_command_horizon',.20))
            if recovery is not None:
                self.last_progress,self.progress_position = now,position
            self.publish('RECOVER_CLEARANCE' if recovery is not None else 'HOLD_START_NOT_SAFE',
                         position,recovery=recovery)
            return
        self.unsafe_since = self.unsafe_position = None
        distances,parent = self.grid.search(start,clearance_weight=self.p('clearance_cost_weight',1.5))
        if self.phase == 'RETURNING':
            self.return_tick(now,position,start,distances,parent)
            return
        self.progress.update(now,int(self.grid.known.sum()),len(self.surface.ever),self.goals_reached)
        self.progress_report = self.progress.summary(self.grid.resolution)
        candidates,gain = self.grid.frontiers(distances)
        self.raw_frontier_count = int(candidates.sum())
        self.blacklist = [(c,t) for c,t in self.blacklist if t > now]
        for c,t in self.blacklist:
            suppress_disk(candidates,c,1.5/self.grid.resolution)
        # Mission-long memory prevents the old 240-second revisit loop. The
        # separate close-range inspection pass covers already-known aisles.
        for c,t in self.visited:
            suppress_disk(candidates,c,1.8/self.grid.resolution)
        for c,count in self.failed_visits.items():
            if count >= self.p('max_target_attempts',2):
                suppress_disk(candidates,c,1.5/self.grid.resolution)
        cells = representative_cells(candidates,gain,distances,self.grid.resolution)
        visible_gain = visible_unknown_gain(self.grid,cells,self.p('frontier_sensor_range',6.))
        candidates[:] = False
        gain = np.zeros(self.grid.shape,dtype=float)
        useful = cells[visible_gain>=self.p('frontier_min_gain_area',.5)]
        if len(useful):
            candidates[tuple(useful.T)] = True
            gain[tuple(useful.T)] = visible_gain[visible_gain>=self.p('frontier_min_gain_area',.5)]
        self.useful_frontier_count = int(candidates.sum())
        # Low gain does not equal full coverage. It moves to surface inspection
        # and retains the residual frontier count in the report.
        if (self.phase=='EXPLORING' and self.progress_report['low_gain'] and
                self.progress_report['observation_goals']>=self.p('convergence_min_goals',3)):
            self.phase='INSPECTING'
            self.inspection_started=now
            self.target=None
            self.no_frontier_since=None
            self.inspection_plan=None
        self.remaining_inspection_count = 0
        if self.phase == 'INSPECTING':
            frontier_candidates,frontier_gain = candidates,gain
            # Plans are coarse observed-surface views; safety/reachability is
            # always rechecked at full tick rate, including new moving people.
            if self.inspection_plan is None or now-self.inspection_last_plan>=self.p('inspection_plan_seconds',3.):
                self.inspection_plan = self.surface.inspection_candidates(self.grid,distances,deficits,
                    limit=self.p('inspection_max_surface_targets',60),
                    range_m=self.p('inspection_surface_range',3.))
                self.inspection_last_plan = now
            candidates,gain = (v.copy() for v in self.inspection_plan)
            candidates &= self.grid.safe & np.isfinite(distances)
            for c,count in self.failed_visits.items():
                if count >= self.p('max_target_attempts',2):
                    suppress_disk(candidates,c,1.5/self.grid.resolution)
            for c,t in self.blacklist:
                suppress_disk(candidates,c,1.5/self.grid.resolution)
            # An arrival is NOT proof that the surface gained a second view.
            # Revisit a still-deficient safe view after a cooldown instead of
            # permanently hiding holes as the frontier visit memory does.
            self.inspection_visited=[(c,t) for c,t in self.inspection_visited
                                     if now-t<self.p('inspection_revisit_seconds',20.)]
            for c,t in self.inspection_visited:
                suppress_disk(candidates,c,self.p('inspection_visit_spacing',.8)/self.grid.resolution)
            self.remaining_inspection_count=int(candidates.sum())
            if not self.remaining_inspection_count and self.useful_frontier_count:
                # Inspection can reveal a new opening (e.g. a person leaves).
                # Do not abandon that frontier just because we entered the
                # inspection phase earlier. Start a NEW convergence window;
                # old low-gain evidence must not immediately flip us back.
                self.phase='EXPLORING'
                self.target=None
                self.no_frontier_since=None
                self.progress.reset()
                candidates,gain=frontier_candidates,frontier_gain
                self.publish('RESUME_EXPLORATION',position,candidates=candidates)
                return
            quality=self.surface_report.get('observed_surface_quality_ratio')
            qualified=(quality is not None and quality>=self.p('surface_quality_target',.95) and
                       not self.surface.capacity_limited)
            if (self.progress_report['low_gain'] and qualified and not self.useful_frontier_count and
                    not self.remaining_inspection_count and not self.failed_visits and
                    not self.grid_capacity_limited):
                self.exploration_complete=True
                self.begin_return('observed_reachable_converged',now)
                self.publish('RETURNING',position)
                return
            if self.inspection_started is None:
                self.inspection_started=now
            if now-self.inspection_started>=self.p('inspection_budget_seconds',600.):
                self.begin_return('inspection_budget_partial',now)
                self.publish('RETURNING',position)
                return
        if self.target is not None:
            if math.dist(position,self.grid.xy(self.target)) <= self.p('arrival_tolerance',.3):
                self.goals_reached += 1
                self.visited.append((self.target,now))
                if self.phase=='INSPECTING':
                    self.inspection_visited.append((self.target,now))
                self.target = None
                self.settle_until = now+self.p('settle_seconds',3.)
                self.publish('OBSERVING',position,candidates=candidates)
                return
            if math.dist(position,self.progress_position) >= .25:
                self.progress_position,self.last_progress = position,now
            if not np.isfinite(distances[self.target]) or now-self.last_progress > self.p('progress_timeout',25.):
                self.failed_visits[self.target] = self.failed_visits.get(self.target,0)+1
                self.blacklist.append((self.target,now+self.p('blacklist_seconds',60.)))
                self.target = None
                self.failures += 1
                # Recompute suppression next tick before selecting another goal.
                self.publish('REPLAN_BLOCKED',position,candidates=candidates)
                return
        if self.target is None:
            options = np.argwhere(candidates)
            if len(options):
                # Reward unknown cells, penalize actual reachable path length.
                scores = [gain[tuple(c)]/(2.+distances[tuple(c)]) for c in options]
                self.target = tuple(options[int(np.argmax(scores))])
                self.goals_selected += 1
                self.last_progress,self.progress_position = now,position
                self.goal_pub.publish(self.pose(self.grid.xy(self.target)))
                self.no_frontier_since = None
            else:
                self.no_frontier_since = self.no_frontier_since or now
                state = 'WAIT_FRONTIER'
                if now-self.no_frontier_since > self.p('no_frontier_seconds',30.):
                    if self.phase == 'EXPLORING':
                        self.phase='INSPECTING'
                        self.inspection_started=now
                        self.inspection_plan=None
                        self.no_frontier_since=None
                        state='START_INSPECTION'
                    else:
                        if (qualified and not self.useful_frontier_count and
                                not self.failed_visits and not self.grid_capacity_limited and
                                not self.progress_report['low_gain']):
                            # Wait for the full evidence window instead of
                            # declaring completion after only a short pause.
                            self.publish('VERIFY_CONVERGENCE',position,candidates=candidates)
                            return
                        self.begin_return('reachable_viewpoints_exhausted',now)
                        state='RETURNING'
                self.publish(state,position,candidates=candidates)
                return
        path = self.grid.path(parent,start,self.target)
        self.publish(self.phase,position,path,candidates)

    def save(self):
        try:
            rospy.wait_for_service('/demo/save_map',timeout=1.)
            result = rospy.ServiceProxy('/demo/save_map',Trigger)()
            if not result.success:
                rospy.logwarn('Exploration map save: %s',result.message)
            return result.success
        except (rospy.ROSException,rospy.ServiceException) as exc:
            rospy.logwarn('Exploration map save unavailable: %s',exc)
            return False

    def run(self):
        while not rospy.is_shutdown():
            began = time.monotonic()
            self.tick()
            self.last_compute_ms = (time.monotonic()-began)*1000.
            self.tick_times.append(self.last_compute_ms)
            self.tick_times = self.tick_times[-120:]
            if began-self.last_save >= self.p('save_every_wall_seconds',60.):
                self.save()
                self.last_save = began
            # Wall time keeps diagnostics live when /clock pauses.
            time.sleep(max(.01,1./self.p('planning_hz',2.)-(time.monotonic()-began)))


if __name__ == '__main__':
    rospy.init_node('warehouse_explorer')
    Explorer().run()
