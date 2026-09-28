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
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
# catkin devel relays execute into a separate context and cannot be imported as
# library modules. Prefer this actual source/install directory over relay dir.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from exploration_grid import ExplorationGrid, expand


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
        self.grid = ExplorationGrid(self.p('bounds', [-4,-12,47,15]),
                                    self.p('resolution',.2),self.p('clearance',.8),
                                    self.p('unknown_margin',.4))
        self.lock = threading.Lock()
        self.inputs = {}
        self.paused = False
        self.target = None
        self.blacklist = []
        self.visited = []
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
        self.last_save = time.monotonic()
        self.last_free = None
        self.cache = {}
        self.goals_selected = self.goals_reached = self.failures = 0
        self.start_wall = time.monotonic()
        self.travel = 0.
        self.previous = None
        self.report_file = Path(self.p('report','/tmp/warehouse_exploration.json'))
        for name, topic, cls in (
            ('odom','/uav1/fastlio/odom',Odometry),
            ('mavros','/mavros/local_position/pose',PoseStamped),
            ('map','/uav1/fastlio/cloud_map',PointCloud2),
            ('scan','/uav1/fastlio/registered_scan',PointCloud2),
            ('free','/uav1/local_free_space',PointCloud2)):
            rospy.Subscriber(topic,cls,self.receive,name,queue_size=1,buff_size=16000000)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
        self.waypoint_pub = rospy.Publisher('/uav1/planner/waypoint',PoseStamped,queue_size=1)
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

    def publish(self, state, position=None, path=None, candidates=None, recovery=None):
        now = rospy.Time.now()
        route = RosPath()
        route.header.frame_id,route.header.stamp = 'map',now
        if position is not None:
            waypoint = self.grid.waypoint(path,position,self.p('lookahead',.6)) if path else position
            if recovery is not None:
                waypoint = recovery
            self.waypoint_pub.publish(self.pose(waypoint))
            route.poses = [self.pose(self.grid.xy(c)) for c in path] if path else [self.pose(position)]
            if recovery is not None:
                route.poses.append(self.pose(recovery))
        self.path_pub.publish(route)
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
                           target_map_xy=self.grid.xy(self.target) if self.target else None,
                           frontier_candidates=int(candidates.sum()) if candidates is not None else 0,
                           note='Observed 2D area, not 3D completeness. No backend. Ghost removal needs reobservation.')
        self.status_pub.publish(json.dumps(self.report))
        markers = MarkerArray()
        text = Marker(header=route.header,ns='exploration',id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
        text.pose.orientation.w = 1.
        if position is not None:
            text.pose.position.x,text.pose.position.y = position
        text.pose.position.z = (self.altitude or 1.2)+1.5
        text.scale.z = .4
        text.color.r,text.color.g,text.color.a = 1.,1.,1.
        text.text = '%s | %.1f m2 | goals %d' % (state,self.report['known_free_area_m2'],self.goals_reached)
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
        rospy.loginfo_throttle(10,'Exploration %s area=%.1f m2 reached=%d target=%s',state,
                              self.report['known_free_area_m2'],self.goals_reached,self.report['target_map_xy'])

    def tick(self):
        wall, now = time.monotonic(),rospy.Time.now().to_sec()
        with self.lock:
            inputs = self.inputs.copy()
        if 'mavros' in inputs and self.home is None:
            self.home = inputs['mavros'][0].pose.position.z
        if any(name not in inputs or wall-inputs[name][1] > self.p('input_timeout_wall',3.)
               for name in ('odom','map','scan','free','mavros')):
            # Stop publishing waypoints: existing controller watchdog holds.
            self.home_since = None
            self.publish('WAIT_FRESH_DATA')
            return
        # A publisher's timer can keep receiving fresh while its source scan
        # has stopped. Check acquisition stamps, not receipt time alone.
        if any(now-inputs[name][0].header.stamp.to_sec()>self.p('input_timeout_sim',1.5)
               or inputs[name][0].header.stamp.to_sec()>now+.1
               for name in ('odom','map','scan','free')):
            self.home_since = None
            self.publish('WAIT_FRESH_DATA')
            return
        odom = inputs['odom'][0]
        p = odom.pose.pose.position
        position = (p.x,p.y)
        if self.home_xy is None:
            self.home_xy = position
        if self.altitude is None:
            height = inputs['mavros'][0].pose.position.z-self.home
            if height >= self.p('takeoff_relative_height',1.2)-.10:
                self.ready_since = self.ready_since or now
                if now-self.ready_since >= 1.:
                    self.altitude = p.z + self.p('takeoff_relative_height',1.2)-height
                    self.settle_until = now+4.
            else:
                self.ready_since = None
            self.publish('TAKEOFF')
            return
        if self.previous is not None:
            self.travel += math.dist(position,self.previous)
        self.previous = position
        free_msg = inputs['free'][0]
        if free_msg is not self.last_free:
            # Only near-flight-height rays establish navigable space, not
            # floor/roof rays which could pass under/over shelving.
            self.grid.observe_free(self.slice(self.cloud('free',inputs),.2,.2))
            self.last_free = free_msg
        points = np.concatenate([self.slice(self.cloud('map',inputs)),self.slice(self.cloud('scan',inputs))])
        self.grid.update_obstacles(points)
        start = self.grid.cell(position)
        if self.mission_started is None:
            self.mission_started = now
        if self.phase in ('COMPLETE','RETURN_FAILED'):
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
        elif now-self.return_started>self.p('return_timeout',1200.):
            self.phase='RETURN_FAILED'
            self.publish('RETURN_FAILED',position)
            return
        if self.paused or now < self.settle_until:
            self.home_since = None
            self.last_progress,self.progress_position = now,position
            self.publish('PAUSED' if self.paused else 'OBSERVING',position)
            return
        if not self.grid.inside(start) or not self.grid.safe[start]:
            self.home_since = None
            recovery = self.grid.reconnect(position,points,self.p('recovery_minimum_clearance',.70))
            if recovery is not None:
                self.last_progress,self.progress_position = now,position
            self.publish('RECOVER_CLEARANCE' if recovery is not None else 'HOLD_START_NOT_SAFE',
                         position,recovery=recovery)
            return
        distances,parent = self.grid.search(start)
        if self.phase == 'RETURNING':
            self.return_tick(now,position,start,distances,parent)
            return
        self.mark_inspected(position)
        candidates,gain = self.grid.frontiers(distances)
        self.blacklist = [(c,t) for c,t in self.blacklist if t > now]
        for c,t in self.blacklist:
            y,x = c
            yy,xx = np.ogrid[:self.grid.shape[0],:self.grid.shape[1]]
            candidates[(yy-y)**2+(xx-x)**2 < (1.5/self.grid.resolution)**2] = False
        # Mission-long memory prevents the old 240-second revisit loop. The
        # separate close-range inspection pass covers already-known aisles.
        for c,t in self.visited:
            y,x = c
            yy,xx = np.ogrid[:self.grid.shape[0],:self.grid.shape[1]]
            candidates[(yy-y)**2+(xx-x)**2 < (1.8/self.grid.resolution)**2] = False
        for c,count in self.failed_visits.items():
            if count >= self.p('max_target_attempts',2):
                y,x = c
                yy,xx = np.ogrid[:self.grid.shape[0],:self.grid.shape[1]]
                candidates[(yy-y)**2+(xx-x)**2 < (1.5/self.grid.resolution)**2] = False
        if self.phase == 'INSPECTING':
            # Revisit reachable known space near measured obstacle surfaces,
            # rather than declaring "known floor" equal to "rack scanned".
            candidates = (self.grid.safe & np.isfinite(distances) & (distances>1.2) &
                          ~self.inspected & expand(self.grid.occupied,
                          self.p('inspection_surface_range',3.0)/self.grid.resolution))
            gain = np.ones(self.grid.shape)
            for c,count in self.failed_visits.items():
                if count >= self.p('max_target_attempts',2):
                    yy,xx = np.ogrid[:self.grid.shape[0],:self.grid.shape[1]]
                    candidates[(yy-c[0])**2+(xx-c[1])**2<(1.5/self.grid.resolution)**2]=False
            for c,t in self.blacklist:
                yy,xx = np.ogrid[:self.grid.shape[0],:self.grid.shape[1]]
                candidates[(yy-c[0])**2+(xx-c[1])**2<(1.5/self.grid.resolution)**2]=False
        if self.target is not None:
            if math.dist(position,self.grid.xy(self.target)) <= self.p('arrival_tolerance',.3):
                self.goals_reached += 1
                self.visited.append((self.target,now))
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
                        self.no_frontier_since=None
                        state='START_INSPECTION'
                    else:
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
            if began-self.last_save >= self.p('save_every_wall_seconds',60.):
                self.save()
                self.last_save = began
            # Wall time keeps diagnostics live when /clock pauses.
            time.sleep(max(.01,1./self.p('planning_hz',2.)-(time.monotonic()-began)))


if __name__ == '__main__':
    rospy.init_node('warehouse_explorer')
    Explorer().run()
