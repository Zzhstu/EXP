#!/usr/bin/env python3
"""Sensor-only progress/observed-surface evidence, not ground-truth coverage.

Missing surfaces which have NEVER returned a ray cannot enter this denominator.
Fixed-altitude navigation cannot certify roof/interior/occluded mesh completeness.
"""
from collections import deque
import math
import numpy as np


class ProgressWindow:
    def __init__(self, seconds=90., free_threshold=1., surface_threshold=25):
        self.seconds = seconds
        self.free_threshold = free_threshold
        self.surface_threshold = surface_threshold
        self.samples = deque()

    def update(self, now, known_count, surface_count, goals):
        # Inputs are monotonic evidence counters, not noisy raw map point count.
        self.samples.append((now,known_count,surface_count,goals))
        while len(self.samples)>1 and self.samples[1][0] <= now-self.seconds:
            self.samples.popleft()

    def reset(self):
        self.samples.clear()

    def summary(self, resolution):
        if not self.samples:
            return dict(window_ready=False,new_free_area_m2=0.,new_surface_voxels=0,
                        observation_goals=0,low_gain=False)
        first,last = self.samples[0],self.samples[-1]
        area = max(0,last[1]-first[1])*resolution**2
        surfaces = max(0,last[2]-first[2])
        ready = last[0]-first[0] >= self.seconds
        return dict(window_ready=ready,new_free_area_m2=area,new_surface_voxels=surfaces,
                    observation_goals=max(0,last[3]-first[3]),
                    low_gain=bool(ready and area<self.free_threshold and surfaces<self.surface_threshold))


def suppress_disk(mask, cell, radius):
    """Visit-memory suppression in a small patch, not a whole-grid allocation."""
    n = int(math.ceil(radius))
    y,x = cell
    y0,y1 = max(0,y-n),min(mask.shape[0],y+n+1)
    x0,x1 = max(0,x-n),min(mask.shape[1],x+n+1)
    yy,xx = np.ogrid[y0:y1,x0:x1]
    patch = mask[y0:y1,x0:x1]
    patch[(yy-y)**2+(xx-x)**2 < radius**2] = False


def representative_cells(mask, gain, distances, resolution, spacing=1.):
    """One best representative per metre bin; avoid scoring every cyan pixel."""
    cells = np.argwhere(mask)
    if not len(cells):
        return cells
    n = max(1,int(round(spacing/resolution)))
    bins = cells//n
    rank = gain[mask]/(2.+distances[mask])
    order = np.argsort(-rank,kind='stable')
    _,indexes = np.unique(bins[order],axis=0,return_index=True)
    return cells[order[np.sort(indexes)]]


def visible_unknown_gain(grid, cells, sensor_range=6., rays=96):
    """Approximate horizontal visibility, stopped by RAW occupied cells.

    Gain may enter unknown but never passes a measured wall. Densely sample
    each ray to prevent skipping a one-cell wall. This is a scoring heuristic,
    NOT a traversability test or 3-D visibility certificate.
    """
    if not len(cells):
        return np.empty(0)
    angles = np.arange(rays)*2*math.pi/rays
    steps = np.arange(grid.resolution*.5,sensor_range+1e-6,grid.resolution*.5)
    offsets = np.stack((np.sin(angles)[:,None]*steps,
                        np.cos(angles)[:,None]*steps),axis=-1)/grid.resolution
    offsets = np.floor(offsets+.5).astype(np.int32)
    gain = []
    for cell in cells:
        points = offsets+cell
        y,x = points[...,0],points[...,1]
        valid = (y>=0)&(y<grid.shape[0])&(x>=0)&(x<grid.shape[1])
        yc,xc = np.clip(y,0,grid.shape[0]-1),np.clip(x,0,grid.shape[1]-1)
        blocked = ~valid | grid.occupied[yc,xc]
        visible = (np.cumsum(blocked,axis=1)==0)&~grid.known[yc,xc]
        indexes = yc[visible]*grid.shape[1]+xc[visible]
        gain.append(len(np.unique(indexes))*grid.resolution**2)
    return np.asarray(gain)


def raw_line_visible(grid, a, b, endpoint_margin=0.):
    """Only interior cells block a ray; its endpoint may be a surface."""
    count = max(2,int(math.ceil(math.dist(a,b)/grid.resolution*4)))
    t = np.linspace(0.,1.,count,endpoint=False)
    if endpoint_margin:
        t = t[(1.-t)*math.dist(a,b)>endpoint_margin]
    x = np.floor((a[0]+t*(b[0]-a[0])-grid.x0)/grid.resolution).astype(np.int32)
    y = np.floor((a[1]+t*(b[1]-a[1])-grid.y0)/grid.resolution).astype(np.int32)
    end = grid.cell(b)
    interior = (y!=end[0]) | (x!=end[1])
    y,x = y[interior],x[interior]
    if ((y<0)|(y>=grid.shape[0])|(x<0)|(x>=grid.shape[1])).any():
        return False
    return not grid.occupied[y,x].any()


class SurfaceEvidence:
    def __init__(self, voxel=.4, minimum_hits=3, minimum_views=2,
                 baseline=.75, maximum=150000):
        self.voxel,self.minimum_hits,self.minimum_views = voxel,minimum_hits,minimum_views
        self.baseline,self.maximum = baseline,maximum
        self.records = {}
        self.ever = set()
        self.capacity_limited = False
        self.last_stamp = None

    def keys(self, points):
        if not len(points):
            return np.empty((0,3),dtype=np.int64)
        return np.unique(np.floor(points/self.voxel).astype(np.int64),axis=0)

    def observe(self, points, observer, stamp, max_range=6.):
        # Re-publishing the same scan does not add evidence. Out-of-order scans
        # cannot turn a parked scanner into independent observations either.
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        points = np.asarray(points)
        observer = np.asarray(observer)
        points = points[np.linalg.norm(points-observer,axis=1)<=max_range]
        for k in self.keys(points):
            key = tuple(k)
            center = (k+.5)*self.voxel
            angle = math.atan2(observer[1]-center[1],observer[0]-center[0])
            bit = 1 << (int((angle+math.pi)/(2*math.pi)*8)%8)
            record = self.records.get(key)
            if record is None:
                if len(self.records)>=self.maximum or len(self.ever)>=self.maximum:
                    self.capacity_limited = True
                    continue
                record = [0,bit,observer.copy()]
                self.records[key] = record
                self.ever.add(key)
            record[0] += 1
            if np.linalg.norm(observer-record[2])>=self.baseline:
                record[1] |= bit

    def complete(self, key):
        record = self.records.get(tuple(key))
        return bool(record and record[0]>=self.minimum_hits and
                    bin(record[1]).count('1')>=self.minimum_views)

    def deficits(self, current_map):
        # Restrict to the CURRENT maintained map. Cleared people must not
        # survive as permanent inspection targets in this evidence sidecar.
        keys = self.keys(current_map)
        done = np.asarray([self.complete(k) for k in keys],dtype=bool)
        return (keys[~done]+.5)*self.voxel,dict(
            observed_surface_voxels=len(keys),qualified_surface_voxels=int(done.sum()),
            observed_surface_quality_ratio=float(done.mean()) if len(keys) else None,
            surface_evidence_capacity_limited=self.capacity_limited)

    def inspection_candidates(self, grid, distances, deficits, minimum_distance=1.2,
                              limit=180, range_m=3.):
        """Observe deficient measured surfaces from another safe aisle view.

        Bounded coarse representatives; candidates do NOT contain truth shelf
        locations. Surface ray is checked against measured occupancy only.
        """
        mask = np.zeros(grid.shape,dtype=bool)
        gain = np.zeros(grid.shape,dtype=float)
        if not len(deficits):
            return mask,gain
        xy_cells = np.asarray([grid.cell(p[:2]) for p in deficits])
        _,index = np.unique(xy_cells//max(1,int(.8/grid.resolution)),axis=0,return_index=True)
        # Bounded sampling may leave deficits. It never authorizes 'complete'.
        index = index[::max(1,int(math.ceil(len(index)/limit)))]
        for surface in deficits[index]:
            # Sector centres, not boundaries: grid quantization and the .30m
            # arrival tolerance must not turn a proposed "new" direction back
            # into the old sector. Score the ACTUAL rasterized waypoint below.
            for angle in (np.arange(8)+.5)*2*math.pi/8:
                for radius in (1.5,2.5,range_m):
                    xy = surface[:2]+radius*np.asarray([math.cos(angle),math.sin(angle)])
                    cell = grid.cell(xy)
                    if not grid.inside(cell) or not grid.safe[cell] or not np.isfinite(distances[cell]):
                        continue
                    # A coarse voxel centre can lie just behind the actual
                    # first return. Only scoring uses a quantization margin;
                    # path/footprint safety ALWAYS keeps full obstacle inflation.
                    if distances[cell]<minimum_distance or not raw_line_visible(
                            grid,grid.xy(cell),surface[:2],self.voxel*math.sqrt(2)/2):
                        continue
                    record = self.records.get(tuple(np.floor(surface/self.voxel).astype(np.int64)))
                    view_xy=grid.xy(cell)
                    direction = math.atan2(view_xy[1]-surface[1],view_xy[0]-surface[0])
                    bit = 1 << (int((direction+math.pi)/(2*math.pi)*8)%8)
                    if record and record[0]>=self.minimum_hits and record[1]&bit:
                        continue  # another direction, not another identical visit
                    mask[cell] = True
                    gain[cell] += 1
        return mask,gain
