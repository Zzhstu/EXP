#!/usr/bin/env python3
"""Small bounded 2-D frontier planner. No ROS, scene geometry or ground truth.

Free evidence is accumulated; occupied cells are rebuilt from the maintained
map plus CURRENT returns, so a cleared person does not become a permanent wall.
This is a fixed-height projection, not a proof of full 3-D collision freedom.
"""
import heapq
import math
import numpy as np
from exploration_native import search as native_search


def expand(mask, radius, outside=False):
    """Disk dilation without scipy; no wraparound at map boundaries."""
    n = int(math.ceil(radius))
    padded = np.pad(mask, n, constant_values=outside)
    out = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if dx * dx + dy * dy <= radius * radius:
                out |= padded[n+dy:n+dy+h, n+dx:n+dx+w]
    return out


class ExplorationGrid:
    def __init__(self, bounds, resolution=.2, clearance=.8, unknown_margin=.4,
                 max_cells=200000):
        self.x0, self.y0, self.x1, self.y1 = map(float, bounds)
        self.resolution = float(resolution)
        if (not all(math.isfinite(v) for v in bounds) or resolution < .1 or
                self.x1 <= self.x0 or self.y1 <= self.y0 or clearance < .4 or
                unknown_margin < .4):
            raise ValueError('Invalid bounds/resolution/body clearance')
        self.shape = (int(math.ceil((self.y1-self.y0)/resolution)),
                      int(math.ceil((self.x1-self.x0)/resolution)))
        self.max_cells = int(max_cells)
        if self.max_cells < 1 or self.shape[0]*self.shape[1] > self.max_cells:
            raise ValueError('Exploration grid exceeds configured cell capacity')
        self.known = np.zeros(self.shape, dtype=bool)
        self.occupied = self.known.copy()
        self.safe = self.known.copy()
        self.body_known = self.known.copy()
        # Account for point-to-cell quantization in the same way as old A*.
        self.clearance = clearance / resolution + math.sqrt(2)/2
        self.unknown_margin = unknown_margin / resolution

    def grow_to_include(self, points, margin=4.0, chunk=8.0):
        """Grow the computational window around *observed* points.

        Returned padding is (y_low, y_high, x_low, x_high), suitable for
        np.pad. Existing cell/world coordinates are preserved. Nothing is
        marked known by growing; outside space remains unknown and therefore
        non-traversable. None means the finite cell budget would be exceeded.
        """
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return (0, 0, 0, 0)
        points = points.reshape((-1, points.shape[-1]))[:, :2]
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return (0, 0, 0, 0)
        if margin < 0 or chunk <= 0:
            raise ValueError('Growth margin/chunk must be nonnegative/positive')
        r = self.resolution
        chunk_cells = max(1, int(math.ceil(chunk/r)))

        def cells_needed(distance):
            needed = max(0, int(math.ceil(distance/r - 1e-9)))
            return int(math.ceil(needed/chunk_cells))*chunk_cells if needed else 0

        left = cells_needed(self.x0 + margin - float(points[:, 0].min()))
        right = cells_needed(float(points[:, 0].max()) - (self.x1 - margin))
        low = cells_needed(self.y0 + margin - float(points[:, 1].min()))
        high = cells_needed(float(points[:, 1].max()) - (self.y1 - margin))
        if not (left or right or low or high):
            return (0, 0, 0, 0)
        height, width = self.shape[0]+low+high, self.shape[1]+left+right
        if height*width > self.max_cells:
            # A coarse chunk is a performance optimization, not a reason to
            # reject an otherwise affordable one-cell boundary extension.
            left = max(0, int(math.ceil((self.x0+margin-float(points[:,0].min()))/r-1e-9))) if self.x0+margin > points[:,0].min() else 0
            right = max(0, int(math.ceil((float(points[:,0].max())-(self.x1-margin))/r-1e-9))) if points[:,0].max() > self.x1-margin else 0
            low = max(0, int(math.ceil((self.y0+margin-float(points[:,1].min()))/r-1e-9))) if self.y0+margin > points[:,1].min() else 0
            high = max(0, int(math.ceil((float(points[:,1].max())-(self.y1-margin))/r-1e-9))) if points[:,1].max() > self.y1-margin else 0
            height, width = self.shape[0]+low+high, self.shape[1]+left+right
            if height*width > self.max_cells:
                return None
        pads = (low, high, left, right)
        if not any(pads):
            return pads
        self.known = np.pad(self.known, ((low, high), (left, right)), constant_values=False)
        self.occupied = np.pad(self.occupied, ((low, high), (left, right)), constant_values=False)
        self.safe = np.pad(self.safe, ((low, high), (left, right)), constant_values=False)
        self.body_known = np.pad(self.body_known, ((low, high), (left, right)), constant_values=False)
        self.x0 -= left*r
        self.y0 -= low*r
        self.x1 = self.x0 + width*r
        self.y1 = self.y0 + height*r
        self.shape = (height, width)
        return pads

    def cell(self, xy):
        return (int(math.floor((xy[1]-self.y0)/self.resolution)),
                int(math.floor((xy[0]-self.x0)/self.resolution)))

    def inside(self, cell):
        y, x = cell
        return 0 <= y < self.shape[0] and 0 <= x < self.shape[1]

    def xy(self, cell):
        y, x = cell
        return (self.x0+(x+.5)*self.resolution, self.y0+(y+.5)*self.resolution)

    def raster(self, points):
        result = np.zeros(self.shape, dtype=bool)
        if len(points):
            points = np.asarray(points)
            points = points[np.isfinite(points[:, :2]).all(axis=1)]
            x = np.floor((points[:, 0]-self.x0)/self.resolution).astype(np.int64)
            y = np.floor((points[:, 1]-self.y0)/self.resolution).astype(np.int64)
            valid = (x >= 0) & (x < self.shape[1]) & (y >= 0) & (y < self.shape[0])
            result[y[valid], x[valid]] = True
        return result

    def observe_free(self, points):
        self.known |= self.raster(points)

    def update_obstacles(self, points):
        self.occupied = self.raster(points)
        # Never erase occupied cells with projected 2-D free observations.
        self.body_known = self.known & ~expand(~self.known, self.unknown_margin, outside=True)
        self.safe = self.body_known & ~expand(self.occupied, self.clearance)

    def reconnect(self, position, points, minimum_clearance=.45, max_distance=2., command_horizon=.10):
        """Recover when new returns invalidate the current inflated cell.

        NEVER erase inflation around the start. Only return a short segment to
        a normally safe cell if all sampled raw-point clearances are >= the
        start clearance (5mm numerical allowance) and >= the body radius plus
        5cm. Body footprint must remain observed. The controller separately
        validates each short command against a fresh scan, even on level 2.
        This is not a 3-D collision certificate.
        """
        start = self.cell(position)
        if (not math.isfinite(command_horizon) or not .03<=command_horizon<=.25 or
                not math.isfinite(minimum_clearance) or minimum_clearance < .45 or
                not math.isfinite(max_distance) or max_distance <= 0):
            raise ValueError('Recovery command horizon must be between .03 and .25 m')
        if not self.inside(start) or not self.body_known[start] or not len(points):
            return None
        points = np.asarray(points)[:,:2]
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return None
        position = np.asarray(position)
        initial = float(np.linalg.norm(points-position,axis=1).min())
        if initial < minimum_clearance:
            return None
        # No point farther than initial+2*max_distance can become the nearest
        # on a segment this short (triangle inequality). Bound per-sample work.
        points = points[np.linalg.norm(points-position,axis=1) <= initial+2*max_distance+.1]
        candidates = []
        n = int(math.ceil(max_distance/self.resolution))
        for y in range(max(0,start[0]-n),min(self.shape[0],start[0]+n+1)):
            for x in range(max(0,start[1]-n),min(self.shape[1],start[1]+n+1)):
                if self.safe[y,x]:
                    xy = np.asarray(self.xy((y,x)))
                    d = float(np.linalg.norm(xy-position))
                    if d <= max_distance:
                        candidates.append((d,y,x))
        # Prefer the closest normally-safe cell; every candidate still needs a
        # fully observed, strictly clearance-improving join from the REAL pose.
        for length,y,x in sorted(candidates):
            target = np.asarray(self.xy((y,x)))
            previous = initial
            valid = True
            for t in np.linspace(0,1,max(2,int(math.ceil(length/.04))+1)):
                point = position+t*(target-position)
                cell = self.cell(point)
                distance = float(np.linalg.norm(points-point,axis=1).min())
                if (not self.inside(cell) or not self.body_known[cell] or
                        distance < max(initial,previous)-.005 or distance < minimum_clearance):
                    valid = False
                    break
                previous = distance
            if valid and previous >= initial+.05:
                # Bounded command horizon keeps P-controller recovery slow; it is
                # revalidated every tick instead of blindly flying the join.
                point = position + min(1.,command_horizon/max(length,1e-6))*(target-position)
                return tuple(float(v) for v in point)
        return None

    def search(self, start, backend='auto', clearance_weight=0.):
        """Dijkstra with optional soft edge cost; same HARD safe/no-cut rules.

        With nonzero weight, distance means equivalent length cost, not actual
        path length. A narrow but safe corridor stays reachable, not excluded.
        """
        if backend not in ('auto','native','python'):
            raise ValueError('Unknown search backend')
        if not math.isfinite(clearance_weight) or not 0<=clearance_weight<=100:
            raise ValueError('Clearance cost weight must be finite in [0,100]')
        penalties = (expand(~self.safe,1.01,outside=True).astype(np.float32)*clearance_weight
                     if clearance_weight else None)
        if self.inside(start) and backend != 'python':
            result = native_search(self.safe,start,self.resolution,penalties)
            if result is not None:
                return result
            if backend == 'native':
                raise RuntimeError('Build astra_exploration_search first')
        distance = np.full(self.shape, np.inf)
        parent = {}
        if not self.inside(start) or not self.safe[start]:
            return distance, parent
        distance[start] = 0.
        queue = [(0., start)]
        while queue:
            cost, u = heapq.heappop(queue)
            if cost != distance[u]:
                continue
            y, x = u
            for dy, dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                v = y+dy, x+dx
                if not self.inside(v) or not self.safe[v]:
                    continue
                if dx and dy and (not self.safe[y+dy,x] or not self.safe[y,x+dx]):
                    continue
                new = cost + self.resolution * (math.sqrt(2) if dx and dy else 1.)*\
                    (1.+(float(penalties[v]) if penalties is not None else 0.))
                if new + 1e-9 < distance[v]:
                    distance[v] = new
                    parent[v] = u
                    heapq.heappush(queue, (new, v))
        return distance, parent

    def frontiers(self, distances, min_distance=1.2):
        unknown = ~self.known & ~self.occupied
        # A viewpoint stays behind the frontier with enough body clearance.
        boundary = self.known & ~self.occupied & expand(unknown, 1.01)
        candidates = (expand(boundary, 1.8/self.resolution) & self.safe &
                      np.isfinite(distances) & (distances >= min_distance))
        # Integral-image gain, used only to rank candidates, not certify sight.
        n = int(2.0/self.resolution)
        p = np.pad(unknown.astype(np.int32), n)
        summed = np.pad(p.cumsum(0).cumsum(1), ((1,0),(1,0)))
        k = 2*n+1
        gain = summed[k:,k:] - summed[:-k,k:] - summed[k:,:-k] + summed[:-k,:-k]
        return candidates, gain

    def path(self, parent, start, goal):
        result = [goal]
        while result[-1] != start:
            if isinstance(parent,np.ndarray):
                index = int(parent[result[-1]])
                if index < 0:
                    return []
                result.append(divmod(index,self.shape[1]))
            else:
                if result[-1] not in parent:
                    return []
                result.append(parent[result[-1]])
        return list(reversed(result))

    def segment_safe(self, a, b):
        # Check a swept grid traversal more densely than the cell size. Include
        # both side cells at diagonal transitions, not just rounded samples.
        count = max(1, int(math.ceil(math.dist(a, b)/self.resolution*4)))
        previous = self.cell(a)
        for t in np.linspace(0, 1, count+1):
            cell = self.cell((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
            if not self.inside(cell) or not self.safe[cell]:
                return False
            if cell[0] != previous[0] and cell[1] != previous[1]:
                if not self.safe[cell[0],previous[1]] or not self.safe[previous[0],cell[1]]:
                    return False
            previous = cell
        return True

    def waypoint(self, path, position, lookahead=.6):
        best = position
        for cell in path:
            candidate = self.xy(cell)
            if not self.segment_safe(position, candidate):
                break
            best = candidate
            if math.dist(position, candidate) >= lookahead:
                break
        return best
