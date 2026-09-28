#!/usr/bin/env python3
"""Small bounded 2-D frontier planner. No ROS, scene geometry or ground truth.

Free evidence is accumulated; occupied cells are rebuilt from the maintained
map plus CURRENT returns, so a cleared person does not become a permanent wall.
This is a fixed-height projection, not a proof of full 3-D collision freedom.
"""
import heapq
import math
import numpy as np


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
    def __init__(self, bounds, resolution=.2, clearance=.8, unknown_margin=.4):
        self.x0, self.y0, self.x1, self.y1 = map(float, bounds)
        self.resolution = float(resolution)
        if (not all(math.isfinite(v) for v in bounds) or resolution < .1 or
                self.x1 <= self.x0 or self.y1 <= self.y0 or clearance < .4 or
                unknown_margin < .4):
            raise ValueError('Invalid bounds/resolution/body clearance')
        self.shape = (int(math.ceil((self.y1-self.y0)/resolution)),
                      int(math.ceil((self.x1-self.x0)/resolution)))
        if self.shape[0]*self.shape[1] > 200000:
            raise ValueError('Exploration grid too large; bounded at 200000 cells')
        self.known = np.zeros(self.shape, dtype=bool)
        self.occupied = self.known.copy()
        self.safe = self.known.copy()
        self.body_known = self.known.copy()
        # Account for point-to-cell quantization in the same way as old A*.
        self.clearance = clearance / resolution + math.sqrt(2)/2
        self.unknown_margin = unknown_margin / resolution

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

    def reconnect(self, position, points, minimum_clearance=.70, max_distance=1.):
        """Recover when new returns invalidate the current inflated cell.

        NEVER erase inflation around the start. Only return a short segment to
        a normally safe cell if all sampled raw-point clearances are >= the
        start clearance (5mm numerical allowance) and >= the reactive warning
        distance. Body footprint must remain observed. Existing risk controller
        still overrides this output. This is not a 3-D collision certificate.
        """
        start = self.cell(position)
        if not self.inside(start) or not self.body_known[start] or not len(points):
            return None
        points = np.asarray(points)[:,:2]
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
                # 10cm command horizon keeps P-controller recovery slow; it is
                # revalidated every tick instead of blindly flying the join.
                point = position + min(1.,.10/max(length,1e-6))*(target-position)
                return tuple(float(v) for v in point)
        return None

    def search(self, start):
        """Dijkstra on only observed, inflated free cells; no corner cutting."""
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
                new = cost + self.resolution * (math.sqrt(2) if dx and dy else 1.)
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
