"""Cached clearance graph with reachable target projections, from online maps.

Coarse nodes sample clearance at their physical centres. Unlike max-pooling
inflated pixels, this does not silently erase an entire narrow doorway. Every
returned segment is verified on the pixel clearance map before execution.
"""
import math
import cv2
import numpy as np
import heapq

try:
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra
except ImportError:  # Keep a NumPy-only fallback for lightweight submissions.
    csr_matrix = dijkstra = None

from swarm_rescue.solutions.upgrade.navigation import (
    _bidirectional_astar, _line_is_free, _nearest_free, _smooth_path)


class SensorRoutePlanner:
    def __init__(self, size, cell_size=12, clearance=18):
        self.size = tuple(map(int, size))
        self.cell_size = cell_size
        self.clearance = clearance
        self.epoch = -1
        self.blocked = self.raw = self.coarse = self.labels = None
        self.bid_cache = {}
        self.fine = None
        self.graph = None
        self.fields = {}

    def refresh(self, obstacles, step):
        epoch = step//40
        if self.epoch == epoch:
            return
        self.epoch = epoch
        self.raw = (np.asarray(obstacles) > 0).astype(np.uint8)
        width, height = self.size
        if self.raw.shape != (height, width):
            self.blocked = None
            return
        distance = cv2.distanceTransform(1-self.raw, cv2.DIST_L2, 5)
        self.blocked = (distance <= self.clearance+1).astype(np.uint8)
        margin = self.clearance+1
        self.blocked[:margin] = self.blocked[-margin:] = 1
        self.blocked[:, :margin] = self.blocked[:, -margin:] = 1
        rows, cols = math.ceil(height/self.cell_size), math.ceil(width/self.cell_size)
        rr = np.minimum((np.arange(rows)+.5)*self.cell_size, height-1).astype(int)
        cc = np.minimum((np.arange(cols)+.5)*self.cell_size, width-1).astype(int)
        self.coarse = self.blocked[np.ix_(rr, cc)] > 0
        _, self.labels = cv2.connectedComponents((~self.coarse).astype(np.uint8), connectivity=4)
        self.bid_cache.clear()
        self.fields.clear()
        if csr_matrix is not None:
            rows, cols = self.coarse.shape
            rr, cc = np.indices((rows, cols))
            free = ~self.coarse
            edge_rows, edge_cols, edge_costs = [], [], []
            for dr, dc in ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                r2, c2 = rr+dr, cc+dc
                valid = (r2 >= 0) & (r2 < rows) & (c2 >= 0) & (c2 < cols) & free
                r2, c2 = np.clip(r2, 0, rows-1), np.clip(c2, 0, cols-1)
                valid &= free[r2, c2]
                if dr and dc:
                    valid &= free[r2, cc] & free[rr, c2]
                edge_rows.append((rr*cols+cc)[valid])
                edge_cols.append((r2*cols+c2)[valid])
                edge_costs.append(np.full(int(valid.sum()), self.cell_size*math.hypot(dr, dc)))
            self.graph = csr_matrix((np.concatenate(edge_costs),
                (np.concatenate(edge_rows), np.concatenate(edge_cols))), shape=(rows*cols, rows*cols))

    def cell(self, point):
        width, height = self.size
        return int((height/2-point[1])//self.cell_size), int((point[0]+width/2)//self.cell_size)

    def world(self, cell):
        width, height = self.size
        return ((cell[1]+.5)*self.cell_size-width/2, height/2-(cell[0]+.5)*self.cell_size)

    def plan(self, start, goal, *, max_projection=48):
        route = self._plan(start, goal, max_projection=max_projection)
        if not route and self.cell_size > 8 and self.raw is not None:
            if self.fine is None:
                self.fine = SensorRoutePlanner(self.size, cell_size=8, clearance=self.clearance)
            self.fine.refresh(self.raw, self.epoch*40)
            route = self.fine.plan(start, goal, max_projection=max_projection)
        return route

    def _plan(self, start, goal, *, max_projection=48):
        if self.blocked is None:
            return []
        a = _nearest_free(self.cell(start), self.coarse, 6)
        if a is None:
            return []
        r, c = self.cell(goal)
        radius = int(math.ceil(max_projection/self.cell_size))+1
        candidates = []
        for rr in range(max(0, r-radius), min(self.coarse.shape[0], r+radius+1)):
            for cc in range(max(0, c-radius), min(self.coarse.shape[1], c+radius+1)):
                node = rr, cc
                distance = math.dist(self.world(node), goal)
                if distance <= max_projection and self.labels[node] == self.labels[a] and not self.coarse[node]:
                    candidates.append((distance+.03*math.dist(start, self.world(node)), node))
        if not candidates:
            return []
        b = min(candidates)[1]
        nodes = _bidirectional_astar(self.coarse, a, b)
        if not nodes:
            return []
        points = [self.world(node) for node in nodes]
        if not _line_is_free(start, points[0], self.size, self.raw):
            return []
        if any(not _line_is_free(x, y, self.size, self.blocked) for x, y in zip(points, points[1:])):
            return []
        if _line_is_free(points[-1], goal, self.size, self.blocked):
            points.append(tuple(goal))
        points = _smooth_path(points, self.size, self.blocked)
        # The first segment may escape an existing inflation-margin overlap;
        # physical walls and the local swept-footprint filter still constrain it.
        if not _line_is_free(start, points[0], self.size, self.raw):
            return []
        return points

    def _field(self, start):
        if dijkstra is not None and self.graph is not None:
            return dijkstra(self.graph, directed=True, indices=start[0]*self.coarse.shape[1]+start[1]).reshape(self.coarse.shape)
        result = np.full(self.coarse.shape, np.inf)
        result[start] = 0.
        queue = [(0., start)]
        rows, cols = result.shape
        while queue:
            cost, (r, c) = heapq.heappop(queue)
            if cost != result[r, c]:
                continue
            for dr, dc in ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                rr, cc = r+dr, c+dc
                if not (0 <= rr < rows and 0 <= cc < cols) or self.coarse[rr, cc]:
                    continue
                if dr and dc and (self.coarse[rr, c] or self.coarse[r, cc]):
                    continue
                following = cost+self.cell_size*math.hypot(dr, dc)
                if following < result[rr, cc]:
                    result[rr, cc] = following
                    heapq.heappush(queue, (following, (rr, cc)))
        return result

    def estimate(self, start, goal, *, reverse=False):
        direct = math.dist(start, goal)
        if self.blocked is None or _line_is_free(start, goal, self.size, self.blocked):
            return direct
        source, destination = (goal, start) if reverse else (start, goal)
        key = tuple(int(v//64) for v in source)
        a = _nearest_free(self.cell(source), self.coarse, 6)
        if a is None:
            return direct*1.6+90
        if key not in self.fields:
            self.fields[key] = self._field(a)
        field = self.fields[key]
        r, c = self.cell(destination)
        lengths = []
        for rr in range(max(0, r-4), min(self.coarse.shape[0], r+5)):
            for cc in range(max(0, c-4), min(self.coarse.shape[1], c+5)):
                residual = math.dist(self.world((rr, cc)), destination)
                if residual <= 48 and math.isfinite(field[rr, cc]):
                    lengths.append(float(field[rr, cc])+residual)
        # Shared fields replace a separate A* invocation for every target bid.
        # Unknown/disconnected geometry remains uncertain, not forbidden.
        return max(direct, min(lengths)) if lengths else direct*1.6+90
