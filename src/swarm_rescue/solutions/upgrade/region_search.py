"""Connected search regions and route costs from sensor-derived grids only."""
from __future__ import annotations

from collections import deque
import math
import cv2
import numpy as np


def distances(grid, start):
    """Four-connected geodesic distances; unknown cells are never traversed."""
    result = np.full(grid.shape, np.inf)
    if start is None or grid[start] != 1:
        return result
    queue = deque([start])
    result[start] = 0
    rows, cols = grid.shape
    while queue:
        r, c = queue.popleft()
        for rr, cc in ((r-1, c), (r+1, c), (r, c-1), (r, c+1)):
            if 0 <= rr < rows and 0 <= cc < cols and grid[rr, cc] == 1 and not math.isfinite(result[rr, cc]):
                result[rr, cc] = result[r, c]+1
                queue.append((rr, cc))
    return result


def choose_search_view(grid, coverage, start, owner, peers, reservations, blocked, step, cell_size=36):
    """Group nearby candidate views; prefer reachable, lightly searched regions.

    Connected candidate regions replace per-cell hash ownership. Geodesic
    Voronoi preference is soft, so an idle robot can finish another region.
    Reservations expire; a radio partition cannot permanently reserve work.
    """
    own = distances(grid, start)
    unknown = (grid == 0).astype(np.float32)
    missing = (1-coverage).astype(np.float32)
    missing[grid == 2] = 0
    # Local expected gain is deliberately a heuristic: exact occlusion is
    # checked by sensing at the selected view, not asserted from an unknown map.
    gain = cv2.boxFilter(missing, -1, (7, 7), normalize=False, borderType=cv2.BORDER_CONSTANT)
    boundary = cv2.dilate(unknown, np.ones((3, 3), np.uint8)) > 0
    candidates = (grid == 1) & np.isfinite(own) & (own >= 2) & ((coverage < .85) | boundary)
    for cell, until in blocked.items():
        if until > step:
            candidates[cell] = False
    if not np.any(candidates):
        return None
    _, labels = cv2.connectedComponents(candidates.astype(np.uint8), connectivity=8)
    preference = np.ones(grid.shape)
    for peer_id, peer_cell in peers.items():
        peer = distances(grid, peer_cell)
        closer = (peer < own) | ((peer == own) & (peer_id < owner))
        preference[closer] = np.maximum(preference[closer], 1.8)
    for peer_id, (cell, seen) in reservations.items():
        if peer_id == owner or not 0 <= step-seen <= 90:
            continue
        rr, cc = np.indices(grid.shape)
        near = np.hypot(rr-cell[0], cc-cell[1])
        preference *= 1+2*np.maximum(0, 1-near/4)
    # Penalize reaching around a wall into a distant region. This adopts C2's
    # soft continuity idea without imposing its 3D graph or CVRP dependency.
    rr, cc = np.indices(grid.shape)
    direct = np.maximum(1, np.hypot(rr-start[0], cc-start[1]))
    continuity = 1+np.maximum(0, own/direct-1)**2*.15
    utility = gain/(own*cell_size+90)/preference/continuity
    utility[~candidates] = -np.inf
    representatives = []
    for region in np.unique(labels[candidates]):
        indices = np.flatnonzero(labels == region)
        best = indices[np.argmax(utility.ravel()[indices])]
        representatives.append(best)
    selected = max(representatives, key=lambda i: (utility.ravel()[i], -i))
    return tuple(map(int, np.unravel_index(selected, grid.shape)))


class RouteCosts:
    """Bounded path estimates on a coarse map, reused for a short planning epoch."""
    def __init__(self, size, cell_size=24):
        self.width, self.height = map(int, size)
        self.cell_size = cell_size
        self.grid = None
        self.fields = {}
        self.epoch = -1

    def cell(self, point):
        return (int((self.height/2-point[1])//self.cell_size),
                int((point[0]+self.width/2)//self.cell_size))

    def refresh(self, evidence_map, step):
        if step//40 == self.epoch:
            return
        self.epoch = step//40
        rows, cols = math.ceil(self.height/self.cell_size), math.ceil(self.width/self.cell_size)
        rr, cc = np.indices((rows, cols))
        er = np.minimum(((rr+.5)*self.cell_size/evidence_map.cell_size).astype(int), evidence_map.rows-1)
        ec = np.minimum(((cc+.5)*self.cell_size/evidence_map.cell_size).astype(int), evidence_map.cols-1)
        known = evidence_map.evidence[er, ec] < 0
        padded = np.pad(evidence_map.bitmap, ((0, rows*self.cell_size-self.height),
                                             (0, cols*self.cell_size-self.width)), constant_values=1)
        walls = padded.reshape(rows, self.cell_size, cols, self.cell_size).max(axis=(1, 3)) > 0
        self.grid = np.where(walls, 2, np.where(known, 1, 0)).astype(np.uint8)
        self.fields.clear()

    def estimate(self, start, goal):
        direct = math.dist(start, goal)
        a, b = self.cell(start), self.cell(goal)
        if self.grid is None or not all(0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1] for r, c in (a, b)):
            return direct*1.3
        if a not in self.fields:
            self.fields[a] = distances(self.grid, a)
        length = self.fields[a][b]*self.cell_size
        # Unknown/disconnected on this coarse map is uncertain, not proof of
        # physical impossibility. Keep a finite conservative bid for discovery.
        return max(direct, float(length)) if math.isfinite(length) else direct*1.6+90
