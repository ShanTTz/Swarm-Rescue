"""Fine clearance graph for failed/projected A* approaches, using sensor maps."""
import math

import cv2
import numpy as np

from swarm_rescue.solutions.upgrade.navigation_v12 import SensorRoutePlanner


class PortalApproachPlanner(SensorRoutePlanner):
    def __init__(self, size, clearance=20):
        super().__init__(size, cell_size=8, clearance=clearance)
        self.rebuilds = 0

    def refresh(self, obstacles, step):
        grid = (np.asarray(obstacles) > 0).astype(np.uint8)
        if self.raw is not None and np.array_equal(grid, self.raw):
            return
        width, height = self.size
        if grid.shape != (height, width):
            self.blocked = None
            return
        self.raw = grid
        self.epoch = int(step)
        self.rebuilds += 1
        distance = cv2.distanceTransform(1-grid, cv2.DIST_L2, 5)
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
        # Only A* path queries are needed: avoid building a large SciPy graph
        # or distance fields on every sensor-map update.
        self.graph = None
