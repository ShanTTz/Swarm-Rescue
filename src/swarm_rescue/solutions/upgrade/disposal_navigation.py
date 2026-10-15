"""Sensor-derived disposal footprints, separate from the wall map.

Connected measured surface returns are convexified. Unseen space beyond
those returns is never extrapolated to a known map shape.
"""
import math
import cv2
import numpy as np


class DisposalFootprint:
    CELL = 8

    def __init__(self, size):
        self.width, self.height = map(int, size)
        self.points = set()
        self.mask = np.zeros((self.height, self.width), dtype=np.uint8)
        self.dirty = False
        self.revision = 0
        self.cursor = 0

    def add(self, point):
        try:
            x, y = map(float, point)
        except (TypeError, ValueError):
            return
        if not math.isfinite(x) or not math.isfinite(y):
            return
        col = int(round((x + self.width / 2) / self.CELL))
        row = int(round((self.height / 2 - y) / self.CELL))
        if not (0 <= row * self.CELL < self.height and 0 <= col * self.CELL < self.width):
            return
        key = (row, col)
        if key not in self.points:
            self.points.add(key)
            self.dirty = True

    def snapshot(self, count=32):
        points = sorted(self.points)
        if not points:
            return ()
        selected = [points[(self.cursor + i) % len(points)] for i in range(min(count, len(points)))]
        self.cursor = (self.cursor + len(selected)) % len(points)
        return tuple((c * self.CELL - self.width/2, self.height/2-r*self.CELL) for r, c in selected)

    def rebuild(self):
        if not self.dirty:
            return False
        coarse = np.zeros((math.ceil(self.height/self.CELL), math.ceil(self.width/self.CELL)), dtype=np.uint8)
        points = np.asarray(sorted(self.points), dtype=np.int32)
        coarse[points[:, 0], points[:, 1]] = 1
        # Do not span a convex hull between distant pads.
        connected = cv2.dilate(coarse, np.ones((7, 7), np.uint8))
        _, labels = cv2.connectedComponents(connected)
        groups = labels[points[:, 0], points[:, 1]]
        self.mask.fill(0)
        for group in np.unique(groups):
            pixels = points[groups == group][:, ::-1].copy() * self.CELL
            hull = cv2.convexHull(pixels)
            if len(hull) >= 3:
                cv2.fillConvexPoly(self.mask, hull, 1)
            else:
                for pixel in pixels:
                    cv2.circle(self.mask, tuple(pixel), 4, 1, -1)
        self.dirty = False
        self.revision += 1
        return True

    def contains(self, point):
        col = int(round(float(point[0])+self.width/2))
        row = int(round(self.height/2-float(point[1])))
        return 0 <= row < self.height and 0 <= col < self.width and bool(self.mask[row, col])

    def crosses(self, start, path):
        previous = start
        for point in path:
            count = max(2, int(math.dist(previous, point)/4)+1)
            if any(self.contains((previous[0]+(point[0]-previous[0])*t,
                                  previous[1]+(point[1]-previous[1])*t))
                   for t in np.linspace(0, 1, count)):
                return True
            previous = point
        return False

    def near(self, point, distance=24):
        return self.contains(point) or any(math.hypot(c*self.CELL-self.width/2-point[0],
                                                     self.height/2-r*self.CELL-point[1]) <= distance
                                           for r, c in self.points)
