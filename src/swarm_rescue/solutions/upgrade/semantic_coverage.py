"""Persistent semantic ray coverage, independent of geometric occupancy.

Nine samples per coarse cell record where a bomb-sized object could actually
intersect a semantic ray. This is a search heuristic, not a detection guarantee.
"""
from __future__ import annotations
import math
import numpy as np


class SemanticCoverage:
    def __init__(self, size, cell_size=36):
        self.width, self.height = map(int, size)
        self.cell_size = int(cell_size)
        self.rows = math.ceil(self.height / self.cell_size)
        self.cols = math.ceil(self.width / self.cell_size)
        self.mask = np.zeros((self.rows, self.cols), dtype=np.uint16)
        self.full_mask = 511
        self.cursor = 0
        offsets = np.array([(x, y) for y in (1/6, 1/2, 5/6) for x in (1/6, 1/2, 5/6)])
        rows, cols = np.indices(self.mask.shape)
        self.x = -self.width/2 + (cols[..., None]+offsets[:, 0])*self.cell_size
        self.y = self.height/2 - (rows[..., None]+offsets[:, 1])*self.cell_size

    def update(self, position, heading, lidar, lidar_angles, semantic_angles, detections=()):
        if lidar is None or semantic_angles is None or heading is None:
            return
        ranges, angles = np.asarray(lidar, dtype=float), np.asarray(lidar_angles, dtype=float)
        n = min(len(ranges), len(angles))
        if not n or not len(semantic_angles):
            return
        dx, dy = self.x-position[0], self.y-position[1]
        distance = np.hypot(dx, dy)
        selected = np.flatnonzero((distance <= 182.0) & (distance >= 4.0))
        if not selected.size:
            return
        bearings = np.arctan2(dy.ravel()[selected], dx.ravel()[selected]) - heading
        semantic_angles = np.asarray(semantic_angles, dtype=float)
        errors = np.abs(np.arctan2(np.sin(bearings[:, None]-semantic_angles),
                                  np.cos(bearings[:, None]-semantic_angles)))
        nearest = np.argmin(errors, axis=1)
        error = errors[np.arange(len(selected)), nearest]
        d = distance.ravel()[selected]
        # Bound the transverse distance to a semantic ray by a bomb-sized disk.
        visible = (error < math.pi/2) & (d*np.sin(error) <= 9.0)
        selected_angles = semantic_angles[nearest]
        lidar_errors = np.abs(np.arctan2(np.sin(selected_angles[:, None]-angles[:n]),
                                        np.cos(selected_angles[:, None]-angles[:n])))
        lidar_nearest = np.argmin(lidar_errors, axis=1)
        limit = ranges[lidar_nearest] - 12.0
        for detection in detections:
            try:
                angular_error = np.abs(np.arctan2(np.sin(selected_angles-float(detection.angle)),
                                                  np.cos(selected_angles-float(detection.angle))))
                limit = np.where(angular_error < 0.02, np.minimum(limit, float(detection.distance)-8), limit)
            except (TypeError, ValueError):
                continue
        visible &= np.isfinite(limit) & (d <= limit)
        indices = selected[visible]
        cell, sample = indices//9, indices%9
        np.bitwise_or.at(self.mask.ravel(), cell, (1 << sample).astype(np.uint16))

    def snapshot(self, count=40):
        known = np.flatnonzero(self.mask)
        if not len(known):
            return ()
        count = min(count, len(known))
        page = known[np.arange(self.cursor, self.cursor+count) % len(known)]
        self.cursor = (self.cursor+count) % len(known)
        return tuple((int(index//self.cols), int(index%self.cols), int(self.mask.ravel()[index])) for index in page)

    def merge(self, entries):
        for entry in entries:
            try:
                row, col, mask = map(int, entry)
            except (TypeError, ValueError):
                continue
            if 0 <= row < self.rows and 0 <= col < self.cols and 0 <= mask <= self.full_mask:
                self.mask[row, col] |= mask

    def observed_fraction(self):
        # Small lookup avoids a dependency on NumPy versions with bit_count.
        lookup = np.array([value.bit_count() for value in range(512)], dtype=float)
        return lookup[self.mask] / 9.0
