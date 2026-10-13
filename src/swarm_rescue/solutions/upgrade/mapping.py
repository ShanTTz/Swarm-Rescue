"""Bounded, sensor-only occupancy evidence for online rescue navigation."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


class OnlineEvidenceMap:
    """Fuse sparse LiDAR scans while allowing later free rays to clear ghosts."""

    def __init__(self, size: Sequence[int], cell_size: int = 8) -> None:
        self.width, self.height = int(size[0]), int(size[1])
        self.cell_size = int(cell_size)
        self.rows = int(math.ceil(self.height / self.cell_size))
        self.cols = int(math.ceil(self.width / self.cell_size))
        self.evidence = np.zeros((self.rows, self.cols), dtype=np.int8)
        self.occupied = np.zeros((self.rows, self.cols), dtype=bool)
        self.bitmap = np.zeros((self.height, self.width), dtype=np.uint8)
        self.observed = np.full((self.rows, self.cols), -1, dtype=np.int32)
        self.sources = np.full((self.rows, self.cols), -1, dtype=np.int16)
        self._peer_stamps: dict[int, int] = {}
        self._snapshot_cursor = 0
        self._scan_source = -1
        self._scan_step = -1
        self._urgent: set[int] = set()

    def _apply(self, indices: np.ndarray, amount: int) -> list[tuple[int, int]]:
        if indices.size == 0:
            return []
        indices = np.unique(indices.astype(np.intp, copy=False))
        row, col = indices // self.cols, indices % self.cols
        old = self.evidence[row, col].astype(np.int16)
        new = np.clip(old + amount, -12, 12).astype(np.int8)
        self.evidence[row, col] = new
        if self._scan_step >= 0:
            self.observed[row, col] = self._scan_step
            self.sources[row, col] = self._scan_source
        before = self.occupied[row, col]
        after = np.where(before, new > 2, new >= 5)
        changed = np.flatnonzero(before != after)
        self._urgent.update(int(indices[index]) for index in changed)
        self.occupied[row, col] = after
        for index in changed:
            r, c = int(row[index]), int(col[index])
            self.bitmap[
                r * self.cell_size:min(self.height, (r + 1) * self.cell_size),
                c * self.cell_size:min(self.width, (c + 1) * self.cell_size),
            ] = int(after[index])
        return [(int(row[i]), int(col[i])) for i in changed]

    def update_scan(
        self,
        position: tuple[float, float],
        heading: float,
        distances: Sequence[float],
        angles: Sequence[float],
        dynamic_hits: Sequence[tuple[float, float]],
        *,
        stride: int = 6,
        source: int = -1,
        step: int = -1,
    ) -> tuple[tuple[tuple[int, int], ...], tuple[tuple[int, int], ...]]:
        self._scan_source, self._scan_step = int(source), int(step)
        count = min(len(distances), len(angles))
        if count == 0:
            return (), ()
        selected = np.arange(0, count, max(1, stride), dtype=int)
        ranges = np.asarray(distances, dtype=float)[selected]
        relative = np.asarray(angles, dtype=float)[selected]
        valid = np.isfinite(ranges) & np.isfinite(relative) & (ranges > 8.0)
        ranges, relative = ranges[valid], relative[valid]
        if not ranges.size:
            return (), ()
        bearing = float(heading) + relative
        hit = ranges < 285.0
        dynamic = np.zeros(len(ranges), dtype=bool)
        for dynamic_distance, dynamic_angle in dynamic_hits:
            if not math.isfinite(dynamic_distance) or not math.isfinite(dynamic_angle):
                continue
            angle_error = np.abs(np.arctan2(
                np.sin(relative - dynamic_angle), np.cos(relative - dynamic_angle)
            ))
            dynamic |= (angle_error <= 0.14) & (
                np.abs(ranges - dynamic_distance) <= 25.0
            )

        sample_distances = np.arange(0.0, 283.0, float(self.cell_size))
        free_limit = np.clip(ranges - 14.0, 0.0, 282.0)
        free_limit = np.where(hit, free_limit, 282.0)
        samples = sample_distances[None, :] <= free_limit[:, None]
        x = position[0] + np.cos(bearing)[:, None] * sample_distances[None, :]
        y = position[1] + np.sin(bearing)[:, None] * sample_distances[None, :]
        col = np.floor((x + self.width / 2.0) / self.cell_size).astype(int)
        row = np.floor((self.height / 2.0 - y) / self.cell_size).astype(int)
        valid_cells = (
            samples & (row >= 0) & (row < self.rows)
            & (col >= 0) & (col < self.cols)
        )
        free_indices = row[valid_cells] * self.cols + col[valid_cells]
        cleared = self._apply(free_indices, -1)

        # An object can first appear to LiDAR beyond semantic range, creating
        # a wall hypothesis. A later semantic classification must erase that
        # endpoint; clearing only the ray in front leaves a permanent ghost.
        dynamic_wall = hit & dynamic
        dynamic_x = position[0] + ranges[dynamic_wall] * np.cos(bearing[dynamic_wall])
        dynamic_y = position[1] + ranges[dynamic_wall] * np.sin(bearing[dynamic_wall])
        dynamic_col = np.floor((dynamic_x + self.width/2) / self.cell_size).astype(int)
        dynamic_row = np.floor((self.height/2 - dynamic_y) / self.cell_size).astype(int)
        valid_dynamic = ((dynamic_row >= 0) & (dynamic_row < self.rows)
                         & (dynamic_col >= 0) & (dynamic_col < self.cols))
        dynamic_indices = dynamic_row[valid_dynamic] * self.cols + dynamic_col[valid_dynamic]
        cleared += self._apply(dynamic_indices, -24)

        wall = hit & ~dynamic
        hit_x = position[0] + ranges[wall] * np.cos(bearing[wall])
        hit_y = position[1] + ranges[wall] * np.sin(bearing[wall])
        hit_col = np.floor((hit_x + self.width / 2.0) / self.cell_size).astype(int)
        hit_row = np.floor((self.height / 2.0 - hit_y) / self.cell_size).astype(int)
        valid_hits = (
            (hit_row >= 0) & (hit_row < self.rows)
            & (hit_col >= 0) & (hit_col < self.cols)
        )
        hit_row, hit_col = hit_row[valid_hits], hit_col[valid_hits]
        hit_indices = hit_row * self.cols + hit_col
        self._apply(hit_indices, 4)
        # Share confirmed wall cells seen in this scan; peers apply their own
        # evidence thresholds and can later clear a stale return.
        if hit_indices.size:
            unique = np.unique(hit_indices)
            confirmed = unique[
                self.occupied[unique // self.cols, unique % self.cols]
            ]
            hits = tuple(
                (int(index // self.cols), int(index % self.cols))
                for index in confirmed[:80]
            )
        else:
            hits = ()
        return hits, tuple(cleared[:80])

    def merge_peer_changes(
        self,
        hits: Sequence[tuple[int, int]],
        clears: Sequence[tuple[int, int]],
        *, source: int | None = None, step: int | None = None,
    ) -> None:
        if source is not None and step is not None:
            if step <= self._peer_stamps.get(source, -1):
                return
            self._peer_stamps[source] = step
        self._scan_source, self._scan_step = -1, -1
        for points, amount in ((hits, 4), (clears, -4)):
            valid = [
                row * self.cols + col
                for row, col in points
                if 0 <= row < self.rows and 0 <= col < self.cols
            ]
            self._apply(np.asarray(valid, dtype=np.intp), amount)

    def snapshot(self, count: int = 60):
        """Relay a rotating page of original observation versions, including free space."""
        known = np.flatnonzero(self.observed >= 0)
        if not known.size:
            return ()
        count = min(count, len(known))
        cursor = self._snapshot_cursor % len(known)
        page = known[np.arange(cursor, cursor+count) % len(known)]
        self._snapshot_cursor = (cursor+count) % len(known)
        # Prioritize changed walls/clears, while reserving most slots for the
        # rotating relay page. Original observation versions are retained.
        urgent = sorted(self._urgent)[:max(1, count//3)]
        self._urgent.difference_update(urgent)
        if urgent:
            page = np.asarray(urgent + [int(i) for i in page if int(i) not in urgent][:count-len(urgent)])
        rows, cols = np.unravel_index(page, self.evidence.shape)
        return tuple((int(r), int(c), int(self.evidence[r, c]),
                      int(self.observed[r, c]), int(self.sources[r, c])) for r, c in zip(rows, cols))

    def merge_snapshot(self, entries, now: int) -> None:
        self._scan_step = -1
        if not isinstance(entries, (tuple, list)):
            return
        accepted = {}
        for entry in entries:
            try:
                row, col, value, step, source = map(int, entry)
            except (TypeError, ValueError):
                continue
            if not (0 <= row < self.rows and 0 <= col < self.cols and -12 <= value <= 12
                    and 0 <= step <= now and 0 <= source < 1000):
                continue
            if (step, -source) <= (int(self.observed[row, col]), -int(self.sources[row, col])):
                continue
            previous = accepted.get((row, col))
            if previous is None or (step, -source) > (previous[1], -previous[2]):
                accepted[row, col] = value, step, source
        if not accepted:
            return
        cells = np.asarray(list(accepted), dtype=np.intp)
        values = np.asarray(list(accepted.values()), dtype=np.int32)
        row, col = cells[:, 0], cells[:, 1]
        before = self.occupied[row, col]
        after = np.where(before, values[:, 0] > 2, values[:, 0] >= 5)
        self.evidence[row, col] = values[:, 0]
        self.observed[row, col], self.sources[row, col] = values[:, 1], values[:, 2]
        self.occupied[row, col] = after
        for index in np.flatnonzero(before != after):
            r, c = int(row[index]), int(col[index])
            self._urgent.add(r*self.cols+c)
            self.bitmap[r*self.cell_size:min(self.height, (r+1)*self.cell_size),
                        c*self.cell_size:min(self.width, (c+1)*self.cell_size)] = int(after[index])
