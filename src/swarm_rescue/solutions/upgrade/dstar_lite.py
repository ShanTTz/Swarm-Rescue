"""D* Lite (Koenig and Likhachev, AAAI 2002), bounded for online control.

Reuses g/rhs values when the start moves or occupancy changes. A changed goal
or grid shape starts a fresh search. Paths use eight neighbors without corner
cuts; continuous collision validation remains in the occupancy wrapper.
"""
from __future__ import annotations

import heapq
import math
import numpy as np


class DStarLite:
    MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))

    def __init__(self, expansion_limit=5000):
        self.blocked = None
        self.goal = None
        self.expansion_limit = expansion_limit
        self.resets = self.reuses = self.last_expansions = 0

    @staticmethod
    def _distance(a, b):
        dr, dc = abs(a[0]-b[0]), abs(a[1]-b[1])
        return max(dr, dc)+(math.sqrt(2.)-1.)*min(dr, dc)

    @staticmethod
    def _equal(a, b):
        return a == b or (math.isfinite(a) and math.isfinite(b) and abs(a-b) < 1e-9)

    def _adjacent(self, node):
        r, c = node
        rows, cols = self.blocked.shape
        for dr, dc in self.MOVES:
            if 0 <= r+dr < rows and 0 <= c+dc < cols:
                yield r+dr, c+dc

    def _neighbors(self, node):
        if self.blocked[node]:
            return
        r, c = node
        for nxt in self._adjacent(node):
            dr, dc = nxt[0]-r, nxt[1]-c
            if (not self.blocked[nxt]
                    and not (dr and dc and (self.blocked[r+dr, c] or self.blocked[r, c+dc]))):
                yield nxt

    def _key(self, node):
        value = min(self.g[node], self.rhs[node])
        return value+self._distance(self.start, node)+self.km, value

    def _queue(self, node):
        key = self._key(node)
        self.pending[node] = key
        heapq.heappush(self.heap, (key, node))

    def _update(self, node):
        if node != self.goal:
            self.rhs[node] = min((self._distance(node, n)+self.g[n] for n in self._neighbors(node)), default=math.inf)
        self.pending.pop(node, None)
        if not self._equal(self.g[node], self.rhs[node]):
            self._queue(node)

    def _top(self):
        while self.heap and self.pending.get(self.heap[0][1]) != self.heap[0][0]:
            heapq.heappop(self.heap)
        return self.heap[0][0] if self.heap else (math.inf, math.inf)

    def __call__(self, blocked, start, goal):
        blocked = np.asarray(blocked, dtype=bool)
        rows, cols = blocked.shape
        if any(not (0 <= p[0] < rows and 0 <= p[1] < cols) or blocked[p] for p in (start, goal)):
            return []
        if self.blocked is None or self.blocked.shape != blocked.shape or self.goal != goal:
            self.blocked, self.goal, self.start = blocked.copy(), goal, start
            self.g = np.full(blocked.shape, math.inf)
            self.rhs = self.g.copy()
            self.rhs[goal] = 0.
            self.km, self.heap, self.pending = 0., [], {}
            self._queue(goal)
            self.resets += 1
        else:
            self.km += self._distance(self.start, start)
            self.start = start
            changed = np.argwhere(self.blocked != blocked)
            self.blocked[:] = blocked
            affected = set()
            for r, c in changed:
                node = int(r), int(c)
                affected.add(node)
                affected.update(self._adjacent(node))
            for node in sorted(affected):
                self._update(node)
            self.reuses += 1
        if len(self.heap) > 4*blocked.size+64:
            self.heap = [(key, node) for node, key in self.pending.items()]
            heapq.heapify(self.heap)
        self.last_expansions = 0
        while self._top() < self._key(start) or not self._equal(self.rhs[start], self.g[start]):
            if not self.heap or self.last_expansions >= self.expansion_limit:
                return []  # Never execute an unfinished incremental repair.
            old_key, node = heapq.heappop(self.heap)
            self.pending.pop(node, None)
            new_key = self._key(node)
            if old_key < new_key:
                self._queue(node)
                continue
            self.last_expansions += 1
            if self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for neighbor in self._adjacent(node):
                    self._update(neighbor)
            else:
                self.g[node] = math.inf
                self._update(node)
                for neighbor in self._adjacent(node):
                    self._update(neighbor)
        if not math.isfinite(self.g[start]):
            return []
        path, seen = [start], {start}
        while path[-1] != goal:
            node = path[-1]
            candidates = [(self._distance(node, n)+self.g[n], n) for n in self._neighbors(node)]
            if not candidates:
                return []
            _, nxt = min(candidates)
            if nxt in seen or not math.isfinite(self.g[nxt]) or self.g[nxt] >= self.g[node]-1e-9:
                return []
            seen.add(nxt)
            path.append(nxt)
        return path
