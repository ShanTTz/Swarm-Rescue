"""Bounded 2-D Lazy Theta* on cell centres, with conservative supercover LOS.

Adapted from Nash, Koenig and Tovey, AAAI 2010. Optimistic parent shortcuts
are validated when a node is expanded and repaired through closed neighbors.
This is an any-angle experiment, not an optimality or dynamic-safety guarantee.
"""
from __future__ import annotations

import heapq
import math
import numpy as np


def grid_line_is_free(blocked, start, end):
    """Check every crossed cell, including both sides of an exact corner."""
    rows, cols = blocked.shape
    y, x = start
    y1, x1 = end
    if not (0 <= y < rows and 0 <= x < cols and 0 <= y1 < rows and 0 <= x1 < cols):
        return False
    if blocked[y, x] or blocked[y1, x1]:
        return False
    nx, ny = abs(x1-x), abs(y1-y)
    sx, sy = (1 if x1 > x else -1), (1 if y1 > y else -1)
    ix = iy = 0
    while ix < nx or iy < ny:
        decision = (1+2*ix)*ny-(1+2*iy)*nx
        if decision == 0:
            if blocked[y, x+sx] or blocked[y+sy, x]:
                return False
            x, y = x+sx, y+sy
            ix, iy = ix+1, iy+1
        elif decision < 0:
            x, ix = x+sx, ix+1
        else:
            y, iy = y+sy, iy+1
        if blocked[y, x]:
            return False
    return True


def lazy_theta_search(blocked: np.ndarray, start, goal):
    rows, cols = blocked.shape
    if any(not (0 <= p[0] < rows and 0 <= p[1] < cols) or blocked[p] for p in (start, goal)):
        return []
    if start == goal:
        return [start]
    moves = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))

    def neighbors(node):
        r, c = node
        for dr, dc in moves:
            nxt = r+dr, c+dc
            if (0 <= nxt[0] < rows and 0 <= nxt[1] < cols and not blocked[nxt]
                    and not (dr and dc and (blocked[r+dr, c] or blocked[r, c+dc]))):
                yield nxt

    costs, parents, closed = {start: 0.}, {start: start}, set()
    heap = [(math.dist(start, goal), 0., start)]
    limit = min(rows*cols, 5000)
    while heap and len(closed) < limit:
        _, queued_cost, node = heapq.heappop(heap)
        if node in closed or queued_cost > costs[node]+1e-9:
            continue
        parent = parents[node]
        if not grid_line_is_free(blocked, parent, node):
            repairs = [(costs[n]+math.dist(n, node), n) for n in neighbors(node) if n in closed]
            if not repairs:
                continue
            costs[node], parents[node] = min(repairs)
        if node == goal:
            path = [goal]
            while path[-1] != start:
                nxt = parents[path[-1]]
                if nxt in path:  # Defensive cycle guard, never return a partial route.
                    return []
                path.append(nxt)
            path.reverse()
            return path if all(grid_line_is_free(blocked, a, b) for a, b in zip(path, path[1:])) else []
        closed.add(node)
        parent = parents[node]
        for nxt in neighbors(node):
            if nxt in closed:
                continue
            candidate = costs[parent]+math.dist(parent, nxt)
            if candidate+1e-9 < costs.get(nxt, math.inf):
                costs[nxt], parents[nxt] = candidate, parent
                heapq.heappush(heap, (candidate+math.dist(nxt, goal), candidate, nxt))
    return []
