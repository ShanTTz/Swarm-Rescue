"""Small deterministic rectangular assignment; no solver dependency."""
from __future__ import annotations

import math


def minimum_assignment(costs: list[list[float]]) -> list[int]:
    """Hungarian shortest augmenting paths, O(rows² columns), rows <= columns.

    Returns one distinct column per row. Callers supply finite dummy costs for
    rejection. Infinite real edges are forbidden, not silently made feasible.
    """
    n = len(costs)
    if not n:
        return []
    m = len(costs[0])
    if m < n or any(len(row) != m for row in costs):
        raise ValueError('Require a rectangular matrix with rows <= columns')
    u, v, p, way = [0.]*(n+1), [0.]*(m+1), [0]*(m+1), [0]*(m+1)
    for row in range(1, n+1):
        p[0] = row
        col = 0
        best, used = [math.inf]*(m+1), [False]*(m+1)
        while True:
            used[col] = True
            i, delta, next_col = p[col], math.inf, 0
            for j in range(1, m+1):
                if used[j]:
                    continue
                value = float(costs[i-1][j-1])-u[i]-v[j]
                if value < best[j]:
                    best[j], way[j] = value, col
                if best[j] < delta:
                    delta, next_col = best[j], j
            if not math.isfinite(delta):
                raise ValueError('No finite assignment; supply rejection columns')
            for j in range(m+1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    best[j] -= delta
            col = next_col
            if p[col] == 0:
                break
        while col:
            previous = way[col]
            p[col] = p[previous]
            col = previous
    result = [-1]*n
    for j in range(1, m+1):
        if p[j]:
            result[p[j]-1] = j-1
    return result


def allocate_offers(offers: dict[int, dict[tuple[int, int], float]]) -> dict[tuple[int, int], int]:
    """Match actual fresh offers, instead of inventing bids for silent peers.

    Each vehicle receives at most one pickup. Cardinality comes before cost;
    the large rejection cost exceeds all possible finite team cost differences.
    This is local matching, not consensus under arbitrary network partitions.
    """
    owners = sorted(offers)
    tokens = sorted({token for entries in offers.values() for token in entries})
    if not owners or not tokens:
        return {}
    largest = max([1.] + [abs(value) for entries in offers.values()
                         for value in entries.values() if math.isfinite(value)])
    reject = (2*len(owners)+1)*(largest+1)
    matrix = [[offers[owner].get(token, math.inf) for token in tokens]
              + [reject]*len(owners) for owner in owners]
    selected = minimum_assignment(matrix)
    return {tokens[col]: owners[row] for row, col in enumerate(selected)
            if col < len(tokens) and math.isfinite(matrix[row][col])}
