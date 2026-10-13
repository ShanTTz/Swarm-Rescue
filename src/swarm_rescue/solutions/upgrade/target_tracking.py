"""Track anonymous semantic observations without simulator entity identifiers.

An observation batch is associated one-to-one. Co-observed tracks cannot be
merged by a wide noise gate; radio aliases retain globally unique source IDs.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

Token = tuple[int, int]
Point = tuple[float, float]


@dataclass
class Observation:
    point: Point
    distance: float
    detection: Any = None


@dataclass
class Track:
    point: Point
    sigma: float
    observed: int
    source: int


def cluster_hits(hits: list[tuple[Any, Point]]) -> list[Observation]:
    """Aggregate repeated rays conservatively, without transitive chaining."""
    groups: list[list[tuple[Any, Point]]] = []
    for detection, point in sorted(hits, key=lambda hit: (hit[1], float(hit[0].distance))):
        if not all(math.isfinite(v) for v in (*point, float(detection.distance))):
            continue
        group = next((group for group in groups if
            all(math.dist(point, previous[1]) <= 20.0 for previous in group)), None)
        if group is None:
            groups.append([(detection, point)])
        else:
            group.append((detection, point))
    result = []
    for group in groups:
        representative = min(group, key=lambda hit: float(hit[0].distance))[0]
        point = tuple(sum(hit[1][axis] for hit in group) / len(group) for axis in (0, 1))
        result.append(Observation(point, float(representative.distance), representative))
    return result


class TargetTracker:
    def __init__(self, owner: int):
        self.owner = int(owner)
        self.sequence = 0
        self.tracks: dict[Token, Track] = {}
        self.aliases: dict[Token, Token] = {}
        self.separated: set[frozenset[Token]] = set()
        self.local_tokens: list[Token] = []
        self.retired: set[Token] = set()

    def retire(self, tokens) -> None:
        for raw in tokens if isinstance(tokens, (set, tuple, list)) else ():
            try:
                token = tuple(map(int, raw))
            except (TypeError, ValueError):
                continue
            if len(token) == 2:
                self.retired.add(self.resolve(token))

    def resolve(self, token: Token) -> Token:
        path = []
        while token in self.aliases:
            path.append(token)
            token = self.aliases[token]
        for value in path:
            self.aliases[value] = token
        return token

    def _new(self, observation: Observation, step: int) -> Token:
        self.sequence += 1
        token = self.owner, -1_000_000 - self.sequence
        self.tracks[token] = Track(observation.point, self._sigma(observation.distance), step, self.owner)
        return token

    @staticmethod
    def _sigma(distance: float) -> float:
        # Correlated GPS/heading errors are not reduced by the ray count.
        return 5.0 + min(15.0, max(0.0, distance) * 0.07)

    def _update(self, token: Token, observation: Observation, step: int, source: int) -> None:
        track = self.tracks[token]
        sigma = self._sigma(observation.distance)
        if step < track.observed:
            return
        weight = max(0.15, min(0.7, track.sigma ** 2 / (track.sigma ** 2 + sigma ** 2)))
        track.point = tuple((1-weight)*track.point[i] + weight*observation.point[i] for i in (0, 1))
        track.sigma = max(5.0, min(track.sigma, sigma))
        track.observed, track.source = step, source

    def observe(self, observations: list[Observation], step: int) -> list[Token]:
        # Lowest-cost pairs first, each track used at most once in this frame.
        pairs = []
        for index, observation in enumerate(observations):
            for token, track in self.tracks.items():
                if token in self.retired:
                    continue
                distance = math.dist(observation.point, track.point)
                gate = min(42.0, max(18.0, 1.6 * max(track.sigma, self._sigma(observation.distance))))
                if distance <= gate:
                    pairs.append((distance / gate, distance, index, token))
        assigned: dict[int, Token] = {}
        used: set[Token] = set()
        for _, _, index, token in sorted(pairs):
            if index not in assigned and token not in used:
                assigned[index] = token
                used.add(token)
        result = []
        for index, observation in enumerate(observations):
            token = assigned.get(index)
            if token is None:
                token = self._new(observation, step)
            else:
                self._update(token, observation, step, self.owner)
            result.append(token)
        for index, token in enumerate(result):
            for other in result[index+1:]:
                self.separated.add(frozenset((token, other)))
        self.local_tokens = result
        return result

    def merge(self, entries, aliases=(), separated=()) -> None:
        # Preserve sender batch distinctness even if its IDs differ from ours.
        used: set[Token] = set()
        batch = []
        for entry in entries:
            try:
                raw, point, sigma, step, source = entry
                token = int(raw[0]), int(raw[1])
                point = float(point[0]), float(point[1])
                sigma, step, source = float(sigma), int(step), int(source)
            except (TypeError, ValueError, IndexError):
                continue
            if not all(math.isfinite(v) for v in (*point, sigma)) or not 1 <= sigma <= 100:
                continue
            token = self.resolve(token)
            if token not in self.tracks:
                candidates = sorted((math.dist(point, track.point), other)
                    for other, track in self.tracks.items() if other not in used
                    and other not in self.retired
                    and frozenset((token, other)) not in self.separated)
                gate = min(28.0, max(16.0, sigma * 1.3))
                match = candidates[0][1] if candidates and candidates[0][0] <= gate and (
                    len(candidates) == 1 or candidates[1][0] - candidates[0][0] >= 7.0) else None
                self.tracks[token] = Track(point, sigma, step, source)
                if match is not None:
                    token = self.join(token, match)
            track = self.tracks[token]
            if step > track.observed:
                self._update(token, Observation(point, max(0, (sigma-5)/0.07)), step, source)
            used.add(token)
            batch.append(token)
        # A snapshot contains historical tracks, not one simultaneous frame.
        # Only explicit co-observation constraints prove distinct identities.
        for pair in separated if isinstance(separated, (tuple, list)) else ():
            try:
                a, b = pair
                a, b = self.resolve(tuple(map(int, a))), self.resolve(tuple(map(int, b)))
            except (TypeError, ValueError):
                continue
            if len(a) == len(b) == 2 and a != b:
                self.separated.add(frozenset((a, b)))
        for entry in aliases if isinstance(aliases, (tuple, list)) else ():
            try:
                raw, canonical = entry
                a, b = (int(raw[0]), int(raw[1])), (int(canonical[0]), int(canonical[1]))
            except (TypeError, ValueError, IndexError):
                continue
            self.join(a, b)

    def join(self, a: Token, b: Token) -> Token:
        a, b = self.resolve(a), self.resolve(b)
        if a == b or frozenset((a, b)) in self.separated:
            return a
        if (a in self.retired) != (b in self.retired):
            return a
        canonical, old = min(a, b), max(a, b)
        if canonical not in self.tracks and old not in self.tracks:
            return a
        if old in self.tracks:
            old_track = self.tracks.pop(old)
            current = self.tracks.get(canonical)
            if current is None or (old_track.observed, -old_track.sigma) > (current.observed, -current.sigma):
                self.tracks[canonical] = old_track
        self.aliases[old] = canonical
        if old in self.retired:
            self.retired.discard(old)
            self.retired.add(canonical)
        self.separated = {frozenset(canonical if value == old else value for value in pair)
                          for pair in self.separated}
        return canonical

    def snapshot(self):
        return tuple((token, track.point, track.sigma, track.observed, track.source)
                     for token, track in sorted(self.tracks.items()))
