"""Frame-wide assignment for anonymous adjacent bomb observations."""
import math

from swarm_rescue.solutions.upgrade.assignment import minimum_assignment
from swarm_rescue.solutions.upgrade.target_tracking import TargetTracker


class GlobalTargetTracker(TargetTracker):
    def observe(self, observations, step):
        tokens = sorted(token for token in self.tracks if token not in self.retired)
        costs = []
        for observation in observations:
            row = []
            for token in tokens:
                track = self.tracks[token]
                distance = math.dist(observation.point, track.point)
                gate = min(42., max(18., 1.6*max(track.sigma, self._sigma(observation.distance))))
                row.append(distance/gate if distance <= gate else math.inf)
            # Prioritize feasible association cardinality, then residual. A
            # greedy first pair can steal the only feasible track of another hit.
            row += [len(observations)+1.]*len(observations)
            costs.append(row)
        selected = minimum_assignment(costs)
        result = []
        for index, observation in enumerate(observations):
            col = selected[index]
            if col < len(tokens):
                token = tokens[col]
                self._update(token, observation, step, self.owner)
            else:
                token = self._new(observation, step)
            result.append(token)
        for index, token in enumerate(result):
            for other in result[index+1:]:
                self.separated.add(frozenset((token, other)))
        self.local_tokens = result
        return result
