"""v13 repair: uncertain pickup hints accelerate stalled-task recovery."""
import math
import os

from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.solutions.upgrade.portal_navigation import PortalApproachPlanner


class MyDroneRescueV13(MyDroneRescueV12):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._vacated_targets = {}
        self._vacancy_enabled = os.environ.get('SWARM_RESCUE_V13_VACANCY', '1') == '1'
        self._block_history_enabled = os.environ.get('SWARM_RESCUE_V13_BLOCK_HISTORY', '1') == '1'
        self._vacancy_claim_guard = os.environ.get('SWARM_RESCUE_V13_CLAIM_GUARD', '0') == '1'
        self._vacancy_policy = os.environ.get('SWARM_RESCUE_V13_VACANCY_POLICY', 'hard')
        if self._vacancy_policy not in ('hard', 'soft', 'progress'):
            raise ValueError('Unknown SWARM_RESCUE_V13_VACANCY_POLICY')
        self._vacancy_decisions = set()
        self._portal_enabled = os.environ.get('SWARM_RESCUE_V13_PORTAL', '0') == '1'
        self._portal_planners = {}
        self._endgame_enabled = os.environ.get('SWARM_RESCUE_V13_ENDGAME', '0') == '1'
        self._disposal_vacancy_only = os.environ.get('SWARM_RESCUE_V13_DISPOSAL_VACANCY', '0') == '1'
        self._portal_progress_gate = os.environ.get('SWARM_RESCUE_V13_PORTAL_PROGRESS_GATE', '1') == '1'
        self._portal_recovery_tokens = set()
        self._vacancy_radius = float(os.environ.get('SWARM_RESCUE_V13_VACANCY_RADIUS', '18'))
        if not math.isfinite(self._vacancy_radius) or not 0 <= self._vacancy_radius <= 18:
            raise ValueError('SWARM_RESCUE_V13_VACANCY_RADIUS must be between 0 and 18')
        self._corner_guard_enabled = os.environ.get('SWARM_RESCUE_V13_CORNER_GUARD', '0') == '1'
        self._trajectory.peer_sweep = os.environ.get('SWARM_RESCUE_V13_PEER_SWEEP', '0') == '1'
        self._trajectory.peer_soft = os.environ.get('SWARM_RESCUE_V13_PEER_SOFT', '0') == '1'
        self._bomb_center_enabled = os.environ.get('SWARM_RESCUE_V13_BOMB_CENTER', '1') == '1'

    def _semantic_to_world(self, detection):
        point = super()._semantic_to_world(detection)
        if (point is None or not self._bomb_center_enabled
                or detection.entity_type != DroneSemanticSensor.TypeEntity.BOMB):
            return point
        # Public semantic distance ends at the visible surface. A 12px radial
        # shift approximates the known bomb footprint's centre and reduces
        # view-dependent split tracks. It is an estimate, never an entity ID.
        heading = self.measured_compass_angle()
        if heading is None:
            return point
        bearing = float(heading)+float(detection.angle)
        return point[0]+12.*math.cos(bearing), point[1]+12.*math.sin(bearing)

    def _navigate_online(self, target, cmd):
        if not self._corner_guard_enabled or self.grasped_bombs():
            return super()._navigate_online(target, cmd)
        # A clearance-checked polyline is safe only if its corners are reached
        # closely enough. The old 20px arrival tolerance skips them early,
        # leaving the local controller aiming across the corner of a wall.
        # Keep tolerance above the motion controller's 10px stopping radius.
        previous = self._PATROL_ARRIVE_RADIUS
        self._PATROL_ARRIVE_RADIUS = 12.
        try:
            return super()._navigate_online(target, cmd)
        finally:
            self._PATROL_ARRIVE_RADIUS = previous

    def _all_remaining_attached(self):
        if not self._endgame_enabled:
            return False
        delivered = sum(max(0., value) for value in self._team_deliveries.values())
        if not self._EXPECTED_BOMBS - 2.01 <= delivered < self._EXPECTED_BOMBS - .01:
            return False
        step, own_id = int(self.elapsed_timestep), int(self.identifier or 0)
        attached = 0
        for owner, task in self._team_tasks.items():
            mode, token, _, reported = task
            if (mode != 'carry' or token is None or not 0 <= step-reported <= 4
                    or self._target_tracker.resolve(token) in self._disposed_tokens):
                continue
            if owner == own_id:
                loaded = bool(self.grasped_bombs())
            else:
                peer = self._peer_state.get(owner)
                loaded = bool(peer and 0 <= step-peer[2] <= 4 and peer[4] and peer[1] > 0)
            progress = self._team_progress.get(owner)
            if (loaded and self._task_has_progress(owner, task) and progress is not None
                    and 0 <= step-progress[3] <= 120):
                # Count physical carriers, not possibly duplicated anonymous
                # target IDs. One load per carrier is a conservative lower bound.
                attached += 1
        return delivered + attached >= self._EXPECTED_BOMBS - .01

    def _select_unclaimed_bomb(self):
        return None if self._all_remaining_attached() else super()._select_unclaimed_bomb()

    def _select_recovery_target(self):
        return None if self._all_remaining_attached() else super()._select_recovery_target()

    def _select_discovered_target(self):
        return None if self._all_remaining_attached() else super()._select_discovered_target()

    def _should_wait_for_carriers(self):
        # Standby is temporary: drop, stale radio or lack of progress releases
        # it. Only real disposal rewards can mark the mission complete.
        return self._all_remaining_attached() or super()._should_wait_for_carriers()

    def _plan_navigation(self, start, target):
        route = super()._plan_navigation(start, target)
        if (not self._portal_enabled or self.grasped_bombs()
                or self._operational_stage not in ('chase', 'approach_shared', 'recover')
                or route and math.dist(route[-1], target) <= 24):
            return route
        if self._portal_progress_gate:
            state = self._task_monitor.current
            token = self._current_bomb_token
            if state is None or state.mode != 'claim' or token is None:
                return route
            token = self._target_tracker.resolve(token)
            if self._target_tracker.resolve(state.token) != token:
                return route
            recovering = any(self._target_tracker.resolve(raw) == token
                             for raw in self._portal_recovery_tokens)
            if not recovering and int(self.elapsed_timestep)-state.advanced < 120:
                return route
            self._portal_recovery_tokens.add(token)
        key = tuple(map(int, self.size_area)) if self.size_area else None
        if key is None:
            return route
        planner = self._portal_planners.get(key)
        if planner is None:
            planner = self._portal_planners[key] = PortalApproachPlanner(key, clearance=20)
        planner.refresh(self._online_obstacles, int(self.elapsed_timestep))
        repaired = planner.plan(start, target, max_projection=24)
        if repaired:
            self._diagnostic_event('portal_route', waypoints=len(repaired), rebuilds=planner.rebuilds)
            return repaired
        return route

    def _reset_explore_state(self):
        # Repeated chase calls must not erase consecutive motion failures.
        # Existing safe-motion and completed-escape code clears this history.
        if not self._block_history_enabled:
            return super()._reset_explore_state()

    def _remember_vacancy(self, token, point, at):
        token = self._target_tracker.resolve(token)
        previous = self._vacated_targets.get(token)
        if previous is None or at > previous[1]:
            self._vacated_targets[token] = (tuple(point), int(at))

    def _set_own_task(self, mode, token, point=None):
        owner = int(self.identifier or 0)
        previous = self._team_tasks.get(owner)
        if mode == 'carry' and token is not None and (previous is None or previous[:2] != (mode, token)):
            location = point if point is not None else self._bomb_token_centers.get(token)
            if location is not None:
                self._remember_vacancy(token, location, int(self.elapsed_timestep))
                self._diagnostic_event('pickup_vacancy', token=token, target=location)
        return super()._set_own_task(mode, token, point)

    def define_message_for_all(self):
        message = super().define_message_for_all()
        # Preserve pickup time when relaying; stale observations must not be
        # made fresh simply because another vehicle rebroadcast them.
        message['pickup_vacancies_v13'] = tuple((token, point, at)
            for token, (point, at) in sorted(self._vacated_targets.items()))
        return message

    def _merge_team_status(self):
        super()._merge_team_status()
        step = int(self.elapsed_timestep)
        for _, message in self.communicator.received_messages:
            if not isinstance(message, dict):
                continue
            entries = message.get('pickup_vacancies_v13', ())
            if not isinstance(entries, (tuple, list)):
                continue
            for entry in entries[:64]:
                try:
                    raw, point, at = entry
                    token = int(raw[0]), int(raw[1])
                    point, at = tuple(map(float, point)), int(at)
                except (TypeError, ValueError, IndexError, OverflowError):
                    continue
                if len(point) == 2 and all(math.isfinite(x) for x in point) and 0 <= at <= step:
                    self._remember_vacancy(token, point, at)

    def _vacancy_match(self, token):
        if not self._vacancy_enabled:
            return None
        token = self._target_tracker.resolve(token)
        observations = [source[token] for source in (self._discovered_targets, self._recovery_targets)
                        if token in source]
        if not observations:
            return None
        point, observed = max(observations, key=lambda entry: entry[1])
        matches = []
        for raw, (old_point, at) in self._vacated_targets.items():
            completed = self._target_tracker.resolve(raw)
            if self._disposal_vacancy_only and completed not in self._disposed_tokens:
                continue
            # Only an observation predating attachment can be suppressed.
            # A subsequent real observation reactivates a dropped bomb. Keep
            # explicitly co-observed neighbouring targets independently valid.
            if (observed <= at and math.dist(point, old_point) <= self._vacancy_radius
                    and frozenset((token, completed)) not in self._target_tracker.separated):
                matches.append((completed, point, old_point, observed, at))
        return min(matches, key=lambda item: (item[0] != token, -item[4])) if matches else None

    def _target_bid(self, owner, position, health, point):
        cost = super()._target_bid(owner, position, health, point)
        if self._vacancy_policy != 'soft' or owner != int(self.identifier or 0):
            return cost
        # A nearby pickup is an uncertain association, not proof that another
        # anonymous target disappeared. Make this lead less attractive, while
        # leaving it available when no better fresh lead is actionable.
        own = self._team_tasks.get(owner)
        for source in (self._discovered_targets, self._recovery_targets):
            for token, (location, _) in source.items():
                if tuple(location) != tuple(point):
                    continue
                token = self._target_tracker.resolve(token)
                match = self._vacancy_match(token)
                if (match is not None and token != match[0]
                        and not (own and own[0] == 'claim' and own[1] is not None
                                 and self._target_tracker.resolve(own[1]) == token)):
                    return cost + 150.
        return cost

    def _target_invalid(self, token):
        if super()._target_invalid(token):
            return True
        if self._vacancy_policy == 'progress':
            # A spatial pickup hint cannot prove another anonymous target is
            # gone. Recovery uses this hint only after measured stagnation.
            return False
        token = self._target_tracker.resolve(token)
        match = self._vacancy_match(token)
        if match is not None:
            completed, point, old_point, observed, at = match
            if self._vacancy_policy == 'soft' and token != completed:
                return False
            own = self._team_tasks.get(int(self.identifier or 0))
            # Proximity cannot revoke an independent accepted claim.
            protected = (self._vacancy_claim_guard and token != completed
                and own is not None and own[0] == 'claim' and own[1] is not None
                and self._target_tracker.resolve(own[1]) == token)
            decision = token, completed, at, protected
            if decision not in self._vacancy_decisions:
                self._vacancy_decisions.add(decision)
                self._diagnostic_event('vacancy_decision', token=token,
                    pickup_token=completed, target=point, pickup_point=old_point,
                    observed=observed, pickup_step=at, claim_protected=protected)
            return not protected
        return False

    def _own_task_stalled(self, mode, token):
        if super()._own_task_stalled(mode, token):
            return True
        if self._vacancy_policy != 'progress' or mode != 'claim' or token is None:
            return False
        state = self._task_monitor.current
        if (state is None or state.mode != mode
                or self._target_tracker.resolve(state.token) != self._target_tracker.resolve(token)
                or int(self.elapsed_timestep)-state.advanced < 120):
            return False
        if self._vacancy_match(token) is None:
            return False
        self._diagnostic_event('vacancy_no_progress', token=token, advanced=state.advanced)
        return True
