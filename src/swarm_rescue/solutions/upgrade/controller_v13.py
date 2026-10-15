"""v13 repair: uncertain pickup hints accelerate stalled-task recovery."""
import math
import os
from collections import deque
import numpy as np

from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.solutions.upgrade.portal_navigation import PortalApproachPlanner
from swarm_rescue.solutions.upgrade.disposal_navigation import DisposalFootprint
from swarm_rescue.solutions.upgrade.carrier_traffic import CarrierRightOfWay, EmptyCongestionRecovery
from swarm_rescue.solutions.upgrade.navigation import plan_occupancy_path


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
        self._disposal_footprint = DisposalFootprint(self.size_area) if self.size_area else None
        self._disposal_avoid_active = False
        self._disposal_motion_window = deque(maxlen=13)
        self._payload_motion_window = deque(maxlen=9)
        self._payload_wide_route = False
        self._opening_scout_released = False
        self._opening_scout_goal = self._patrol_route[2] if len(self._patrol_route) >= 3 else None
        self._carrier_traffic = CarrierRightOfWay()
        self._empty_traffic = EmptyCongestionRecovery()
        self._traffic_motion_goal = None
        self._traffic_checked_step = -1
        self._traffic_standby = False

    def _control_step(self):
        cmd = super()._control_step()
        step = int(self.elapsed_timestep)
        # Idle/standby/near-pickup branches may not call the rollout at all.
        if self._traffic_checked_step != step:
            target = self._traffic_target()
            if target is not None:
                self._traffic_rollout(target,cmd,True)
        if self._carrier_traffic.owner is not None and not self.grasped_bombs():
            cmd['grasper'] = 0
        if self._empty_traffic.target is not None and not self.grasped_bombs():
            cmd['grasper'] = 0
        if getattr(self,'_traffic_standby',False) and not self.grasped_bombs():
            cmd['grasper'] = 0
        return cmd

    def _rollout_toward(self, x, y, cmd):
        self._traffic_motion_goal = (float(x), float(y))
        target = self._traffic_target()
        result=self._traffic_rollout(target if target is not None else (x,y),cmd,target is not None)
        # Reaching a temporary refuge is not arrival at the original task.
        return False if target is not None and result is not None else result

    def _traffic_rollout(self, target, cmd, escaping):
        critic=getattr(self,'_trajectory',None)
        if critic is None:
            return super()._rollout_toward(*target,cmd)
        previous=critic.escape_peers
        critic.escape_peers=bool(escaping and not self.grasped_bombs())
        try:
            return super()._rollout_toward(*target,cmd)
        finally:
            critic.escape_peers=previous

    def _traffic_target(self):
        self._traffic_standby=False
        step = self._traffic_checked_step = int(self.elapsed_timestep)
        position, heading = self.measured_gps_position(), self.measured_compass_angle()
        if position is None or heading is None or not self._pose_estimate.can_map:
            self._carrier_traffic.clear()
            self._empty_traffic.clear()
            return None
        if self.grasped_bombs():
            self._carrier_traffic.clear()
            self._empty_traffic.clear()
            return None
        goals = {}
        for _, message in self.communicator.received_messages:
            if not isinstance(message, dict): continue
            goal, seen, owner = message.get('traffic_goal_v1'), message.get('step'), message.get('id')
            if not isinstance(owner,int) or not isinstance(seen,int) or not 0 <= step-seen <= 6: continue
            if isinstance(goal,(tuple,list)) and len(goal)==2:
                try:
                    point=tuple(map(float,goal))
                    if all(math.isfinite(v) for v in point): goals[owner]=point
                except (TypeError,ValueError,OverflowError): pass
        previous=self._carrier_traffic.owner
        target=self._carrier_traffic.choose(tuple(map(float,position)),int(self.identifier or 0),
                   bool(self.grasped_bombs()),step,self._peer_state,goals,self._traffic_segment_free,
                   own_goal=self._traffic_motion_goal,eligible=self._traffic_eligible_carriers())
        if self._carrier_traffic.owner != previous:
            self._diagnostic_event('traffic_yield', peer=self._carrier_traffic.owner, target=target,
                                   carrier_goal=goals.get(self._carrier_traffic.owner))
            if previous is not None:
                self._nav_path=[]
                self._nav_target=None
        if target is not None:
            self._empty_traffic.clear()
            return target
        if not self._empty_escape_enabled():
            self._empty_traffic.clear()
            return None
        previous_escape=self._empty_traffic.target
        col=int(round(position[0]+self.size_area[0]/2))
        row=int(round(self.size_area[1]/2-position[1]))
        wall_near=bool(np.any(self._online_obstacles[max(0,row-22):row+23,max(0,col-22):col+23]))
        visible=any(obj.entity_type==DroneSemanticSensor.TypeEntity.BOMB and not obj.grasped and obj.distance<55
                    for obj in self.semantic_values() or ())
        target=self._empty_traffic.choose(tuple(map(float,position)),int(self.identifier or 0),False,
                   int(self.drone_health),step,self._peer_state,wall_near,visible,
                   self._traffic_motion_goal,self._traffic_segment_free)
        if previous_escape is None and target is not None:
            self._diagnostic_event('empty_congestion_escape', target=target, wall_near=wall_near)
        elif previous_escape is not None and target is None:
            self._nav_path=[]
            self._nav_target=None
        if target is None and self._traffic_can_standby():
            self._traffic_standby=True
            return tuple(map(float,position))
        return target

    def _traffic_can_standby(self):
        if self.grasped_bombs() or not self._physical_loads_accounted():
            return False
        if any(obj.entity_type==DroneSemanticSensor.TypeEntity.BOMB and not obj.grasped
               for obj in self.semantic_values() or ()):
            return False
        values=self.lidar_values()
        if values is None:
            return False
        values=np.asarray(values,dtype=float)
        valid=np.isfinite(values)&(values>0)
        if not np.any(valid) or float(np.min(values[valid]))<60:
            return False
        position=self.measured_gps_position()
        if position is None:
            return False
        step=int(self.elapsed_timestep)
        return not any(owner!=int(self.identifier or 0) and loaded and health>0
                       and 0<=step-seen<=6 and math.dist(position,point)<120
                       for owner,(point,health,seen,_,loaded,_) in self._peer_state.items())

    def _traffic_eligible_carriers(self):
        if self._max_steps>2000:
            return None
        if self._known_disposal is None or not self.size_area:
            return set()
        # Near-home crossings in short missions are already handled by the
        # original delivery policy. Avoid disrupting rapid pickup/search there;
        # use the new queue arbitration on the long remote transport leg.
        distance=.5*max(self.size_area)
        return {owner for owner,(point,_,_,_,loaded,_) in self._peer_state.items()
                if loaded and math.dist(point,self._known_disposal)>distance}

    def _empty_escape_enabled(self):
        if self._max_steps>2000:
            return True
        # Preserve short-deadline scouting until every outstanding load has a
        # physical carrier. This gates an optional escape, never mission end.
        return self._physical_loads_accounted()

    def _physical_loads_accounted(self):
        step=int(self.elapsed_timestep)
        delivered=sum(max(0.,value) for value in self._team_deliveries.values())
        carrying=int(bool(self.grasped_bombs()))
        carrying+=sum(owner!=int(self.identifier or 0) and health>0 and loaded
                      and 0<=step-seen<=4
                      for owner,(_,health,seen,_,loaded,_) in self._peer_state.items())
        return delivered+carrying>=self._EXPECTED_BOMBS-.01

    def _traffic_segment_free(self, start, target):
        heading=self.measured_compass_angle()
        ranges,angles=self.lidar_values(),self.lidar_rays_angles()
        if heading is None or ranges is None or angles is None: return False
        direction=math.atan2(target[1]-start[1],target[0]-start[0])-heading
        ranges,angles=np.asarray(ranges,dtype=float),np.asarray(angles,dtype=float)
        valid=np.isfinite(ranges)&(ranges>0)&(np.abs(np.arctan2(np.sin(angles-direction),np.cos(angles-direction)))<.14)
        length=math.dist(start,target)
        if not np.any(valid) or float(np.min(ranges[valid]))<length+16: return False
        footprint=self._disposal_footprint
        if footprint and footprint.crosses(start,[target]): return False
        grid=self._online_obstacles
        delta=(target[0]-start[0])/length,(target[1]-start[1])/length
        points=np.asarray([(start[0]+t*(target[0]-start[0])-delta[1]*offset,
                            start[1]+t*(target[1]-start[1])+delta[0]*offset)
                           for t in np.linspace(0,1,max(3,int(length/4))) for offset in (-13,0,13)])
        cols=np.rint(points[:,0]+self.size_area[0]/2).astype(int)
        rows=np.rint(self.size_area[1]/2-points[:,1]).astype(int)
        if np.any(cols<0) or np.any(cols>=grid.shape[1]) or np.any(rows<0) or np.any(rows>=grid.shape[0]): return False
        return not bool(np.any(grid[rows,cols]))

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

    def _plan_with_disposal_layer(self, start, target):
        if self.grasped_bombs() and getattr(self, '_payload_wide_route', False) and self._planner_backend == 'astar':
            # The load projects beyond the drone. A path clear for its centre
            # can still be impossible for the local controller at a wall end.
            # Prefer room for the load; retain the original planner when a
            # narrow passage is the only observed route.
            for cell in (16, 8):
                route = plan_occupancy_path(self.size_area, self._online_obstacles,
                    start, target, clearance=32, cell_size=cell)
                if route:
                    return route
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
        footprint = getattr(self, '_disposal_footprint', None)
        message['disposal_surface_v1'] = footprint.snapshot() if footprint else ()
        message['traffic_goal_v1'] = self._traffic_motion_goal
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

    def _opening_scout_active(self):
        # The existing coverage policy gives the last two drones opening
        # probe legs. Keep a small scout reserve on short-deadline missions
        # instead of letting every scout turn back with an early local load.
        if (getattr(self, '_max_steps', 5000) > 2000
                or getattr(self, '_team_size', 1) < 6
                or int(self.identifier or 0) < self._team_size-2
                or getattr(self, '_opening_scout_released', False)):
            return False
        sector_seen = False
        goal = getattr(self, '_opening_scout_goal', None)
        if self._patrol_index >= 2 and goal is not None:
            position = self.measured_gps_position()
            if position is not None:
                gap = math.dist(position, goal)
                # Finish distinct coverage sectors, rather than both releasing
                # at their shared transit waypoint. Stop before pushing into
                # a bomb lying at the probe point itself.
                sector_seen = gap <= 50 or (gap <= 180 and any(
                    obj.entity_type == DroneSemanticSensor.TypeEntity.BOMB
                    and not obj.grasped and math.isfinite(float(obj.distance)) and obj.distance <= 140
                    for obj in self.semantic_values() or ()))
        if (int(self.elapsed_timestep) >= int(self._max_steps*.45)
                or self._patrol_index >= 3 or sector_seen):
            self._opening_scout_released = True
            self._diagnostic_event('opening_scout_complete', leg=self._patrol_index)
            return False
        return True

    def _own_offer(self):
        return {} if self._opening_scout_active() else super()._own_offer()

    def _can_claim_target(self, token, point):
        return False if self._opening_scout_active() else super()._can_claim_target(token, point)

    def _control_explore(self, cmd):
        if self._opening_scout_active():
            return self._control_patrol(cmd)
        return super()._control_explore(cmd)

    def _target_bid(self, owner, position, health, point):
        # Above half health, geometry determines assignment. A single timed
        # collision must not add an abrupt 55px penalty at health 45 and
        # reshuffle otherwise capable searchers. Keep the original low-health
        # risk model and retirement rules for vulnerable drones.
        bid_health = 50 if health >= 25 else health
        cost = super()._target_bid(owner, position, bid_health, point)
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



    def _update_online_obstacles(self):
        super()._update_online_obstacles()
        footprint = getattr(self, '_disposal_footprint', None)
        step = int(self.elapsed_timestep)
        if footprint is None or step % 8 != int(self.identifier or 0) % 8:
            return
        position, heading = self.measured_gps_position(), self.measured_compass_angle()
        trusted = not hasattr(self, '_pose_estimate') or self._pose_estimate.can_map
        if trusted and position is not None and heading is not None:
            for obj in self.semantic_values() or ():
                if obj.entity_type != DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                    continue
                if not math.isfinite(float(obj.distance)) or not math.isfinite(float(obj.angle)):
                    continue
                bearing = float(heading) + float(obj.angle)
                footprint.add((position[0] + obj.distance*math.cos(bearing),
                               position[1] + obj.distance*math.sin(bearing)))
        for _, message in self.communicator.received_messages:
            if not isinstance(message, dict):
                continue
            timestamp = message.get('step')
            if not isinstance(timestamp, int) or not 0 <= step-timestamp <= 16:
                continue
            points = message.get('disposal_surface_v1', ())
            if isinstance(points, (tuple, list)):
                for point in points[:32]:
                    footprint.add(point)
        changed = footprint.rebuild()
        if self.grasped_bombs():
            self._disposal_avoid_active = False
            self._disposal_motion_window.clear()
            if position is not None and trusted:
                window = self._payload_motion_window
                window.append((step, tuple(map(float, position))))
                if len(window) == window.maxlen and not self._payload_wide_route:
                    earliest, start = window[0]
                    spread = max(math.dist(start, point) for _, point in window)
                    # Keep original carrying and delivery until a distant
                    # waypoint has demonstrably trapped the loaded drone.
                    visible_pad = any(obj.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER
                                      for obj in self.semantic_values() or ())
                    if (not visible_pad and step-earliest >= 56 and spread < 64 and math.dist(start, position) < 20
                            and self._known_disposal is not None
                            and math.dist(position, self._known_disposal) > 80
                            and self._carry_exit_steps_left <= 0):
                        self._payload_wide_route = True
                        self._nav_path = []
                        self._nav_target = None
                        self._diagnostic_event('payload_clearance_replan', spread=spread)
            return
        self._payload_motion_window.clear()
        self._payload_wide_route = False
        if position is not None and trusted and footprint.points:
            window = self._disposal_motion_window
            window.append((step, tuple(map(float, position))))
            if not self._disposal_avoid_active and len(window) == window.maxlen:
                earliest, start = window[0]
                spread = max(math.dist(start, point) for _, point in window)
                blocked = (self._nav_target is not None and footprint.near(self._nav_target)
                           or footprint.crosses(position, self._nav_path))
                delivered = sum(max(0., value) for value in self._team_deliveries.values())
                schedule_safe = self._max_steps > 2000 or delivered >= self._EXPECTED_BOMBS-2
                if (schedule_safe and step-earliest >= 88 and spread < 42 and blocked
                        and self._operational_stage in ('search', 'approach_shared', 'recover')):
                    self._disposal_avoid_active = True
                    self._nav_path = []
                    self._nav_target = None
                    self._diagnostic_event('disposal_stall_replan', spread=spread)
        if changed and self._disposal_avoid_active and position is not None:
            if footprint.crosses(position, self._nav_path):
                self._nav_path = []
                self._nav_target = None

    def _control_patrol(self, cmd):
        footprint = getattr(self, '_disposal_footprint', None)
        if footprint and getattr(self, '_disposal_avoid_active', False) and not self.grasped_bombs():
            index = self._patrol_index % len(self._patrol_route)
            target = self._patrol_route[index]
            position = self.measured_gps_position()
            if position is not None and self._nav_target != target and footprint.near(target, 32):
                route = self._plan_navigation(tuple(map(float, position)), target)
                if route and math.dist(route[-1], target) > 10:
                    targets = list(self._patrol_route)
                    targets[index] = route[-1]
                    self._patrol_route = tuple(targets)
                    self._nav_path = route
                    self._nav_target = route[-1]
                    self._patrol_target_age = 0
                    self._diagnostic_event('disposal_patrol_project', target=target, projected=route[-1])
        return super()._control_patrol(cmd)


    def _plan_navigation(self, start, target):
        footprint = getattr(self, '_disposal_footprint', None)
        route = self._plan_with_disposal_layer(start, target)
        if (footprint is None or not footprint.revision or self.grasped_bombs()
                or not getattr(self, '_disposal_avoid_active', False)
                or not (footprint.crosses(start, route) or footprint.near(target))):
            return route
        original = self._online_obstacles
        self._online_obstacles = np.maximum(original, footprint.mask)
        try:
            return self._plan_with_disposal_layer(start, target)
        finally:
            self._online_obstacles = original
