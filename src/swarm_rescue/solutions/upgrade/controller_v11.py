"""v11 candidate: explicit task offers, joint tracking and payload rollouts."""
from __future__ import annotations

import math
import os
import numpy as np

from swarm_rescue.solutions.upgrade.controller import MyDroneRescueExample as V10
from swarm_rescue.solutions.upgrade.assignment import allocate_offers
from swarm_rescue.solutions.upgrade.global_tracking import GlobalTargetTracker
from swarm_rescue.solutions.upgrade.navigation import plan_occupancy_path
from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController


class MyDroneRescueV11(V10):
    _OFFER_TTL = 16

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._target_tracker = GlobalTargetTracker(int(self.identifier or 0))
        self._offers = {}
        self._offer_assignment = {}
        self._assignment_step = -1
        self._trajectory = PayloadTrajectoryController()
        self._pickup_geometry = None
        self._carried_offset = None
        self._use_trajectory = os.environ.get('SWARM_RESCUE_TRAJECTORY', '1') == '1'
        self._use_auction = os.environ.get('SWARM_RESCUE_AUCTION', '1') == '1'

    def _control_chase_bomb(self, cmd, bomb):
        detection = bomb[0]
        position, heading = self.measured_gps_position(), self.measured_compass_angle()
        if position is not None and heading is not None and detection.distance < 45:
            # Held bombs disappear from BOTH public ray sensors. Preserve the
            # final pre-grasp surface observation; never inspect bomb objects.
            centre = np.asarray(position)+(float(detection.distance)+12)*np.array([
                math.cos(heading+detection.angle), math.sin(heading+detection.angle)])
            self._pickup_geometry = tuple(centre), int(self.elapsed_timestep)
            self._carried_offset = None
        return super()._control_chase_bomb(cmd, bomb)

    def _control_step(self):
        if not self.grasped_bombs():
            self._carried_offset = None
        return super()._control_step()

    def _run_carry_exit(self, cmd, disposal_hit):
        if not self._use_trajectory:
            return super()._run_carry_exit(cmd, disposal_hit)
        if self._carry_exit_steps_left <= 0:
            return False
        if disposal_hit is not None and float(disposal_hit[0].distance) < 90:
            self._carry_exit_steps_left = 0
            return False
        pos, heading = self.measured_gps_position(), self.measured_compass_angle()
        if pos is None or heading is None:
            return super()._run_carry_exit(cmd, disposal_hit)
        # Back out along the actual approach trail before attempting a loaded
        # turn. The old fixed forward burst pushes edge bombs into their wall.
        goal = next((point for point in reversed(self._breadcrumbs)
                     if 40 <= math.dist(pos, point) <= 130), None)
        if goal is None:
            goal = tuple(pos-65*np.array([math.cos(heading), math.sin(heading)]))
        self._carry_exit_steps_left -= 1
        result = self._rollout_toward(*goal, cmd)
        cmd['grasper'] = 1
        return result is not None

    def _own_offer(self):
        pos = self.measured_gps_position()
        if pos is None or self.grasped_bombs() or self._retired_low_health:
            return {}
        point = tuple(map(float, pos))
        step = int(self.elapsed_timestep)
        entries = dict(self._discovered_targets)
        entries.update(self._recovery_targets)
        # No invented bid from another vehicle's location. Peers advertise what
        # they actually know and are willing to approach, including cooldowns.
        result = {}
        for token, (target, _) in entries.items():
            token = self._target_tracker.resolve(token)
            if (token in self._disposed_tokens or self._target_invalid(token)
                    or self._claim_cooldown_until.get(token, -1) > step):
                continue
            result[token] = self._target_bid(int(self.identifier or 0), point,
                                            int(self.drone_health), target)
        return result

    def define_message_for_all(self):
        message = super().define_message_for_all()
        message['task_offers_v11'] = tuple(sorted(self._own_offer().items()))
        return message

    def _merge_team_status(self):
        super()._merge_team_status()
        step = int(self.elapsed_timestep)
        for _, message in self.communicator.received_messages:
            if not isinstance(message, dict):
                continue
            owner, timestamp = message.get('id'), message.get('step')
            if (not isinstance(owner, int) or not 0 <= owner < self._team_size
                    or owner == int(self.identifier or 0) or not isinstance(timestamp, int)
                    or not 0 <= step-timestamp <= self._OFFER_TTL):
                continue
            entries = message.get('task_offers_v11')
            if not isinstance(entries, (tuple, list)):
                continue
            packet = {}
            for entry in entries[:64]:
                try:
                    raw, value = entry
                    token = self._target_tracker.resolve((int(raw[0]), int(raw[1])))
                    value = float(value)
                except (TypeError, ValueError, IndexError):
                    continue
                if math.isfinite(value) and 0 <= value <= 1_000_000:
                    packet[token] = min(value, packet.get(token, math.inf))
            old = self._offers.get(owner)
            if old is None or timestamp >= old[0]:
                self._offers[owner] = timestamp, packet
        self._assignment_step = -1

    def _refresh_assignment(self):
        step, my_id = int(self.elapsed_timestep), int(self.identifier or 0)
        if self._assignment_step == step:
            return
        busy, reserved = set(), set()
        for owner, (mode, token, _, seen) in self._team_tasks.items():
            if mode in ('claim', 'carry') and step-seen <= self._TASK_LEASE_STEPS:
                if mode == 'claim' and self._peer_claim_stalled(owner, token):
                    continue
                busy.add(owner)
                if token is not None:
                    reserved.add(self._target_tracker.resolve(token))
        own = self._own_offer()
        packets = {my_id: own} if my_id not in busy else {}
        for owner, (seen, packet) in self._offers.items():
            if owner not in busy and 0 <= step-seen <= self._OFFER_TTL:
                packets[owner] = packet
        packets = {owner: {self._target_tracker.resolve(token): value
                    for token, value in packet.items()
                    if self._target_tracker.resolve(token) not in reserved
                    and self._target_tracker.resolve(token) not in self._disposed_tokens}
                   for owner, packet in packets.items()}
        self._offer_assignment = allocate_offers(packets)
        self._assignment_step = step

    def _can_claim_target(self, token, point):
        if not self._use_auction:
            return super()._can_claim_target(token, point)
        step, my_id = int(self.elapsed_timestep), int(self.identifier or 0)
        if self._target_invalid(token):
            return False
        own = self._team_tasks.get(my_id)
        continuing = bool(own and own[0] == 'claim' and own[1] == token)
        own_started = self._claim_starts.get((my_id, token), own[3] if continuing else step)
        pos = self.measured_gps_position()
        if pos is None:
            return False
        for owner, (mode, other, other_point, seen) in self._team_tasks.items():
            if owner == my_id or step-seen > self._TASK_LEASE_STEPS:
                continue
            peer = self._peer_state.get(owner)
            near = bool(peer and step-peer[2] <= 12 and other_point is not None
                        and math.dist(other_point, point) < 45
                        and math.dist(peer[0], point) < 60 and math.dist(pos, point) < 85)
            loaded_near = bool(peer and mode == 'carry' and step-peer[2] <= 8
                               and peer[4] and math.dist(peer[0], point) < 55)
            if mode in ('claim', 'carry') and (other == token or near or loaded_near):
                if mode == 'claim' and self._peer_claim_stalled(owner, other):
                    continue
                started = self._claim_starts.get((owner, other), seen)
                if mode == 'carry' or not continuing or (started, owner) < (own_started, my_id):
                    return False
        if continuing:
            return True
        self._refresh_assignment()
        # An unknown/no-offer target remains locally actionable. Silence never
        # reserves it. Explicit offers expire and stable accepted tasks prevail.
        return self._offer_assignment.get(token, my_id) == my_id

    def _select_unclaimed_bomb(self):
        pos = self.measured_gps_position()
        if pos is None:
            return None
        for detection, point in sorted(self._visible_bombs, key=lambda item: (
                self._token_for_detection(*item) != self._current_bomb_token,
                self._target_bid(int(self.identifier or 0), tuple(map(float, pos)),
                                 int(self.drone_health), item[1]))):
            token = self._token_for_detection(detection, point)
            if (self._claim_cooldown_until.get(token, -1) <= int(self.elapsed_timestep)
                    and self._can_claim_target(token, point)):
                return detection, point
        return None

    def _plan_navigation(self, start, target):
        if not self._use_trajectory or not self.grasped_bombs():
            return super()._plan_navigation(start, target)
        # A 32px circle erases doors that a lengthwise two-disc load can cross.
        # The coarse route only constrains translation; rollouts check rotation.
        path = plan_occupancy_path(self.size_area, self._online_obstacles, start, target,
                                   clearance=20, cell_size=16)
        if not path:
            path = plan_occupancy_path(self.size_area, self._online_obstacles, start, target,
                                       clearance=18, cell_size=8)
        return path

    def _rollout_toward(self, x, y, cmd):
        pos, heading = self.measured_gps_position(), self.measured_compass_angle()
        ranges, angles = self.lidar_values(), self.lidar_rays_angles()
        if not self._use_trajectory or pos is None or heading is None or ranges is None or angles is None:
            return None
        c, s = math.cos(heading), math.sin(heading)
        transform = np.array([[c, s], [-s, c]])
        goal = transform @ (np.array([x, y])-pos)
        if np.linalg.norm(goal) < self._ARRIVE_RADIUS:
            self._set_motion(cmd, forward=0., lateral=0., rotation=0.)
            return True
        velocity = self.measured_velocity()
        velocity = transform @ velocity if velocity is not None else np.zeros(2)
        loaded = bool(self.grasped_bombs())
        distances = np.asarray(ranges, dtype=float).copy()
        angles = np.asarray(angles, dtype=float)
        offset = self._carried_offset
        if loaded and offset is None:
            geometry = self._pickup_geometry
            if geometry is not None and int(self.elapsed_timestep)-geometry[1] <= 8:
                estimate = transform @ (np.asarray(geometry[0])-pos)
                radius = float(np.linalg.norm(estimate))
                if 14 <= radius <= 40:
                    offset = estimate*min(30., max(20., radius))/radius
            if offset is None:
                offset = np.array([24., 0.])
            self._carried_offset = offset
        # Our load is invisible to our public ray sensors. Any visible grasped
        # bomb belongs to a peer: keep its LiDAR obstacle and never substitute
        # it for our pre-grasp footprint estimate.
        peers = []
        for owner, (point, _, seen, _, _, peer_velocity) in self._peer_state.items():
            if owner != int(self.identifier or 0) and int(self.elapsed_timestep)-seen <= 4:
                relative = transform @ (np.asarray(point)-pos)
                if np.linalg.norm(relative) < 100:
                    peers.append((relative, transform @ np.asarray(peer_velocity)))
        result = self._trajectory.command(goal, velocity, distances, angles,
                    loaded=loaded, load_offset=offset,
                    speed_limit=.45 if int(self.drone_health) <= self._LOW_HEALTH else 1., peers=peers)
        if result is None:
            return None
        cmd.update(result)
        if self._trajectory.last_safe:
            self._lidar_blocked_steps = 0
        else:
            self._lidar_blocked_steps += 1
            if self._lidar_blocked_steps >= 12:
                self._start_lidar_escape(self._lidar_turn_sign())
        return False

    def _move_toward(self, x, y, cmd, *, use_lidar=False, lidar_clearance=None):
        pos = self.measured_gps_position()
        if use_lidar and pos is not None and math.dist(pos, (x, y)) > 45:
            result = self._rollout_toward(x, y, cmd)
            if result is not None:
                return result
        return super()._move_toward(x, y, cmd, use_lidar=use_lidar, lidar_clearance=lidar_clearance)

    def _move_holonomic_toward(self, x, y, cmd):
        result = self._rollout_toward(x, y, cmd)
        return result if result is not None else super()._move_holonomic_toward(x, y, cmd)
