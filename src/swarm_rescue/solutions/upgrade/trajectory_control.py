"""Bounded receding-horizon control with an observed two-disc payload footprint.

Uses body-frame LiDAR endpoints and odometry velocity only. This deterministic
shooting controller borrows trajectory critics from MPPI, but is not MPPI and
does not provide a collision guarantee with noisy, sparse or disabled sensors.
"""
from __future__ import annotations

import math
import numpy as np


class PayloadTrajectoryController:
    def __init__(self):
        self.previous = np.zeros(3)
        self.last_safe = False
        self.calls = 0
        self.peer_sweep = False
        self.peer_soft = False

    def command(self, goal, velocity, ranges, angles, *, loaded=False,
                load_offset=None, speed_limit=1., peers=()):
        """Goal, velocity, load centre and peers are all in current body frame.

        Velocity uses pixels per simulation step. The engine advances one
        physics time unit per control step (not 1/30). Forces are applied once
        before its ten microsteps: effective impulse is approximately .6/.2
        px/step², not the continuous thrust value 6/2.
        """
        self.calls += 1
        goal = np.asarray(goal, dtype=float)
        velocity = np.asarray(velocity if velocity is not None else (0., 0.), dtype=float)
        ranges, angles = np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float)
        if (goal.shape != (2,) or velocity.shape != (2,) or ranges.ndim != 1
                or ranges.shape != angles.shape or not np.all(np.isfinite(goal))
                or not np.all(np.isfinite(velocity))):
            return None
        valid = np.isfinite(ranges) & np.isfinite(angles) & (ranges > 0.) & (ranges < 295.)
        obstacles = np.column_stack((ranges[valid]*np.cos(angles[valid]),
                                     ranges[valid]*np.sin(angles[valid])))
        if not np.any(np.isfinite(ranges) & (ranges > 0.)):
            return None
        # Distance-limited endpoint cloud preserves every close ray.
        obstacles = obstacles[np.linalg.norm(obstacles, axis=1) < 135.]
        offset = np.asarray(load_offset if load_offset is not None else (24., 0.), dtype=float)
        if offset.shape != (2,) or not np.all(np.isfinite(offset)):
            offset = np.array([24., 0.])
        desired = math.atan2(goal[1], goal[0])
        directions = desired + np.array([0., -.35, .35, -.8, .8, -1.4, 1.4, math.pi])
        thrusts = np.concatenate([np.column_stack((np.cos(directions), np.sin(directions)))*scale
                                 for scale in (speed_limit, .45*speed_limit)], axis=0)
        brake = -velocity/max(1., np.linalg.norm(velocity))
        thrusts = np.vstack((thrusts, brake, [0., 0.]))
        # Three angular actions per translation: align, keep orientation, turn
        # away. A bomb beside a wall may forbid rotation but allow lateral exit.
        turn = np.clip(desired*.8, -.7, .7)
        actions = np.column_stack((np.repeat(thrusts, 3, axis=0),
                                   np.tile([turn, 0., -turn], len(thrusts))))
        count = len(actions)
        position, speed, yaw = np.zeros((count, 2)), np.tile(velocity, (count, 1)), np.zeros(count)
        min_clear = np.full(count, 300.)
        initial = 300.
        radius = 13.  # body radius 10 + local sensing tolerance
        load_radius = 15.
        if len(obstacles):
            initial = float(np.min(np.linalg.norm(obstacles, axis=1)-radius))
            if loaded:
                initial = min(initial, float(np.min(np.linalg.norm(obstacles-offset, axis=1)-load_radius)))
        collision = np.zeros(count, dtype=bool)
        # Six one-step predictions, plus midpoints to bound corner/rotation
        # tunnelling. Evaluate only a 54-action batch, independent of map size.
        impulse = .2 if loaded else .6
        previous_position = position.copy()
        previous_yaw = yaw.copy()
        sweep_peers = bool(self.peer_sweep and peers)
        peer_risk = np.zeros(count)
        if sweep_peers:
            peer_points = np.asarray([point for point, _ in peers], dtype=float)
            peer_velocities = np.asarray([speed for _, speed in peers], dtype=float)
            peer_limits = np.minimum(23., np.linalg.norm(peer_points, axis=1)-.5)
            load_limits = np.minimum(load_radius+radius, np.linalg.norm(offset-peer_points, axis=1)-.5)
        for prediction_step in range(6):
            yaw += actions[:, 2]*.18
            c, s = np.cos(yaw), np.sin(yaw)
            acceleration = np.column_stack((c*actions[:, 0]-s*actions[:, 1],
                                             s*actions[:, 0]+c*actions[:, 1]))*impulse
            speed = .95*speed+acceleration
            position += speed
            for fraction in (.5, 1.):
                centres = previous_position+(position-previous_position)*fraction
                angle = previous_yaw+(yaw-previous_yaw)*fraction
                if len(obstacles):
                    clearance = np.min(np.linalg.norm(centres[:, None, :]-obstacles[None, :, :], axis=2), axis=1)-radius
                    if loaded:
                        lc, ls = np.cos(angle), np.sin(angle)
                        load = centres+np.column_stack((lc*offset[0]-ls*offset[1], ls*offset[0]+lc*offset[1]))
                        clearance = np.minimum(clearance, np.min(np.linalg.norm(load[:, None, :]-obstacles[None, :, :], axis=2), axis=1)-load_radius)
                    min_clear = np.minimum(min_clear, clearance)
                    # If already overlapping the uncertainty margin, permit
                    # escape that improves separation; don't freeze all actions.
                    collision |= clearance < min(-.5, initial-1.)
                if sweep_peers:
                    peer_positions = peer_points+(prediction_step+fraction)*peer_velocities
                    # Preserve the legacy separation threshold while checking
                    # the whole rollout. Permit escape from initial overlap.
                    peer_distances = np.linalg.norm(centres[:, None]-peer_positions[None], axis=2)
                    if self.peer_soft:
                        risk = np.maximum(0., peer_limits-peer_distances)/23.
                        peer_risk = np.maximum(peer_risk, np.max(risk*risk, axis=1))
                    else:
                        collision |= np.any(peer_distances < peer_limits, axis=1)
                    if loaded:
                        lc, ls = np.cos(angle), np.sin(angle)
                        load = centres+np.column_stack((lc*offset[0]-ls*offset[1], ls*offset[0]+lc*offset[1]))
                        load_distances = np.linalg.norm(load[:, None]-peer_positions[None], axis=2)
                        if self.peer_soft:
                            risk = np.maximum(0., load_limits-load_distances)/(load_radius+radius)
                            peer_risk = np.maximum(peer_risk, np.max(risk*risk, axis=1))
                        else:
                            collision |= np.any(load_distances < load_limits, axis=1)
            previous_position, previous_yaw = position.copy(), yaw.copy()
        if not self.peer_sweep or self.peer_soft:
            for point, peer_velocity in peers:
                peer_end = np.asarray(point)+6*np.asarray(peer_velocity)
                collision |= np.linalg.norm(position-peer_end, axis=1) < 23.
        remaining = np.linalg.norm(position-goal, axis=1)
        # Clearance penalty decays rapidly; wide fixed safety bands would close
        # physically usable doors. Negative clearance has a hard rejection.
        cost = remaining+4*np.exp(-np.maximum(min_clear, 0.)/8.)
        cost += .8*np.linalg.norm(actions-self.previous, axis=1)
        cost += .4*np.abs(np.arctan2(np.sin(desired-yaw), np.cos(desired-yaw)))
        if self.peer_soft:
            cost += 20.*peer_risk
        cost[collision] = np.inf
        self.last_safe = bool(np.any(np.isfinite(cost)))
        if self.last_safe:
            chosen = actions[int(np.argmin(cost))]
        else:
            chosen = np.array([*brake, 0.])
        self.previous = chosen.copy()
        return dict(forward=float(chosen[0]), lateral=float(chosen[1]), rotation=float(chosen[2]))
