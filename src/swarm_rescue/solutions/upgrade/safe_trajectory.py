"""Sensor-only swept footprints and full-horizon dynamic collision checks.

Fixed public vehicle geometry is bounded by a 16.5px disc. The caller supplies
the carried object's offset and radius; the selected balanced policy retains
v11's fixed-offset approximation. This bounded shooting controller is not MPPI.
"""
from __future__ import annotations

import math
import numpy as np


class SafeTrajectoryController:
    BODY_RADIUS = 16.5

    def __init__(self):
        self.previous = np.zeros(3)
        self.last_safe = False
        self.calls = 0
        self.rejections = 0
        self.feedback = False
        self.rotation_penalty = True

    def _predict(self, actions, velocity, obstacles, loaded, offset, load_radius, peers, goal):
        count = len(actions)
        position = np.zeros((count, 2))
        speed = np.tile(velocity, (count, 1))
        yaw = np.zeros(count)
        clear = np.full(count, 300.)
        initial = 300.
        if len(obstacles):
            initial = float(np.min(np.linalg.norm(obstacles, axis=1))-self.BODY_RADIUS)
            if loaded:
                initial = min(initial, float(np.min(np.linalg.norm(obstacles-offset, axis=1))-load_radius))
        collision = np.zeros(count, dtype=bool)
        impulse = .2 if loaded else .6
        horizon = (12 if loaded else 8) if self.feedback else 6
        terminal_clearance = np.full(count, 300.)
        offset_angle = math.atan2(offset[1], offset[0])
        for step in range(1, horizon+1):
            old_position, old_yaw = position.copy(), yaw.copy()
            predicted_action = actions
            if self.feedback and step > 3:
                delta = goal-position
                distance = np.linalg.norm(delta, axis=1)
                direction = delta/np.maximum(distance[:, None], 1e-8)
                target_speed = np.minimum(4. if loaded else 10., np.sqrt(2*impulse*np.maximum(distance-8., 0.)))
                required = (direction*target_speed[:, None]-.95*speed)/impulse
                required /= np.maximum(1., np.linalg.norm(required, axis=1))[:, None]
                c, s = np.cos(yaw), np.sin(yaw)
                thrust = np.column_stack((c*required[:, 0]+s*required[:, 1], -s*required[:, 0]+c*required[:, 1]))
                angle_error = np.arctan2(delta[:, 1], delta[:, 0])-yaw
                if loaded:
                    angle_error -= offset_angle
                    angle_error = .5*np.arctan2(np.sin(2*angle_error), np.cos(2*angle_error))
                else:
                    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                predicted_action = np.column_stack((thrust, np.clip(.8*angle_error, -.7, .7)))
            yaw += predicted_action[:, 2]*.18
            c, s = np.cos(yaw), np.sin(yaw)
            speed = .95*speed+impulse*np.column_stack((c*predicted_action[:, 0]-s*predicted_action[:, 1],
                                                     s*predicted_action[:, 0]+c*predicted_action[:, 1]))
            position += speed
            for fraction in (.5, 1.):
                centres = old_position+(position-old_position)*fraction
                angle = old_yaw+(yaw-old_yaw)*fraction
                lc, ls = np.cos(angle), np.sin(angle)
                load = centres+np.column_stack((lc*offset[0]-ls*offset[1],
                                                 ls*offset[0]+lc*offset[1]))
                if len(obstacles):
                    distances = np.linalg.norm(centres[:, None]-obstacles[None], axis=2)
                    clearance = distances.min(axis=1)-self.BODY_RADIUS
                    if loaded:
                        payload_clearance = np.linalg.norm(load[:, None]-obstacles[None], axis=2).min(axis=1)-load_radius
                        clearance = np.minimum(clearance, payload_clearance)
                    clear = np.minimum(clear, clearance)
                    terminal_clearance = clearance
                    # Existing sensing-margin overlap must permit escape, but
                    # new trajectories cannot deepen it by more than 0.5px.
                    collision |= clearance < min(-.5, initial-.5)
                for point, peer_velocity in peers:
                    point, peer_velocity = np.asarray(point), np.asarray(peer_velocity)
                    peer = point+(step-1+fraction)*peer_velocity
                    threshold = min(2*self.BODY_RADIUS, float(np.linalg.norm(point))-.5)
                    collision |= np.linalg.norm(centres-peer, axis=1) < threshold
                    if loaded:
                        initial_load = float(np.linalg.norm(offset-point))
                        collision |= np.linalg.norm(load-peer, axis=1) < min(load_radius+self.BODY_RADIUS, initial_load-.5)
        return position, speed, yaw, clear, collision, terminal_clearance

    def command(self, goal, velocity, ranges, angles, *, loaded=False,
                load_offset=None, load_radius=15., speed_limit=1., peers=(),
                preferred=None, validate_only=False):
        self.calls += 1
        goal = np.asarray(goal, dtype=float)
        velocity = np.asarray(velocity if velocity is not None else (0., 0.), dtype=float)
        ranges, angles = np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float)
        if (goal.shape != (2,) or velocity.shape != (2,) or ranges.ndim != 1
                or ranges.shape != angles.shape or not np.all(np.isfinite(goal))
                or not np.all(np.isfinite(velocity))):
            return None
        if not np.any(np.isfinite(ranges) & (ranges > 0)):
            return None
        valid = np.isfinite(ranges) & np.isfinite(angles) & (ranges > 0) & (ranges < 295)
        obstacles = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
        obstacles = obstacles[np.linalg.norm(obstacles, axis=1) < 135]
        offset = np.asarray(load_offset if load_offset is not None else (24., 0.), dtype=float)
        if offset.shape != (2,) or not np.all(np.isfinite(offset)):
            return None
        if preferred is not None:
            preferred = np.asarray(preferred, dtype=float)
            if preferred.shape != (3,) or not np.all(np.isfinite(preferred)):
                return None
        brake = -velocity/max(1., np.linalg.norm(velocity))
        desired = math.atan2(goal[1], goal[0])
        if validate_only:
            actions = preferred.reshape(1, 3)
        else:
            offsets = [0., -.6, .6, -1.4, 1.4, math.pi] if self.feedback else [0., -.35, .35, -.8, .8, -1.4, 1.4, math.pi]
            directions = desired+np.asarray(offsets)
            thrusts = np.concatenate([np.column_stack((np.cos(directions), np.sin(directions)))*scale
                                     for scale in (speed_limit, .45*speed_limit)])
            thrusts = np.vstack((thrusts, brake, [0., 0.]))
            turn = np.clip(desired*.8, -.7, .7)
            if self.feedback and loaded:
                cargo_error = desired-math.atan2(offset[1], offset[0])
                cargo_error = .5*math.atan2(math.sin(2*cargo_error), math.cos(2*cargo_error))
                turn = np.clip(cargo_error*.8, -.7, .7)
            actions = np.column_stack((np.repeat(thrusts, 3, axis=0), np.tile([turn, 0., -turn], len(thrusts))))
            if preferred is not None:
                actions = np.vstack((preferred, actions))
        position, terminal_speed, yaw, clearance, collision, terminal_clearance = self._predict(
            actions, velocity, obstacles, loaded, offset, load_radius, peers, goal)
        cost = np.linalg.norm(position-goal, axis=1)+4*np.exp(-np.maximum(clearance, 0)/8.)
        cost += .8*np.linalg.norm(actions-self.previous, axis=1)
        cost += .4*np.abs(np.arctan2(np.sin(desired-yaw), np.cos(desired-yaw)))
        if self.rotation_penalty:
            cost += (.6 if loaded else .25)*np.abs(actions[:, 2])
        if self.feedback:
            remaining = goal-position
            distance = np.linalg.norm(remaining, axis=1)
            impulse = .2 if loaded else .6
            target_speed = np.minimum(4. if loaded else 10., np.sqrt(2*impulse*np.maximum(distance-8., 0.)))
            desired_velocity = remaining/np.maximum(distance[:, None], 1e-8)*target_speed[:, None]
            cost += .8*np.linalg.norm(terminal_speed-desired_velocity, axis=1)
            cost += .7*np.maximum(-terminal_clearance, 0.)
            if loaded:
                axis_error = desired-yaw-math.atan2(offset[1], offset[0])
                cost += .8*np.abs(.5*np.arctan2(np.sin(2*axis_error), np.cos(2*axis_error)))
        if preferred is not None:
            cost += 2*np.linalg.norm(actions-preferred, axis=1)
        cost[collision] = np.inf
        self.last_safe = bool(np.any(np.isfinite(cost)))
        if validate_only and not self.last_safe:
            return None
        chosen = actions[int(np.argmin(cost))] if self.last_safe else np.array([*brake, 0.])
        if not self.last_safe:
            self.rejections += 1
        self.previous = chosen.copy()
        return dict(forward=float(chosen[0]), lateral=float(chosen[1]), rotation=float(chosen[2]))


class SequenceTrajectoryController(SafeTrajectoryController):
    """Receding-horizon policy shooting with future braking and cargo alignment.

The first three controls are lattice candidates; later controls respond to
predicted pose/velocity. Only the first command is executed before replanning.
42 policy rollouts keep cost bounded, with 12 loaded / 8 empty prediction steps.
"""
    def __init__(self):
        super().__init__()
        self.feedback = True
