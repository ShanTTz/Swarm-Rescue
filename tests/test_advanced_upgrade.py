"""v11 counterexamples: greedy association, silent bidders and swept loads."""
import itertools
import math
from types import SimpleNamespace

import numpy as np
import pytest

from swarm_rescue.solutions.upgrade.assignment import minimum_assignment, allocate_offers
from swarm_rescue.solutions.upgrade.global_tracking import GlobalTargetTracker
from swarm_rescue.solutions.upgrade.target_tracking import Observation
from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
from swarm_rescue.solutions.upgrade.controller_v11 import MyDroneRescueV11


def test_assignment_matches_brute_force_small_rectangles():
    rng = np.random.default_rng(20261006)
    for n, m in ((1, 3), (2, 4), (3, 5), (4, 4)):
        for _ in range(12):
            costs = rng.integers(-3, 50, size=(n, m)).tolist()
            selected = minimum_assignment(costs)
            actual = sum(costs[i][col] for i, col in enumerate(selected))
            expected = min(sum(costs[i][col] for i, col in enumerate(cols))
                           for cols in itertools.permutations(range(m), n))
            assert len(set(selected)) == n and actual == expected


def test_assignment_rejects_impossible_edges_and_accepts_empty_input():
    assert minimum_assignment([]) == []
    assert minimum_assignment([[math.inf, 20.]]) == [1]
    with pytest.raises(ValueError):
        minimum_assignment([[math.inf]])


def test_global_tracking_avoids_greedy_stealing_only_feasible_track():
    tracker = GlobalTargetTracker(0)
    left, right = tracker.observe([Observation((0, 0), 0), Observation((30, 0), 0)], 0)
    # Greedy picks x=13 -> left, then x=-16 cannot match right. Joint matching
    # assigns x=13 -> right and keeps both existing identities.
    selected = tracker.observe([Observation((13, 0), 0), Observation((-16, 0), 0)], 1)
    assert selected == [right, left]
    assert len(tracker.tracks) == 2
    tracker.retire([left, right])
    assert set(tracker.observe([Observation((13, 0), 0)], 2)).isdisjoint({left, right})


def test_team_matching_allocates_alternative_instead_of_everyone_waiting():
    a, b = (0, -1), (0, -2)
    # Independent per-target min bids would give both tasks to drone 0.
    assert allocate_offers({0: {a: 1., b: 2.}, 1: {a: 2., b: 100.}}) == {a: 1, b: 0}
    assert allocate_offers({0: {a: 20.}, 1: {}}) == {a: 0}
    assert allocate_offers({0: {a: 1.}, 1: {a: 1.}}) == {a: 0}


def test_expired_offer_and_busy_owner_do_not_reserve_new_tasks():
    token = (0, -1)
    obj = SimpleNamespace(identifier=0, elapsed_timestep=100, _assignment_step=-1,
        _TASK_LEASE_STEPS=180, _OFFER_TTL=16, _team_tasks={}, _disposed_tokens=set(),
        _offers={1: (80, {token: 1.})}, _own_offer=lambda: {token: 20.},
        _peer_claim_stalled=lambda *_: False, _target_tracker=GlobalTargetTracker(0))
    MyDroneRescueV11._refresh_assignment(obj)
    assert obj._offer_assignment == {token: 0}
    obj._offers[1] = (100, {token: 1.})
    obj._team_tasks[1] = ('carry', (1, -2), (30, 0), 100)
    obj._assignment_step = -1
    MyDroneRescueV11._refresh_assignment(obj)
    assert obj._offer_assignment == {token: 0}


def _scan(points):
    points = np.asarray(points, dtype=float)
    return np.linalg.norm(points, axis=1), np.arctan2(points[:, 1], points[:, 0])


def test_payload_rollout_passes_lengthwise_door_without_fixed_circle_inflation():
    planner = PayloadTrajectoryController()
    points = [(x, y) for x in range(-60, 121, 3) for y in (-22, 22)]
    ranges, angles = _scan(points)
    cmd = planner.command((120, 0), (1, 0), ranges, angles,
                          loaded=True, load_offset=(24, 0))
    assert planner.last_safe and cmd['forward'] > .4
    assert abs(cmd['lateral']) < .1 and abs(cmd['rotation']) < .1


def test_swept_payload_rejects_turn_that_body_alone_can_make():
    points = [(x, 23) for x in range(-40, 100, 2)]
    ranges, angles = _scan(points)
    body, loaded = PayloadTrajectoryController(), PayloadTrajectoryController()
    a = body.command((0, 100), (0, 0), ranges, angles)
    b = loaded.command((0, 100), (0, 0), ranges, angles, loaded=True, load_offset=(24, 0))
    assert a['rotation'] > 0
    assert loaded.last_safe and b['rotation'] <= 0


def test_rollout_brakes_inertial_wall_approach_and_handles_disabled_lidar():
    planner = PayloadTrajectoryController()
    ranges, angles = _scan([(25, y) for y in range(-50, 51, 2)])
    cmd = planner.command((100, 0), (6, 0), ranges, angles)
    assert cmd['forward'] < 0
    assert planner.command((100, 0), (0, 0), [math.nan], [0]) is None


def test_v11_entry_is_explicit_and_red_mode_stays_unchanged(monkeypatch):
    from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode
    monkeypatch.setenv('SWARM_RESCUE_ALGORITHM', 'v11')
    assert drone_class_for_mode('rescue') is MyDroneRescueV11
    assert drone_class_for_mode('place').__module__.endswith('my_drone_place_example')


def test_edge_pickup_exits_along_approach_trail_instead_of_forward_into_wall():
    commands = []
    obj = SimpleNamespace(_use_trajectory=True, _carry_exit_steps_left=18,
        measured_gps_position=lambda: np.array([100., 0.]), measured_compass_angle=lambda: 0.,
        _breadcrumbs=[(0., 0.), (45., 0.), (85., 0.)],
        _rollout_toward=lambda x, y, cmd: commands.append((x, y)) or False)
    cmd = {'grasper': 0}
    assert MyDroneRescueV11._run_carry_exit(obj, cmd, None)
    assert commands == [(45., 0.)] and cmd['grasper'] == 1


def test_all_sixteen_archived_layouts_keep_ten_fixed_bombs():
    import hashlib
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]/'fixtures/hard_layouts'
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['simulation_seeds'] == [20261005, 314159]
    assert len(manifest['fixed_layouts']) == 16
    for entry in manifest['fixed_layouts']:
        raw = (root/entry['file']).read_bytes()
        document = json.loads(raw)
        assert hashlib.sha256(raw).hexdigest() == entry['sha256']
        assert len(document['bombs']) == 10 and not document['zones_config']


def test_peer_carried_bomb_stays_obstacle_and_never_replaces_own_footprint():
    from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
    captured = {}
    def predict(goal, velocity, ranges, angles, **kwargs):
        captured.update(kwargs, ranges=np.asarray(ranges).copy())
        return dict(forward=.4, lateral=0., rotation=0.)
    obj = SimpleNamespace(_use_trajectory=True, _ARRIVE_RADIUS=10, _LOW_HEALTH=10,
        identifier=0, elapsed_timestep=100, drone_health=50,
        measured_gps_position=lambda: np.zeros(2), measured_compass_angle=lambda: 0.,
        measured_velocity=lambda: np.zeros(2), grasped_bombs=lambda: [object()],
        lidar_values=lambda: np.array([60., 300.]), lidar_rays_angles=lambda: np.array([0., math.pi]),
        semantic_values=lambda: [SimpleNamespace(entity_type=DroneSemanticSensor.TypeEntity.BOMB,
                                                 grasped=True, distance=60., angle=0.)],
        _carried_offset=np.array([24., 0.]), _pickup_geometry=None, _peer_state={},
        _trajectory=SimpleNamespace(command=predict, last_safe=True), _lidar_blocked_steps=0)
    assert MyDroneRescueV11._rollout_toward(obj, 100., 0., {}) is False
    assert captured['ranges'][0] == 60.
    assert np.array_equal(captured['load_offset'], [24., 0.])
