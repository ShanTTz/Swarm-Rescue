import math
from types import SimpleNamespace
import numpy as np
import pytest

from swarm_rescue.solutions.upgrade.controller_v13 import MyDroneRescueV13
from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
from swarm_rescue.solutions.upgrade.disposal_navigation import DisposalFootprint
from swarm_rescue.solutions.upgrade.navigation import plan_occupancy_path


@pytest.mark.parametrize('angle,offset', [(0., (0, 0)), (.65, (32, 48))])
def test_sensor_footprint_supports_rotated_translated_pad(angle, offset):
    footprint = DisposalFootprint((600, 600))
    for v in range(-64, 65, 16):
        for x, y in ((v, -64), (v, 64), (-64, v), (64, v)):
            footprint.add((x*math.cos(angle)-y*math.sin(angle)+offset[0],
                           x*math.sin(angle)+y*math.cos(angle)+offset[1]))
    footprint.rebuild()
    assert footprint.contains(offset)
    route = plan_occupancy_path((600, 600), footprint.mask, (-220, 0), (220, 0),
                                clearance=20, cell_size=8)
    assert route and not footprint.crosses((-220, 0), route)




def test_empty_overlay_leaves_carrier_path_and_wall_grid_unchanged(monkeypatch):
    obj = object.__new__(MyDroneRescueV13)
    obj._disposal_footprint = footprint = DisposalFootprint((600, 600))
    for point in [(-48, -48), (48, -48), (48, 48), (-48, 48)]: footprint.add(point)
    footprint.rebuild()
    obj._disposal_avoid_active = True
    grid = obj._online_obstacles = np.zeros((600, 600), np.uint8)
    monkeypatch.setattr(obj, 'grasped_bombs', lambda: [1])
    seen = []
    def route(start, goal):
        seen.append(bool(obj._online_obstacles[300, 300]))
        return [goal]
    monkeypatch.setattr(obj, '_plan_with_disposal_layer', route)
    assert obj._plan_navigation((-220, 0), (220, 0)) == [(220, 0)]
    assert seen == [False] and obj._online_obstacles is grid and not grid.any()


def test_loaded_route_keeps_narrow_passage_fallback(monkeypatch):
    import swarm_rescue.solutions.upgrade.controller_v13 as module
    obj = object.__new__(MyDroneRescueV13)
    obj._planner_backend = 'astar'
    obj._payload_wide_route = True
    obj.size_area = (600, 600)
    obj._online_obstacles = np.zeros((600, 600), np.uint8)
    obj._portal_enabled = False
    monkeypatch.setattr(obj, 'grasped_bombs', lambda: [1])
    calls = []
    def blocked(*args, **kw): calls.append(kw['clearance']); return []
    monkeypatch.setattr(module, 'plan_occupancy_path', blocked)
    monkeypatch.setattr(MyDroneRescueV12, '_plan_navigation', lambda self, start, goal: [goal])
    assert obj._plan_with_disposal_layer((-200, 0), (200, 0)) == [(200, 0)]
    assert calls == [32, 32]


def test_collision_hp_noise_does_not_reshuffle_capable_bidders(monkeypatch):
    obj = object.__new__(MyDroneRescueV13)
    obj._vacancy_policy = 'hard'
    monkeypatch.setattr(MyDroneRescueV12, '_target_bid',
                        lambda self, owner, position, health, point: 100+(50-health)*5)
    assert obj._target_bid(0, (0, 0), 45, (100, 0)) == obj._target_bid(0, (0, 0), 44, (100, 0))
    assert obj._target_bid(0, (0, 0), 25, (100, 0)) == obj._target_bid(0, (0, 0), 50, (100, 0))
    assert obj._target_bid(0, (0, 0), 15, (100, 0)) > obj._target_bid(0, (0, 0), 25, (100, 0))


@pytest.mark.parametrize('owner,budget,active', [(8, 2000, True), (9, 2000, True),
                                              (7, 2000, False), (8, 5000, False)])
def test_short_opening_scout_reserve_is_small_and_does_not_affect_long_missions(owner, budget, active):
    obj = object.__new__(MyDroneRescueV13)
    obj.identifier = owner
    obj._team_size = 10
    obj._max_steps = budget
    obj.elapsed_timestep = 100
    obj._patrol_index = 0
    assert obj._opening_scout_active() is active
    if active:
        assert obj._own_offer() == {}
        assert obj._can_claim_target((0, -1), (10, 10)) is False


@pytest.mark.parametrize('leg,step', [(3, 200), (0, 900)])
def test_scout_releases_on_coverage_progress_or_timeout_and_never_reenters(leg, step):
    obj = object.__new__(MyDroneRescueV13)
    obj.identifier = 8
    obj._team_size = 10
    obj._max_steps = 2000
    obj.elapsed_timestep = step
    obj._patrol_index = leg
    obj._diagnostic_event = lambda *a, **kw: None
    assert not obj._opening_scout_active()
    obj._patrol_index = 0
    assert not obj._opening_scout_active()


@pytest.mark.parametrize('position,grasped,released', [((400, 224), False, False),
                                                     ((520, -210), False, True),
                                                     ((520, -210), True, False)])
def test_shared_transit_point_does_not_end_the_other_scouts_sector(monkeypatch, position, grasped, released):
    from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
    obj = object.__new__(MyDroneRescueV13)
    obj.identifier = 9
    obj._team_size = 10
    obj._max_steps = 2000
    obj.elapsed_timestep = 250
    obj._patrol_index = 2
    obj._opening_scout_goal = (560, -335)
    obj._diagnostic_event = lambda *a, **kw: None
    monkeypatch.setattr(obj, 'measured_gps_position', lambda: np.asarray(position))
    monkeypatch.setattr(obj, 'semantic_values', lambda: [SimpleNamespace(
        entity_type=DroneSemanticSensor.TypeEntity.BOMB, grasped=grasped, distance=80)])
    assert obj._opening_scout_active() is (not released)
