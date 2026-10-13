import math
import numpy as np

from swarm_rescue.solutions.upgrade.controller_v13 import MyDroneRescueV13
from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
from swarm_rescue.solutions.upgrade.global_tracking import GlobalTargetTracker


def fixture(observed=50):
    token, carried = (0, -1000001), (1, -1000001)
    obj = object.__new__(MyDroneRescueV13)
    obj._target_tracker = GlobalTargetTracker(0)
    obj._discovered_targets = {token: ((3., 2.), observed)}
    obj._recovery_targets = {}
    obj._invalid_targets = {}
    obj._disposed_tokens = set()
    obj._vacated_targets = {carried: ((0., 0.), 80)}
    obj._vacancy_enabled = True
    obj._block_history_enabled = True
    obj._vacancy_claim_guard = True
    obj._vacancy_policy = 'hard'
    obj._disposal_vacancy_only = False
    obj._vacancy_radius = 8.
    obj._vacancy_decisions = set()
    obj._team_tasks = {}
    obj.identifier = 0
    obj._diagnostics_enabled = False
    obj.elapsed_timestep = 100
    return obj, token, carried


def test_picked_up_alias_is_suppressed_but_new_observation_reactivates_drop():
    obj, token, _ = fixture()
    assert obj._target_invalid(token)
    obj._discovered_targets[token] = ((3., 2.), 81)
    assert not obj._target_invalid(token)


def test_coobserved_close_neighbour_remains_available():
    obj, token, carried = fixture()
    obj._target_tracker.separated.add(frozenset((token, carried)))
    assert not obj._target_invalid(token)


def test_fresh_recovery_beats_old_discovery_and_remote_times_do_not_refresh():
    obj, token, carried = fixture()
    obj._recovery_targets[token] = ((3., 2.), 90)
    assert not obj._target_invalid(token)
    obj._remember_vacancy(carried, (99., 99.), 70)
    assert obj._vacated_targets[carried] == ((0., 0.), 80)


def test_motion_and_grasping_stay_on_validated_baseline():
    for name in ('_rollout_toward', '_move_toward', '_should_enable_grasper'):
        assert getattr(MyDroneRescueV13, name) is getattr(MyDroneRescueV12, name)
    obj, _, _ = fixture()
    obj._lidar_blocked_steps = 5
    obj._reset_explore_state()
    assert obj._lidar_blocked_steps == 5


def test_peer_pickup_cannot_revoke_independent_existing_claim():
    obj, token, carried = fixture()
    # No co-observation constraint: proximity alone cannot establish identity.
    obj._team_tasks[0] = ('claim', token, (3., 2.), 99)
    assert not obj._target_invalid(token)
    obj._team_tasks[0] = ('search', None, None, 100)
    assert obj._target_invalid(token)


def test_claim_guard_does_not_override_identity_or_negative_scan():
    obj, token, carried = fixture()
    obj._team_tasks[0] = ('claim', token, (3., 2.), 99)
    obj._invalid_targets[token] = 90
    assert obj._target_invalid(token)
    obj._invalid_targets.clear()
    obj._vacated_targets = {token: ((0., 0.), 80)}
    assert obj._target_invalid(token)


def test_block_history_ablation_restores_legacy_reset():
    obj, _, _ = fixture()
    obj._block_history_enabled = False
    obj._lidar_blocked_steps = 5
    obj._reset_explore_state()
    assert obj._lidar_blocked_steps == 0


def test_uncertain_spatial_alias_is_penalized_and_remains_actionable():
    obj, token, carried = fixture()
    obj._vacancy_policy = 'soft'
    cost = MyDroneRescueV12._target_bid(obj, 0, (40., 2.), 50, (3., 2.))
    assert not obj._target_invalid(token)
    assert obj._target_bid(0, (40., 2.), 50, (3., 2.)) == cost + 150.
    obj._team_tasks[0] = ('claim', token, (3., 2.), 99)
    assert obj._target_bid(0, (40., 2.), 50, (3., 2.)) == cost
    obj._team_tasks.clear()
    obj._discovered_targets[token] = ((3., 2.), 81)
    assert obj._target_bid(0, (40., 2.), 50, (3., 2.)) == cost


def test_soft_policy_prioritizes_explicit_identity_over_nearby_alias():
    obj, token, carried = fixture()
    obj._vacancy_policy = 'soft'
    obj._vacated_targets[token] = ((3., 2.), 80)
    assert obj._target_invalid(token)


def test_spatial_exclusion_waits_for_real_disposal_confirmation():
    obj, token, carried = fixture()
    obj._disposal_vacancy_only = True
    assert not obj._target_invalid(token)
    obj._disposed_tokens.add(carried)
    assert obj._target_invalid(token)
    obj._discovered_targets[token] = ((3., 2.), 81)
    assert not obj._target_invalid(token)


def test_conservative_pickup_radius_keeps_the_replay_neighbour_available():
    obj, token, carried = fixture(observed=461)
    obj.elapsed_timestep = 471
    obj._discovered_targets[token] = ((-557.5638818241749, -558.6678062146412), 461)
    obj._vacated_targets[carried] = ((-547.0448871747558, -561.1351752066616), 469)
    assert not obj._target_invalid(token)
    obj._vacancy_radius = 18.
    assert obj._target_invalid(token)


def test_endgame_uses_attached_carriers_and_releases_on_drop_stale_radio_or_stall():
    obj, token, carried = fixture()
    obj._endgame_enabled = True
    obj._EXPECTED_BOMBS = 10
    obj._team_deliveries = {0: 9.}
    obj._team_tasks = {1: ('carry', carried, (0., 0.), 100)}
    obj._peer_state = {1: ((0., 0.), 40, 100, 0., True, (1., 0.))}
    obj._team_progress = {1: ('carry', carried, 50, 98, 100)}
    obj._task_has_progress = lambda *_: True
    assert obj._all_remaining_attached()  # Unresolved ghost token need not stop standby.
    obj._peer_state[1] = ((0., 0.), 40, 100, 0., False, (1., 0.))
    assert not obj._all_remaining_attached()
    obj._peer_state[1] = ((0., 0.), 40, 95, 0., True, (1., 0.))
    assert not obj._all_remaining_attached()
    obj._peer_state[1] = ((0., 0.), 40, 100, 0., True, (1., 0.))
    obj._team_progress[1] = ('carry', carried, 0, 0, 100)
    obj.elapsed_timestep = 121
    obj._team_tasks[1] = ('carry', carried, (0., 0.), 121)
    obj._peer_state[1] = ((0., 0.), 40, 121, 0., True, (1., 0.))
    assert not obj._all_remaining_attached()
    obj.elapsed_timestep = 100
    obj._team_progress[1] = ('carry', carried, 50, 98, 100)
    obj._team_tasks[1] = ('carry', carried, (0., 0.), 100)
    obj._peer_state[1] = ((0., 0.), 40, 100, 0., True, (1., 0.))
    obj._team_deliveries[0] = 8.
    assert not obj._all_remaining_attached()
    obj._team_deliveries[0] = 9.
    obj._disposed_tokens.add(carried)  # Do not count an already disposed load twice.
    assert not obj._all_remaining_attached()


def test_fine_portal_route_survives_max_pool_erasure_and_rebuilds_on_new_wall():
    from swarm_rescue.solutions.upgrade.portal_navigation import PortalApproachPlanner
    from swarm_rescue.solutions.upgrade.navigation import _line_is_free, plan_occupancy_path
    obstacles = np.zeros((240, 240), dtype=np.uint8)
    obstacles[118:122] = 1
    obstacles[118:122, 94:146] = 0
    planner = PortalApproachPlanner((240, 240), clearance=20)
    planner.refresh(obstacles, 1)
    assert not plan_occupancy_path((240,240),obstacles,(0,80),(0,-80),clearance=20,cell_size=16)
    route = planner.plan((0,80), (0,-80), max_projection=24)
    assert route and math.dist(route[-1],(0,-80)) < 1
    assert all(_line_is_free(a,b,planner.size,planner.blocked) for a,b in zip(route,route[1:]))
    planner.refresh(obstacles, 2)
    assert planner.rebuilds == 1
    obstacles[118:122] = 1
    planner.refresh(obstacles, 2)
    assert planner.rebuilds == 2
    assert not planner.plan((0,80),(0,-80),max_projection=24)


def test_projection_cannot_accept_target_across_a_wall_outside_tolerance():
    from swarm_rescue.solutions.upgrade.portal_navigation import PortalApproachPlanner
    obstacles = np.zeros((240,240),dtype=np.uint8)
    obstacles[:,118:122] = 1
    planner = PortalApproachPlanner((240,240),clearance=20)
    planner.refresh(obstacles,0)
    assert not planner.plan((-60,0),(60,0),max_projection=24)


def test_portal_repair_keeps_loaded_and_successful_routes(monkeypatch):
    obj, _, _ = fixture()
    obj._portal_enabled = True
    obj._portal_planners = {}
    obj._operational_stage = 'approach_shared'
    obj.grasped_bombs = lambda: [object()]
    legacy = [(20., 0.)]
    monkeypatch.setattr(MyDroneRescueV12, '_plan_navigation', lambda *_: legacy)
    assert obj._plan_navigation((0., 0.), (100., 0.)) is legacy
    assert not obj._portal_planners
    obj.grasped_bombs = lambda: []
    legacy[:] = [(100., 0.)]
    assert obj._plan_navigation((0., 0.), (100., 0.)) is legacy
    assert not obj._portal_planners


def test_portal_recovery_waits_for_no_progress_and_keeps_the_accepted_task(monkeypatch):
    from types import SimpleNamespace
    obj, token, _ = fixture()
    obj._portal_enabled = obj._portal_progress_gate = True
    obj._portal_recovery_tokens = set()
    obj._portal_planners = {}
    obj._operational_stage = 'approach_shared'
    obj._current_bomb_token = token
    obj._task_monitor = SimpleNamespace(current=SimpleNamespace(mode='claim', token=token, advanced=20))
    obj.grasped_bombs = lambda: []
    obj.size_area = (240, 240)
    obj._online_obstacles = np.zeros((240, 240), dtype=np.uint8)
    monkeypatch.setattr(MyDroneRescueV12, '_plan_navigation', lambda *_: [])
    assert not obj._plan_navigation((0., 80.), (0., -80.))
    assert not obj._portal_planners
    obj.elapsed_timestep = 140
    assert obj._plan_navigation((0., 80.), (0., -80.))
    obj._task_monitor.current.advanced = 140
    obj.elapsed_timestep = 141
    assert obj._plan_navigation((0., 80.), (0., -80.))


def test_empty_navigation_reaches_corner_before_switching_and_restores_tolerance():
    obj, _, _ = fixture()
    obj._corner_guard_enabled = True
    obj._nav_target = (100., -100.)
    obj._nav_age = 0
    obj._nav_path = [(0., 0.), (100., -100.)]
    obj._nav_plan_failed = obj._nav_goal_projected = False
    obj.measured_gps_position = lambda: (-16., 0.)
    obj.grasped_bombs = lambda: []
    obj._run_lidar_escape_if_needed = lambda *_: False
    moves = []
    obj._move_toward = lambda x, y, *args, **kwargs: moves.append((x, y))
    original = obj._PATROL_ARRIVE_RADIUS
    assert obj._ARRIVE_RADIUS < 12.
    obj._navigate_online((100., -100.), {})
    assert moves[-1] == (0., 0.)
    assert obj._PATROL_ARRIVE_RADIUS == original
    obj._corner_guard_enabled = False
    obj._navigate_online((100., -100.), {})
    assert moves[-1] == (100., -100.)


def test_pickup_hint_only_shortens_recovery_after_measured_stagnation():
    from swarm_rescue.solutions.task_progress import TaskProgressMonitor
    obj, token, _ = fixture()
    obj._vacancy_policy = 'progress'
    obj._max_steps = 7200
    obj._task_monitor = TaskProgressMonitor()
    obj._task_monitor.update('claim', token, 50, (0., 0.), 100.)
    obj.elapsed_timestep = 169
    assert not obj._target_invalid(token)
    assert not obj._own_task_stalled('claim', token)
    obj.elapsed_timestep = 170
    assert obj._own_task_stalled('claim', token)
    obj._discovered_targets[token] = ((3., 2.), 81)
    assert not obj._own_task_stalled('claim', token)
    obj._task_monitor.update('carry', token, 50, (0., 0.), 100.)
    assert not obj._own_task_stalled('carry', token)


def test_peer_crossing_between_prediction_endpoints_is_rejected():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    rays, angles = np.full(24, 300.), np.linspace(-math.pi, math.pi, 24)
    peers = [(np.array([3.5, 30.]), np.array([0., -10.]))]
    legacy, swept = PayloadTrajectoryController(), PayloadTrajectoryController()
    swept.peer_sweep = True
    legacy.command((40., 0.), (0., 0.), rays, angles, peers=peers)
    swept.command((40., 0.), (0., 0.), rays, angles, peers=peers)
    assert legacy.last_safe
    assert not swept.last_safe


def test_peer_sweep_preserves_commands_without_nearby_peers():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    legacy, swept = PayloadTrajectoryController(), PayloadTrajectoryController()
    swept.peer_sweep = True
    rays, angles = np.full(32, 100.), np.linspace(-math.pi, math.pi, 32)
    for loaded in (False, True):
        assert legacy.command((60., 25.), (1., 0.), rays, angles, loaded=loaded) == swept.command(
            (60., 25.), (1., 0.), rays, angles, loaded=loaded)
        assert legacy.last_safe == swept.last_safe


def test_peer_sweep_allows_escape_from_existing_overlap():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    controller = PayloadTrajectoryController()
    controller.peer_sweep = True
    cmd = controller.command((-40., 0.), (0., 0.), np.full(24, 300.),
        np.linspace(-math.pi, math.pi, 24), peers=[(np.array([10., 0.]), np.zeros(2))])
    assert controller.last_safe and cmd['forward'] < 0


def test_future_peer_crossing_risk_does_not_freeze_all_actions():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    controller = PayloadTrajectoryController()
    controller.peer_sweep = controller.peer_soft = True
    command = controller.command((40., 0.), (0., 0.), np.full(24, 300.),
        np.linspace(-math.pi, math.pi, 24), peers=[(np.array([3.5, 30.]), np.array([0., -10.]))])
    assert controller.last_safe
    assert all(np.isfinite(value) and -1 <= value <= 1 for value in command.values())
    assert math.hypot(command['forward'], command['lateral']) > 0


def test_soft_peer_risk_retains_legacy_terminal_collision_rejection():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    controller = PayloadTrajectoryController()
    controller.peer_sweep = controller.peer_soft = True
    controller.command((40., 0.), (0., 0.), np.full(24, 300.), np.linspace(-math.pi, math.pi, 24),
        peers=[(np.array([0., 35.]), np.array([0., -35./6]))])
    assert not controller.last_safe


def test_soft_peer_risk_has_no_effect_without_peers():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    legacy, critic = PayloadTrajectoryController(), PayloadTrajectoryController()
    critic.peer_sweep = critic.peer_soft = True
    args=((70., -20.), (0., 1.), np.full(32, 150.), np.linspace(-math.pi, math.pi, 32))
    for loaded in (False, True):
        assert legacy.command(*args, loaded=loaded) == critic.command(*args, loaded=loaded)


def geometry_fixture(position=(100.,0.),heading=math.pi,enabled=True):
    obj=object.__new__(MyDroneRescueV13)
    obj._bomb_center_enabled=enabled
    obj.measured_gps_position=lambda: np.asarray(position)
    obj.measured_compass_angle=lambda: heading
    return obj


def test_bomb_surface_shift_stabilizes_opposite_view_association():
    from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor as Sensor
    from swarm_rescue.solutions.upgrade.target_tracking import Observation
    hit=Sensor.Data(88.,0.,Sensor.TypeEntity.BOMB,False)
    tracker=GlobalTargetTracker(0)
    first=geometry_fixture()._semantic_to_world(hit)
    second=geometry_fixture((-100.,0.),0.)._semantic_to_world(hit)
    assert np.allclose(first,(0.,0.)) and np.allclose(second,(0.,0.))
    token=tracker.observe([Observation(first,88.,hit)],1)[0]
    assert tracker.observe([Observation(second,88.,hit)],2)==[token]
    assert len(tracker.tracks)==1
    raw=GlobalTargetTracker(0)
    raw.observe([Observation((12.,0.),88.,hit)],1)
    raw.observe([Observation((-12.,0.),88.,hit)],2)
    assert len(raw.tracks)==2


def test_bomb_center_correction_preserves_other_entities_and_sensor_failure():
    from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor as Sensor
    obj=geometry_fixture()
    for kind in (Sensor.TypeEntity.DRONE,Sensor.TypeEntity.DISPOSAL_CENTER):
        point=obj._semantic_to_world(Sensor.Data(88.,0.,kind,False))
        assert np.allclose(point,(12.,0.))
    hit=Sensor.Data(88.,0.,Sensor.TypeEntity.BOMB,False)
    assert np.allclose(geometry_fixture(enabled=False)._semantic_to_world(hit),(12.,0.))
    obj.measured_compass_angle=lambda: None
    assert obj._semantic_to_world(hit) is None
