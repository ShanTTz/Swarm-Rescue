"""Regressions for anonymous identity, leases, and actual transport progress."""
from types import SimpleNamespace

from swarm_rescue.solutions.upgrade.target_tracking import TargetTracker, Observation, cluster_hits
from swarm_rescue.solutions.upgrade.controller import MyDroneRescueExample
import math
import numpy as np
from swarm_rescue.solutions.upgrade.mapping import OnlineEvidenceMap
from swarm_rescue.solutions.upgrade.semantic_coverage import SemanticCoverage
from swarm_rescue.solutions.upgrade.region_search import distances, choose_search_view, RouteCosts
from swarm_rescue.solutions.upgrade.pose_estimation import PoseEstimate
from swarm_rescue.solutions.upgrade.navigation import plan_occupancy_path, _line_is_free


def test_coobserved_distant_pair_stays_two_tracks_under_noise():
    tracker = TargetTracker(7)
    first = tracker.observe([Observation((0, 0), 180), Observation((38, 0), 180)], 1)
    second = tracker.observe([Observation((8, 2), 180), Observation((42, 2), 180)], 2)
    assert len(set(first)) == 2 and second == first
    assert tracker.join(*first) == first[0]
    assert len(tracker.tracks) == 2


def test_repeated_rays_do_not_multiply_targets_or_bridge_two_bombs():
    hits = [(SimpleNamespace(distance=180), p) for p in
            ((0, 0), (2, 1), (5, 0), (38, 0), (40, 1))]
    assert len(cluster_hits(hits)) == 2
    chain = [(SimpleNamespace(distance=180), (x, 0)) for x in (0, 18, 36)]
    assert len(cluster_hits(chain)) == 2


def test_radio_identity_aliases_are_stable_and_coobserved_pairs_remain_separate():
    a, b = TargetTracker(7), TargetTracker(2)
    original = a.observe([Observation((0, 0), 60), Observation((38, 0), 60)], 1)
    other = b.observe([Observation((2, 0), 60), Observation((40, 0), 60)], 1)
    a.merge(b.snapshot())
    b.merge(a.snapshot(), tuple(a.aliases.items()))
    assert [a.resolve(t) for t in original] == [b.resolve(t) for t in other]
    assert len(a.tracks) == len(b.tracks) == 2
    before = a.snapshot()
    a.merge(b.snapshot(), tuple(b.aliases.items()))
    assert a.snapshot() == before


def test_radio_source_identity_never_collides_between_drones():
    a, b = TargetTracker(0), TargetTracker(1)
    assert a.observe([Observation((0, 0), 30)], 0) != b.observe([Observation((0, 0), 30)], 0)


def _claimant():
    token = (7, -1000001)
    obj = SimpleNamespace(identifier=7, elapsed_timestep=100, drone_health=50,
        _TASK_LEASE_STEPS=180, _LOW_HEALTH=10, _target_invalid=lambda _: False,
        _opening_role_accepts=lambda *_: True, _peer_claim_stalled=lambda *_: False,
        measured_gps_position=lambda: (0, 0), _peer_state={}, _target_bid=lambda *_: 0,
        _claim_starts={(7, token): 20, (2, token): 80},
        _team_tasks={7: ('claim', token, (100, 0), 100), 2: ('claim', token, (100, 0), 100)})
    return obj, token


def test_later_lower_id_cannot_steal_a_continuing_claim():
    obj, token = _claimant()
    assert MyDroneRescueExample._can_claim_target(obj, token, (100, 0))
    obj.identifier = 2
    assert not MyDroneRescueExample._can_claim_target(obj, token, (100, 0))


def test_simultaneous_claims_have_one_winner_and_stalled_owner_can_be_replaced():
    obj, token = _claimant()
    obj._claim_starts[(2, token)] = 20
    assert not MyDroneRescueExample._can_claim_target(obj, token, (100, 0))
    obj._peer_claim_stalled = lambda *_: True
    assert MyDroneRescueExample._can_claim_target(obj, token, (100, 0))


def test_adjacent_targets_can_be_approached_in_parallel_from_far_away():
    obj, token = _claimant()
    obj._team_tasks[2] = ('claim', (2, -1000002), (138, 0), 100)
    obj._peer_state[2] = ((350, 0), 50, 100, 0, False, (0, 0))
    assert MyDroneRescueExample._can_claim_target(obj, token, (100, 0))


def test_transport_metric_uses_the_route_actually_being_executed():
    obj = SimpleNamespace(_known_disposal=(100, 0), _nav_target=(100, 0),
        _nav_path=[(50, 0), (100, 0)], _return_path=[(0, 100), (100, 100), (100, 0)],
        _active_carry_route='navigation')
    assert MyDroneRescueExample._task_distance(obj, 'carry', None, (0, 0)) == 100
    obj._active_carry_route = 'return'
    assert MyDroneRescueExample._task_distance(obj, 'carry', None, (0, 0)) == 300


def test_old_obstacle_packet_is_idempotent_and_new_free_packet_clears():
    grid = OnlineEvidenceMap((80, 80))
    for _ in range(8):
        grid.merge_peer_changes([(4, 4)], (), source=1, step=10)
    assert grid.evidence[4, 4] == 4 and not grid.occupied[4, 4]
    grid.merge_peer_changes([(4, 4)], (), source=1, step=11)
    assert grid.occupied[4, 4]
    grid.merge_peer_changes((), [(4, 4)], source=1, step=12)
    grid.merge_peer_changes((), [(4, 4)], source=1, step=13)
    assert not grid.occupied[4, 4]


def test_relay_keeps_original_map_version_and_cannot_resurrect_old_wall():
    a, b = OnlineEvidenceMap((80, 80)), OnlineEvidenceMap((80, 80))
    a.merge_snapshot([(4, 4, 8, 10, 2)], 10)
    b.merge_snapshot(a.snapshot(), 30)
    assert b.observed[4, 4] == 10
    assert b.occupied[4, 4]
    b.merge_snapshot([(4, 4, -4, 12, 2)], 30)
    b.merge_snapshot(a.snapshot(), 31)
    assert not b.occupied[4, 4] and b.evidence[4, 4] == -4
    b.merge_snapshot([(4, 4, 8, 100, 2), (-1, 0, 3, 12, 1), None], 31)
    assert b.evidence[4, 4] == -4


def test_geometric_visibility_does_not_mean_full_semantic_coverage():
    coverage = SemanticCoverage((360, 360))
    angles = np.linspace(-math.pi, math.pi, 181)
    coverage.update((0, 0), 0, np.full(181, 300), angles, [0])
    assert np.count_nonzero(coverage.mask) > 0
    assert np.all(coverage.mask[:, :coverage.cols//2] == 0)
    assert np.mean(coverage.observed_fraction()) < .2
    blocked = SemanticCoverage((360, 360))
    blocked.update((0, 0), 0, np.full(181, 20), angles, [0])
    assert not np.any(blocked.mask)


def test_coverage_gossip_is_idempotent_and_unions_partial_views():
    a = SemanticCoverage((100, 100))
    a.merge([(1, 1, 3)])
    a.merge([(1, 1, 3), (1, 1, 12), (1, 1, 512)])
    assert a.mask[1, 1] == 15


def test_connected_search_respects_door_and_never_selects_disconnected_view():
    grid = np.ones((12, 14), dtype=np.uint8)
    grid[:, 6] = 2
    start = (2, 2)
    coverage = np.ones(grid.shape)
    coverage[2, 10] = 0
    assert not math.isfinite(distances(grid, start)[2, 10])
    assert choose_search_view(grid, coverage, start, 0, {}, {}, {}, 20) is None
    grid[10, 6] = 1
    assert distances(grid, start)[2, 10] > math.dist(start, (2, 10))
    assert choose_search_view(grid, coverage, start, 0, {}, {}, {}, 20) == (2, 10)


def test_delivery_bid_accounts_for_loaded_return_and_low_health():
    obj = SimpleNamespace(_route_costs=None, _known_disposal=(0, 0))
    healthy = MyDroneRescueExample._target_bid(obj, 0, (100, 0), 50, (100, 0))
    weak = MyDroneRescueExample._target_bid(obj, 1, (100, 0), 15, (100, 0))
    assert healthy >= 130 and weak > healthy


def test_non_divisible_arena_path_does_not_cross_thin_wall():
    size = (253, 237)
    walls = np.zeros((237, 253), np.uint8)
    walls[:170, 123] = 1
    start, goal = (-70, 30), (70, 30)
    route = plan_occupancy_path(size, walls, start, goal, clearance=10, cell_size=16)
    assert route
    assert all(_line_is_free(a, b, size, walls) for a, b in zip([start]+route, route))
    assert any(point[1] < -50 for point in route)


def test_pose_prediction_uses_previous_heading_and_integrates_once_per_step():
    pose = PoseEstimate()
    pose.update(0, (0, 0), 0, (0, 0, 0))
    pose.update(1, None, None, (10, 0, math.pi/2))
    np.testing.assert_allclose(pose.measured_position(), (10, 0), atol=1e-6)
    assert abs(pose.measured_heading()-math.pi/2) < 1e-6
    pose.update(1, None, None, (10, 0, math.pi/2))
    np.testing.assert_allclose(pose.position, (10, 0), atol=1e-6)
    pose.update(2, None, None, (10, 0, 0))
    np.testing.assert_allclose(pose.position, (10, 10), atol=1e-6)


def test_pose_uncertainty_stops_mapping_then_motion_and_recovers_on_sensor_return():
    pose = PoseEstimate()
    pose.update(0, None, None, (0, 0, 0))
    assert pose.measured_position() is None and not pose.can_map
    pose.update(1, (0, 0), 0, (0, 0, 0))
    for step in range(2, 26):
        pose.update(step, None, None, (10, 0, 0))
    assert not pose.can_map and pose.measured_position() is None
    pose.update(26, (2, 3), .4, (0, 0, 0))
    np.testing.assert_allclose(pose.measured_position(), (2, 3))
    assert pose.can_map


def test_skipped_odometry_steps_are_not_integrated_as_one_complete_interval():
    pose = PoseEstimate()
    pose.update(0, (0, 0), 0, (0, 0, 0))
    pose.update(10, None, None, (10, 0, 0))
    np.testing.assert_allclose(pose.position, (0, 0))
    assert pose.measured_position() is None


def test_relative_lidar_recovery_moves_only_into_a_clear_sector():
    obj = SimpleNamespace(drone_health=50, _LOW_HEALTH=10,
        lidar_values=lambda:[40, 150, 40], lidar_rays_angles=lambda:[-1, 0, 1])
    cmd = {'forward':0, 'lateral':0, 'rotation':0, 'grasper':1}
    MyDroneRescueExample._control_local_recovery(obj, cmd)
    assert cmd['forward'] == .25 and cmd['grasper'] == 1
    obj.lidar_values = lambda:[40, 40, 40]
    cmd['forward'] = 0
    MyDroneRescueExample._control_local_recovery(obj, cmd)
    assert cmd['forward'] == 0


def test_handoff_cannot_ack_a_nearby_robot_through_a_wall():
    obj = SimpleNamespace(measured_gps_position=lambda:(0,0), measured_compass_angle=lambda:0,
        lidar_values=lambda:[20], lidar_rays_angles=lambda:[0])
    assert not MyDroneRescueExample._handoff_visible(obj, (70,0))
    obj.lidar_values = lambda:[60]
    assert MyDroneRescueExample._handoff_visible(obj, (70,0))


def test_baseline_policy_steering_branch_has_all_runtime_dependencies():
    from swarm_rescue.solutions.upgrade.baseline_search import baseline_control_patrol
    calls = []
    obj = SimpleNamespace(identifier=4, size_area=(1000,800),
        _run_lidar_escape_if_needed=lambda _:False, _patrol_scan_steps_left=0,
        _patrol_route=((0,0),), _patrol_index=0, _team_deliveries={},
        _max_steps=2000, _team_size=10, _middle_edge_sweep_enabled=False,
        _middle_edge_probe_enabled=False, _nav_target=(0,0), _patrol_target_age=0,
        _PATROL_ARRIVE_RADIUS=20, _within_radius=lambda *_:False,
        measured_gps_position=lambda:(0,-50), measured_compass_angle=lambda:0,
        lidar_values=lambda:[300], lidar_rays_angles=lambda:[0], _nav_age=0,
        _move_toward=lambda *args, **kw:calls.append(kw))
    baseline_control_patrol(obj, {})
    assert calls and obj._patrol_target_age == 1


def test_controller_public_pose_reads_share_one_odometry_update(monkeypatch):
    from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
    monkeypatch.setattr(DroneAbstract, 'measured_gps_position', lambda _: None)
    monkeypatch.setattr(DroneAbstract, 'measured_compass_angle', lambda _: None)
    obj = object.__new__(MyDroneRescueExample)
    obj._pose_estimate = PoseEstimate()
    obj._pose_estimate.update(0, (0,0), 0, (0,0,0))
    obj.elapsed_timestep = 1
    obj.odometer_is_disabled = lambda:False
    obj.odometer_values = lambda:np.array([4,0,.1])
    np.testing.assert_allclose(obj.measured_gps_position(), (4,0))
    obj.measured_compass_angle()
    obj.measured_velocity()
    np.testing.assert_allclose(obj.measured_gps_position(), (4,0))


def test_cleared_fine_evidence_can_remove_coarse_search_wall():
    online = OnlineEvidenceMap((144,144))
    online.merge_snapshot([(5,5,-6,10,0)],10)
    grid = np.zeros((4,4),dtype=np.uint8)
    grid[1,1] = 2
    obj = SimpleNamespace(_online_map=online, _frontier_grid=grid, identifier=0,
        elapsed_timestep=10, _FRONTIER_CELL_SIZE=36, _peer_frontier_targets={},
        measured_gps_position=lambda:None, measured_compass_angle=lambda:None,
        lidar_values=lambda:None, communicator=SimpleNamespace(received_messages=[]))
    MyDroneRescueExample._update_frontier_map(obj)
    assert grid[1,1] == 1


def test_path_visibility_check_catches_single_pixel_wall_and_corner_cut():
    grid = np.zeros((100,100),np.uint8)
    grid[:,47] = 1
    assert not _line_is_free((-40,0),(40,0),(100,100),grid)
    grid[:]=0
    grid[50,51]=1
    assert not _line_is_free((0,0),(1,-1),(100,100),grid)


def test_radio_coobservation_constraints_survive_relay():
    a,b = TargetTracker(0), TargetTracker(1)
    tokens = a.observe([Observation((0,0),60),Observation((38,0),60)], 1)
    constraints = tuple(tuple(sorted(pair)) for pair in a.separated)
    b.merge(a.snapshot(), separated=constraints)
    b.join(*tokens)
    assert len(b.tracks)==2


def test_exhausted_projected_route_does_not_resume_direct_drive_to_blocked_goal():
    calls=[]
    obj=SimpleNamespace(_ARRIVE_RADIUS=10, _PATROL_ARRIVE_RADIUS=20,
        _nav_age=1, _nav_target=(100,0), _nav_path=[(0,0)], _nav_plan_failed=False,
        _nav_goal_projected=True, _nav_fallback_target=None, _nav_fallback_until=0,
        measured_gps_position=lambda:(0,0), measured_compass_angle=lambda:0,
        grasped_bombs=lambda:[], _run_lidar_escape_if_needed=lambda _:False,
        elapsed_timestep=100, lidar_values=lambda:[300], lidar_rays_angles=lambda:[math.pi/2],
        _move_toward=lambda *args, **kw:calls.append(args[:2]))
    MyDroneRescueExample._navigate_online(obj,(100,0),{})
    assert obj._nav_plan_failed and calls and calls[0] != (100,0)


def test_disposed_identity_cannot_absorb_a_new_nearby_active_bomb():
    tracker = TargetTracker(0)
    done = tracker.observe([Observation((0,0),60)],1)[0]
    tracker.retire([done])
    active = tracker.observe([Observation((12,0),60)],2)[0]
    assert active != done
    tracker.join(done, active)
    assert tracker.resolve(active) != tracker.resolve(done)


def test_v13_default_allows_legacy_selection_and_keeps_red_entrypoint(monkeypatch):
    from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode
    from swarm_rescue.solutions.my_drone_rescue_example import MyDroneRescueExample as OriginalRescue
    from swarm_rescue.solutions.my_drone_place_example import MyDronePlaceExample
    monkeypatch.delenv('SWARM_RESCUE_ALGORITHM', raising=False)
    from swarm_rescue.solutions.upgrade.controller_v13 import MyDroneRescueV13
    assert drone_class_for_mode('rescue') is MyDroneRescueV13
    assert drone_class_for_mode('place') is MyDronePlaceExample
    monkeypatch.setenv('SWARM_RESCUE_ALGORITHM', 'v9')
    assert drone_class_for_mode('rescue') is OriginalRescue
    monkeypatch.setenv('SWARM_RESCUE_ALGORITHM', 'v10')
    assert drone_class_for_mode('rescue') is MyDroneRescueExample
    assert drone_class_for_mode('place') is MyDronePlaceExample


def test_route_cost_field_uses_observed_connectivity_and_invalidates_by_epoch():
    grid = OnlineEvidenceMap((240,240))
    grid.evidence[:]=-6
    grid.bitmap[:,120]=1
    costs = RouteCosts((240,240))
    costs.refresh(grid,10)
    estimate=costs.estimate((-48,48),(48,48))
    assert estimate > 96
    assert costs.fields
    grid.bitmap[:,120]=0
    costs.refresh(grid,20)
    assert costs.estimate((-48,48),(48,48)) == estimate
    costs.refresh(grid,40)
    assert costs.estimate((-48,48),(48,48)) == 96
