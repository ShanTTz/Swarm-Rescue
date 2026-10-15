import math
from types import SimpleNamespace
import pytest
import numpy as np
from swarm_rescue.solutions.upgrade.carrier_traffic import CarrierRightOfWay
from swarm_rescue.solutions.upgrade.controller_v13 import MyDroneRescueV13
from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12

def peer(point=(0.,0.), carrying=True, step=100, heading=0., velocity=(0.,0.)):
    return point,50,step,heading,carrying,velocity

def cluster(step=100, heading=0.):
    return {1:peer(step=step,heading=heading),3:peer((40.,25.),False,step)}

def ready():
    traffic=CarrierRightOfWay()
    for step in range(76,100,4):
        traffic.choose((40.,0.),2,False,step,cluster(step),{1:(100.,0.)},lambda a,b:False)
    return traffic

@pytest.mark.parametrize('heading', [0.,math.pi/2,math.pi])
def test_stationary_carrier_gets_room_even_without_head_on_heading(heading):
    traffic=ready()
    target=traffic.choose((40.,0.),2,False,100,cluster(heading=heading),
                          {1:(100.,0.)},lambda a,b:True)
    assert target and abs(target[1])>=48 and math.dist(target,(0.,0.))>=55

def test_refuge_persists_after_drone_exits_corridor_until_carrier_passes():
    traffic=ready()
    target=traffic.choose((40.,0.),2,False,100,cluster(), {1:(100.,0.)},lambda a,b:True)
    assert traffic.choose(target,2,False,120,cluster(step=120), {1:(100.,0.)},lambda a,b:True)==target
    assert traffic.choose(target,2,False,121,{1:peer((90.,0.),step=121),3:peer((40.,25.),False,121)}, {1:(150.,0.)},lambda a,b:True) is None

@pytest.mark.parametrize('data,loaded',[(peer(carrying=False),False),(peer(step=90),False),(peer(),True)])
def test_empty_stale_peers_and_own_payload_do_not_trigger(data,loaded):
    traffic=ready()
    assert traffic.choose((40.,0.),2,loaded,100,{1:data,3:peer((40.,25.),False)},{1:(100.,0.)},lambda a,b:True) is None

def test_no_unchecked_route_is_used_when_refuge_blocked():
    traffic=ready()
    assert traffic.choose((40.,0.),2,False,100,cluster(),{1:(100.,0.)},lambda a,b:False) is None

def test_refuge_uses_only_available_side_of_wall():
    traffic=ready()
    target=traffic.choose((40.,0.),2,False,100,cluster(),{1:(100.,0.)},lambda a,b:b[1]<-48)
    assert target and target[1]<-48

def test_lost_radio_releases_pinned_refuge_without_following_stale_carrier():
    traffic=ready()
    traffic.choose((40.,0.),2,False,100,cluster(), {1:(100.,0.)},lambda a,b:True)
    assert traffic.choose((40.,0.),2,False,107,cluster(), {},lambda a,b:True) is None
    assert traffic.owner is None

@pytest.mark.parametrize('yield_goal',[None,(40.,55.)])
@pytest.mark.parametrize('answer',[False,True,None])
def test_actual_rollout_delegates_to_existing_collision_controller(monkeypatch,yield_goal,answer):
    obj=object.__new__(MyDroneRescueV13)
    obj._traffic_target=lambda:yield_goal
    seen=[]
    def rollout(self,x,y,cmd): seen.append((x,y)); return answer
    monkeypatch.setattr(MyDroneRescueV12,'_rollout_toward',rollout)
    result=obj._rollout_toward(200.,0.,{})
    assert result is (False if yield_goal is not None and answer is not None else answer)
    assert seen==[yield_goal or (200.,0.)]
    assert obj._traffic_motion_goal==(200.,0.)

def test_idle_path_also_yields_and_empty_yield_does_not_grab(monkeypatch):
    obj=object.__new__(MyDroneRescueV13)
    obj.elapsed_timestep=100
    obj._traffic_checked_step=-1
    obj._traffic_target=lambda:(40.,55.)
    obj._carrier_traffic=SimpleNamespace(owner=1)
    obj._empty_traffic=SimpleNamespace(target=None)
    obj.grasped_bombs=lambda:[]
    monkeypatch.setattr(MyDroneRescueV12,'_control_step',lambda self:dict(grasper=1))
    seen=[]
    monkeypatch.setattr(MyDroneRescueV12,'_rollout_toward',lambda self,x,y,cmd:seen.append((x,y)))
    assert obj._control_step()['grasper']==0
    assert seen==[(40.,55.)]


def test_single_crossing_and_moving_carrier_keep_original_motion():
    traffic=ready()
    assert traffic.choose((40.,0.),2,False,100,{1:peer()},{1:(100.,0.)},lambda a,b:True) is None
    traffic=CarrierRightOfWay()
    for step in range(76,105,4):
        peers={1:peer((2.*(step-76),0.),step=step,velocity=(2.,0.)),3:peer((60.,25.),False,step)}
        assert traffic.choose((60.,0.),2,False,step,peers,{1:(200.,0.)},lambda a,b:True) is None


def test_long_radio_gap_cannot_be_mistaken_for_continuous_stall():
    traffic=ready()
    assert traffic.choose((40.,0.),2,False,200,cluster(step=200),{1:(100.,0.)},lambda a,b:True) is None


def test_refuge_prefers_the_side_of_the_original_route():
    north=ready().choose((40.,0.),2,False,100,cluster(),{1:(100.,0.)},lambda a,b:True,own_goal=(40.,200.))
    south=ready().choose((40.,0.),2,False,100,cluster(),{1:(100.,0.)},lambda a,b:True,own_goal=(40.,-200.))
    assert north[1]>0 and south[1]<0


def test_stalled_empty_queue_member_escapes_but_keeps_stable_first_owner():
    from swarm_rescue.solutions.upgrade.carrier_traffic import EmptyCongestionRecovery
    follower,leader=EmptyCongestionRecovery(),EmptyCongestionRecovery()
    target=None
    for step in range(0,65,8):
        health=50-step//24
        target=follower.choose((0.,0.),4,False,health,step,{1:peer((25.,0.),False,step)},False,False,(100.,0.),lambda a,b:True)
        assert leader.choose((0.,0.),0,False,health,step,{1:peer((25.,0.),False,step)},False,False,(100.,0.),lambda a,b:True) is None
    assert target and math.dist(target,(25.,0.))>=40
    assert follower.choose(target,4,False,48,72,{},False,False,(100.,0.),lambda a,b:True)==target


def test_wall_stall_uses_a_checked_exit_and_resets_after_bounded_escape():
    from swarm_rescue.solutions.upgrade.carrier_traffic import EmptyCongestionRecovery
    recovery=EmptyCongestionRecovery()
    for step in range(0,65,8):
        target=recovery.choose((0.,0.),0,False,50-step//24,step,{},True,False,(100.,0.),lambda a,b:b[0]<-30)
    assert target and target[0]<-30
    assert recovery.choose(target,0,False,48,144,{},False,False,(100.,0.),lambda a,b:True) is None


@pytest.mark.parametrize('loaded,loss,moving',[(True,3,False),(False,0,False),(False,3,True)])
def test_payload_normal_wait_and_progress_do_not_trigger_empty_escape(loaded,loss,moving):
    from swarm_rescue.solutions.upgrade.carrier_traffic import EmptyCongestionRecovery
    recovery=EmptyCongestionRecovery()
    for step in range(0,65,8):
        point=(float(step*2),0.) if moving else (0.,0.)
        assert recovery.choose(point,4,loaded,50-loss*step//64,step,{},loss>0,False,(100.,0.),lambda a,b:True) is None


def test_wall_recovery_does_not_depend_on_wall_clock_health_cooldown():
    from swarm_rescue.solutions.upgrade.carrier_traffic import EmptyCongestionRecovery
    results=[]
    for hp in (50,48):
        recovery=EmptyCongestionRecovery()
        for step in range(0,65,8):
            target=recovery.choose((0.,0.),4,False,hp,step,{},True,False,(100.,0.),lambda a,b:True)
        results.append(target)
    assert results[0] is not None and results[0]==results[1]


def test_short_deadline_escape_requires_real_fresh_carriers_not_claims():
    obj=object.__new__(MyDroneRescueV13)
    obj._max_steps=2000;obj.elapsed_timestep=100;obj.identifier=0
    obj._EXPECTED_BOMBS=10;obj._team_deliveries={0:8.}
    obj.grasped_bombs=lambda:[]
    obj._peer_state={1:peer(carrying=True),2:peer(carrying=False)}
    assert not obj._empty_escape_enabled()
    obj._peer_state[2]=peer(carrying=True)
    assert obj._empty_escape_enabled()
    obj._peer_state[2]=peer(carrying=True,step=95)
    assert not obj._empty_escape_enabled()
    obj._max_steps=5000
    assert obj._empty_escape_enabled()


def test_squeezed_empty_drone_can_increase_gap_without_reaching_full_margin_yet():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    args=((0.,100.),(0.,0.),np.full(32,300.),np.linspace(-math.pi,math.pi,32))
    # Public GPS noise can put two physically touching neighbours inside the
    # 23px uncertainty margin, so no six-step rollout can reach that margin.
    peers=(((19.,0.),(0.,0.)),((-19.,0.),(0.,0.)))
    baseline,escape=PayloadTrajectoryController(),PayloadTrajectoryController()
    baseline.command(*args,peers=peers)
    assert not baseline.last_safe
    escape.escape_peers=True
    command=escape.command(*args,peers=peers)
    assert escape.last_safe and abs(command['forward'])+abs(command['lateral'])>0


def test_overlap_escape_does_not_relax_loaded_drone_checks():
    from swarm_rescue.solutions.upgrade.trajectory_control import PayloadTrajectoryController
    args=((0.,100.),(0.,0.),np.full(32,300.),np.linspace(-math.pi,math.pi,32))
    peers=(((20.,0.),(0.,0.)),)
    baseline,escape=PayloadTrajectoryController(),PayloadTrajectoryController()
    escape.escape_peers=True
    assert baseline.command(*args,loaded=True,peers=peers)==escape.command(*args,loaded=True,peers=peers)


def test_short_mission_queue_arbitration_preserves_near_home_pickup_crossings():
    obj=object.__new__(MyDroneRescueV13)
    obj._max_steps=2000;obj.size_area=(1250,800);obj._known_disposal=(-505.,302.)
    obj._peer_state={1:peer((265.,145.)),2:peer((-200.,-80.))}
    assert obj._traffic_eligible_carriers()=={1}
    obj._max_steps=5000
    assert obj._traffic_eligible_carriers() is None


def test_recovery_yields_to_a_newly_visible_pickup_without_waiting_out_timer():
    from swarm_rescue.solutions.upgrade.carrier_traffic import EmptyCongestionRecovery
    recovery=EmptyCongestionRecovery()
    for step in range(0,65,8):
        target=recovery.choose((0.,0.),4,False,50,step,{},True,False,(100.,0.),lambda a,b:True)
    assert target is not None
    assert recovery.choose((10.,0.),4,False,50,72,{},False,True,(100.,0.),lambda a,b:True) is None
    assert recovery.target is None


def test_motion_escape_keeps_claim_and_neighbour_records_available():
    token=(1,-1000001)
    calls=[]
    obj=SimpleNamespace(elapsed_timestep=100,identifier=4,drone_health=50,
        measured_gps_position=lambda:np.array([0.,0.]),measured_compass_angle=lambda:0.,
        _pose_estimate=SimpleNamespace(can_map=True),grasped_bombs=lambda:[],
        communicator=SimpleNamespace(received_messages=[]),_peer_state={},
        _carrier_traffic=SimpleNamespace(owner=None,choose=lambda *args,**kwargs:None),
        _empty_traffic=SimpleNamespace(target=None,choose=lambda *args,**kwargs:(45.,0.)),
        _empty_escape_enabled=lambda:True,_traffic_eligible_carriers=lambda:None,
        _online_obstacles=np.zeros((100,100),np.uint8),size_area=(100,100),semantic_values=lambda:[],
        _traffic_motion_goal=(100.,0.),_traffic_segment_free=lambda a,b:True,
        _diagnostic_event=lambda *args,**kwargs:None,_current_bomb_token=token,
        _bomb_token_centers={token:(10.,0.)},_claim_cooldown_until={},
        _abandon_claim=lambda *args:calls.append(args))
    assert MyDroneRescueV13._traffic_target(obj)==(45.,0.)
    assert obj._current_bomb_token==token and not obj._claim_cooldown_until and not calls


@pytest.mark.parametrize('covered,near_cargo,visible,clearance,expected',[
    (True,False,False,200.,True),
    (False,False,False,200.,False),
    (True,True,False,200.,False),
    (True,False,True,200.,False),
    (True,False,False,35.,False),
])
def test_safe_standby_requires_actual_loads_clear_space_and_no_new_pickup(covered,near_cargo,visible,clearance,expected):
    from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor as Sensor
    obj=SimpleNamespace(grasped_bombs=lambda:[],_physical_loads_accounted=lambda:covered,
        semantic_values=lambda:[Sensor.Data(30.,0.,Sensor.TypeEntity.BOMB,False)] if visible else [],
        lidar_values=lambda:np.full(32,clearance),measured_gps_position=lambda:np.array([0.,0.]),
        elapsed_timestep=100,identifier=0,
        _peer_state={1:peer((110. if near_cargo else 200.,0.))})
    assert MyDroneRescueV13._traffic_can_standby(obj) is expected
