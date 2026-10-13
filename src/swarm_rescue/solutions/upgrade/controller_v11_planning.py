"""Planner-only ablation on v11: preserve its motion, tracking and allocation."""
import os

from swarm_rescue.solutions.upgrade.controller_v11 import MyDroneRescueV11
from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
from swarm_rescue.solutions.upgrade.dstar_lite import DStarLite


class MyDroneRescueV11Planning(MyDroneRescueV11):
    # This method only selects search; it explicitly delegates legacy A* to
    # v11. No v12 constructor, control loop or lifecycle methods are inherited.
    _plan_navigation = MyDroneRescueV12._plan_navigation

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._planner_backend = os.environ.get('SWARM_RESCUE_PLANNER', 'astar')
        if self._planner_backend not in ('astar', 'lazy_theta', 'hybrid_dstar', 'phase_theta'):
            raise ValueError('Unknown SWARM_RESCUE_PLANNER')
        self._return_searches = {16: DStarLite(), 8: DStarLite()}
