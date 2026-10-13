"""v12 A* direction: v11 control with an independent route policy."""
import os

from swarm_rescue.solutions.upgrade.controller_v11 import MyDroneRescueV11
from swarm_rescue.solutions.upgrade.dstar_lite import DStarLite
from swarm_rescue.solutions.upgrade.planner_policy import plan_for_policy


class MyDroneRescueV12(MyDroneRescueV11):
    DEFAULT_PLANNER = 'astar'
    _plan_navigation = plan_for_policy

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._planner_backend = os.environ.get('SWARM_RESCUE_PLANNER', self.DEFAULT_PLANNER)
        if self._planner_backend not in ('astar', 'lazy_theta', 'hybrid_dstar', 'phase_theta'):
            raise ValueError('Unknown SWARM_RESCUE_PLANNER')
        self._return_searches = {16: DStarLite(), 8: DStarLite()}
