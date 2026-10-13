"""Route policy extracted unchanged from the verified v11 planner trial."""
from swarm_rescue.solutions.upgrade.controller_v11 import MyDroneRescueV11
from swarm_rescue.solutions.upgrade.navigation import plan_occupancy_path
from swarm_rescue.solutions.upgrade.lazy_theta import lazy_theta_search


def plan_for_policy(self, start, target):
    backend = getattr(self, '_planner_backend', 'astar')
    loaded = bool(self.grasped_bombs())
    if backend == 'astar' or (backend == 'hybrid_dstar' and (not loaded)) or (backend == 'phase_theta' and (loaded or getattr(self, '_operational_stage', 'search') != 'search')):
        return MyDroneRescueV11._plan_navigation(self, start, target)
    search = self._return_searches[16] if backend == 'hybrid_dstar' else lazy_theta_search
    clearance = 20 if not loaded or self._use_trajectory else 32
    route = plan_occupancy_path(self.size_area, self._online_obstacles, start, target, clearance=clearance, cell_size=16, search=search)
    if not route:
        route = plan_occupancy_path(self.size_area, self._online_obstacles, start, target, clearance=18 if loaded and self._use_trajectory else clearance, cell_size=8, search=self._return_searches[8] if backend == 'hybrid_dstar' else lazy_theta_search)
    if backend in ('hybrid_dstar', 'phase_theta') and getattr(self, '_diagnostics_enabled', False):
        self._diagnostic_event('planner_route', backend=backend, loaded=loaded, waypoints=len(route), reused=sum((planner.reuses for planner in self._return_searches.values())), resets=sum((planner.resets for planner in self._return_searches.values())), expansions=sum((planner.last_expansions for planner in self._return_searches.values())))
    return route
