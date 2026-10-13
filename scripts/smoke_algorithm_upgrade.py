"""Short CPU physics/competition-guard check, not a scored benchmark."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=80)
    parser.add_argument('--region-search', action='store_true')
    parser.add_argument('--route-bids', action='store_true')
    parser.add_argument('--algorithm', choices=('v10', 'v11', 'v12', 'v13'), default='v13')
    parser.add_argument('--planner', choices=('astar', 'lazy_theta', 'hybrid_dstar', 'phase_theta'), default='astar')
    parser.add_argument('--motion', choices=('balanced', 'v11'), default='balanced')
    args = parser.parse_args()
    os.environ['SWARM_RESCUE_ALGORITHM'] = args.algorithm
    os.environ['SWARM_RESCUE_PLANNER'] = args.planner
    os.environ['SWARM_RESCUE_MOTION'] = args.motion
    os.environ['SWARM_RESCUE_REGION_SEARCH'] = '1' if args.region_search else '0'
    os.environ['SWARM_RESCUE_ROUTE_BIDS'] = '1' if args.route_bids else '0'
    import numpy as np
    random.seed(20261006)
    np.random.seed(20261006)
    from swarm_rescue.simulation.gui_map.playground import Playground
    original = Playground.__init__
    def cpu_init(self, *values, **kwargs):
        kwargs['use_shaders'] = False
        return original(self, *values, **kwargs)
    Playground.__init__ = cpu_init
    from swarm_rescue.maps.map_01 import Map01
    from swarm_rescue.simulation.drone.ground_truth_guard import run_control
    from swarm_rescue.simulation.reporting.bombs_io import bombs_from_data, load_bombs_file
    from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode
    selected = drone_class_for_mode('rescue')
    assert selected.__module__ == ('swarm_rescue.solutions.upgrade.controller_v11_planning'
                                  if args.algorithm == 'v11' and args.planner != 'astar' else
                                  'swarm_rescue.solutions.upgrade.controller_'+args.algorithm
                                  if args.algorithm in ('v11', 'v12', 'v13') else 'swarm_rescue.solutions.upgrade.controller')
    arena = Map01(selected)
    arena.spawn_bombs(bombs_from_data(load_bombs_file(
        ROOT/'fixtures/bombs/Map01_regression_sentinel.json')))
    try:
        for drone in arena.drones:
            drone.set_competition_mode(True)
        arena.playground.step()
        for step in range(1, args.steps+1):
            commands = {}
            for drone in arena.drones:
                drone.elapsed_timestep = step
                drone.gps()._disabled = 20 <= step < 34
                drone.compass()._disabled = 25 <= step < 35
                drone.semantic()._disabled = 40 <= step < 45
            messages = {drone: {drone.communicator: (None, drone.define_message_for_all())}
                        for drone in arena.drones}
            for drone in arena.drones:
                command = run_control(drone)
                assert all(math.isfinite(float(command[key])) and -1 <= command[key] <= 1
                           for key in ('forward', 'lateral', 'rotation'))
                assert command['grasper'] in (0, 1)
                commands[drone] = command
            arena.playground.step(all_commands=commands, all_messages=messages)
        if args.steps >= 45:
            assert all(drone._pose_estimate.can_map for drone in arena.drones)
        print(json.dumps(dict(diagnostic_only=True, steps=args.steps,
            sensor_backend='cpu', competition_guard=True, selected_class=selected.__module__,
            region_search=args.region_search, route_bids=args.route_bids)))
    finally:
        arena.playground.close_window()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
