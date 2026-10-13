"""
Submission entrypoint helpers used by the launcher.
"""
import os

from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.solutions.my_drone_place_example import MyDronePlaceExample
from swarm_rescue.solutions.my_drone_rescue_example import MyDroneRescueExample


def drone_class_for_mode(mode: str):
    """
    Return the concrete drone class for the given team mode.
    """
    team_mode = TeamMode.from_string(mode)
    if team_mode == TeamMode.PLACE:
        return MyDronePlaceExample
    if os.environ.get("SWARM_RESCUE_ALGORITHM", "v13") == "v13":
        from swarm_rescue.solutions.upgrade.controller_v13 import MyDroneRescueV13
        return MyDroneRescueV13
    if os.environ.get("SWARM_RESCUE_ALGORITHM", "v13") == "v12":
        from swarm_rescue.solutions.upgrade.controller_v12 import MyDroneRescueV12
        return MyDroneRescueV12
    if os.environ.get("SWARM_RESCUE_ALGORITHM", "v13") == "v11":
        if os.environ.get('SWARM_RESCUE_PLANNER', 'astar') != 'astar':
            from swarm_rescue.solutions.upgrade.controller_v11_planning import MyDroneRescueV11Planning
            return MyDroneRescueV11Planning
        from swarm_rescue.solutions.upgrade.controller_v11 import MyDroneRescueV11
        return MyDroneRescueV11
    if os.environ.get("SWARM_RESCUE_ALGORITHM", "v13") == "v10":
        from swarm_rescue.solutions.upgrade.controller import MyDroneRescueExample as UpgradedRescue
        return UpgradedRescue
    return MyDroneRescueExample
