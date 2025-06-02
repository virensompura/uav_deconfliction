from typing import List
from data_structures import Waypoint, DroneMission

def get_sample_simulated_schedules_no_conflict() -> List[DroneMission]:
    """Returns a list of simulated drone missions designed to NOT conflict with a typical primary mission."""
    schedules = []
    # Drone B: Flies on a parallel path, far away
    schedules.append(DroneMission(
        drone_id="DroneB_Safe",
        waypoints=[
            Waypoint(x=0, y=150, timestamp=0.0, z=10.0),
            Waypoint(x=100, y=150, timestamp=10.0, z=10.0)
        ]
    ))
    # Drone C: Flies much later in time
    schedules.append(DroneMission(
        drone_id="DroneC_Late",
        waypoints=[
            Waypoint(x=50, y=0, timestamp=20.0, z=20.0),
            Waypoint(x=50, y=100, timestamp=30.0, z=20.0)
        ]
    ))
    return schedules

def get_sample_simulated_schedules_with_conflict() -> List[DroneMission]:
    """Returns a list of simulated drone missions designed to potentially conflict."""
    schedules = []
    # Drone X: Head-on collision course (2D)
    schedules.append(DroneMission(
        drone_id="DroneX_HeadOn",
        waypoints=[
            Waypoint(x=100, y=50, timestamp=0.0), # No Z, so 2D
            Waypoint(x=0, y=50, timestamp=10.0)
        ]
    ))
    # Drone Y: Crossing path (3D), potential vertical proximity issue
    schedules.append(DroneMission(
        drone_id="DroneY_Crossing3D",
        waypoints=[
            Waypoint(x=0, y=0, timestamp=2.0, z=10.0),
            Waypoint(x=100, y=100, timestamp=12.0, z=12.0) # Primary might be at (50,50,10) @ t=5
        ]
    ))
    # Drone Z: Same path, slightly different timing (3D)
    schedules.append(DroneMission(
        drone_id="DroneZ_SamePath3D",
        waypoints=[
            Waypoint(x=0, y=0, timestamp=1.0, z=15.0),
            Waypoint(x=100, y=100, timestamp=11.0, z=25.0)
        ]
    ))
    return schedules

def get_stationary_conflict_schedule() -> List[DroneMission]:
    """A drone that is stationary in a conflicting position."""
    return [
        DroneMission(
            drone_id="DroneS_Stationary",
            waypoints=[Waypoint(x=50, y=50, timestamp=0.0, z=10.0), # Stays at 50,50,10 from t=0 to t=10
                       Waypoint(x=50, y=50, timestamp=10.0, z=10.0)]
        )
    ]