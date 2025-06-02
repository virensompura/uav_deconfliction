import numpy as np
from typing import List, Optional, Tuple, Dict, Any
from data_structures import DroneMission, Waypoint

# --- Constants ---
MINIMUM_DISTANCE_THRESHOLD_2D = 10.0  # meters, for 2D separation
MINIMUM_DISTANCE_THRESHOLD_3D = 15.0  # meters, for 3D separation (can be different)
VERTICAL_SEPARATION_THRESHOLD = 5.0   # meters, for 3D, minimum vertical distance if horizontally close
TIME_STEP_RESOLUTION = 0.5           # seconds, for discretizing time in checks

ConflictInfo = Dict[str, Any]

# --- Helper Functions ---
def get_drone_position_at_time(mission: DroneMission, time_t: float) -> Optional[Waypoint]:
    """Calculates the drone's position (as a Waypoint object) at a specific time t."""
    if not mission.waypoints:
        return None
    
    # If time_t is before mission start or after mission end
    if time_t < mission.get_start_time() - 1e-6 or time_t > mission.get_end_time() + 1e-6:
        return None

    # If mission is a single point (stationary)
    if len(mission.waypoints) == 1:
        wp = mission.waypoints[0]
        # Drone is at its single waypoint if time_t is within its defined single point 'duration'
        # (which is its start and end time being the same or very close for PrimaryMission)
        # For DroneMission, a single waypoint means it's there at that specific timestamp.
        # Let's assume for a single waypoint mission, it's valid at its timestamp.
        # For PrimaryDroneMission with start=end, it means it's there.
        # This logic is simplified; robust handling of stationary drones might need more.
        if abs(time_t - wp.timestamp) < 1e-6 or (mission.get_start_time() == mission.get_end_time() and abs(time_t - mission.get_start_time()) < 1e-6):
            return Waypoint(wp.x, wp.y, time_t, wp.z) # Return with the query time_t
        return None


    # Find the segment the drone is on at time_t
    prev_wp: Optional[Waypoint] = None
    next_wp: Optional[Waypoint] = None

    if time_t <= mission.waypoints[0].timestamp + 1e-6: # At or before first waypoint
        wp = mission.waypoints[0]
        return Waypoint(wp.x, wp.y, time_t, wp.z) # Return with query time_t

    if time_t >= mission.waypoints[-1].timestamp - 1e-6: # At or after last waypoint
        wp = mission.waypoints[-1]
        return Waypoint(wp.x, wp.y, time_t, wp.z) # Return with query time_t
        
    for i in range(len(mission.waypoints) - 1):
        wp1 = mission.waypoints[i]
        wp2 = mission.waypoints[i+1]
        if wp1.timestamp -1e-6 <= time_t <= wp2.timestamp + 1e-6:
            prev_wp = wp1
            next_wp = wp2
            break
    
    if not prev_wp or not next_wp:
        # This should ideally not be reached if time_t is within mission bounds and handled above
        return None

    # If drone is at a waypoint (segment duration is zero or time_t matches waypoint time)
    if abs(prev_wp.timestamp - next_wp.timestamp) < 1e-6 or abs(time_t - prev_wp.timestamp) < 1e-6:
        return Waypoint(prev_wp.x, prev_wp.y, time_t, prev_wp.z)
    if abs(time_t - next_wp.timestamp) < 1e-6:
        return Waypoint(next_wp.x, next_wp.y, time_t, next_wp.z)

    # Linear interpolation
    segment_duration = next_wp.timestamp - prev_wp.timestamp
    time_into_segment = time_t - prev_wp.timestamp
    fraction = time_into_segment / segment_duration

    x = prev_wp.x + fraction * (next_wp.x - prev_wp.x)
    y = prev_wp.y + fraction * (next_wp.y - prev_wp.y)
    z: Optional[float] = None

    if prev_wp.z is not None and next_wp.z is not None:
        z = prev_wp.z + fraction * (next_wp.z - prev_wp.z)
    elif prev_wp.z is not None: # If next_wp.z is None, assume constant altitude from prev_wp
        z = prev_wp.z
    elif next_wp.z is not None: # Should not happen if waypoints are consistent, but if so, use next
        z = next_wp.z
        
    return Waypoint(x, y, time_t, z)


def calculate_distance(wp1: Waypoint, wp2: Waypoint) -> Tuple[float, Optional[float]]:
    """
    Calculates 2D (horizontal) and optionally 3D distance between two waypoints.
    Returns (horizontal_distance, full_3d_distance_if_applicable)
    """
    dx = wp1.x - wp2.x
    dy = wp1.y - wp2.y
    dist_2d = np.sqrt(dx**2 + dy**2)
    
    dist_3d: Optional[float] = None
    if wp1.z is not None and wp2.z is not None:
        dz = wp1.z - wp2.z
        dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)
        return dist_2d, dist_3d
    
    return dist_2d, None


# --- Main Conflict Checking Logic ---
def check_for_conflicts(
    primary_mission: DroneMission, # Can be PrimaryDroneMission
    other_drone_schedules: List[DroneMission],
    safety_buffer_2d: float = MINIMUM_DISTANCE_THRESHOLD_2D,
    safety_buffer_3d: float = MINIMUM_DISTANCE_THRESHOLD_3D,
    vertical_sep_threshold: float = VERTICAL_SEPARATION_THRESHOLD,
    time_resolution: float = TIME_STEP_RESOLUTION
) -> List[ConflictInfo]:
    
    conflicts: List[ConflictInfo] = []
    
    # Determine overall time window for checking based on primary mission
    # For PrimaryDroneMission, this uses its overall mission window
    # For a generic DroneMission, it's based on its first and last waypoint
    if hasattr(primary_mission, 'mission_overall_start_time'):
        # This check ensures we are dealing with a PrimaryDroneMission instance
        # that has the specific overall time window attributes.
        # pylint: disable=no-member 
        # (Pylint might complain if primary_mission is typed as DroneMission and not PrimaryDroneMission here)
        check_start_time = primary_mission.mission_overall_start_time
        check_end_time = primary_mission.mission_overall_end_time
    else:
        check_start_time = primary_mission.get_start_time()
        check_end_time = primary_mission.get_end_time()

    if check_start_time == check_end_time : # If mission is instantaneous or stationary
        # For an instantaneous primary mission, check only at that single time point
        # If it's stationary over a duration, the loop below will handle it if resolution is small
        # Let's ensure at least one check if start and end are same
        # or make check_end_time slightly larger to ensure one iteration.
        if time_resolution == 0: # Avoid infinite loop
            time_resolution = 0.1
        # Ensure the loop runs at least once for an instantaneous primary mission
        check_end_time = check_start_time + time_resolution/2


    current_time = check_start_time
    while current_time <= check_end_time + 1e-6: # Add epsilon for float comparison
        primary_wp_at_t = get_drone_position_at_time(primary_mission, current_time)
        
        if primary_wp_at_t is None: # Primary drone not active or error
            current_time += time_resolution
            if current_time > check_end_time and current_time - time_resolution < check_end_time:
                 current_time = check_end_time # Ensure the exact end time is checked
            elif current_time > check_end_time and check_start_time == check_end_time: # For instantaneous check
                 break # Only one check needed
            continue

        for other_drone in other_drone_schedules:
            if primary_mission.drone_id == other_drone.drone_id:
                continue # Don't check against self

            other_wp_at_t = get_drone_position_at_time(other_drone, current_time)
            
            if other_wp_at_t is None: # Other drone not active at this time
                continue
            
            # Both drones are active, check for conflict
            dist_2d, dist_3d = calculate_distance(primary_wp_at_t, other_wp_at_t)
            
            is_conflict = False
            conflict_type = ""

            if primary_wp_at_t.is_3d and other_wp_at_t.is_3d and dist_3d is not None:
                # Both are 3D: use 3D safety buffer
                if dist_3d < safety_buffer_3d:
                    is_conflict = True
                    conflict_type = "3D proximity"
                # Check vertical separation if horizontally close but 3D separation is met
                elif dist_2d < safety_buffer_2d and abs(primary_wp_at_t.z - other_wp_at_t.z) < vertical_sep_threshold:
                    is_conflict = True
                    conflict_type = "Insufficient vertical separation"
            else:
                # At least one drone is 2D (or z is None), use 2D safety buffer
                if dist_2d < safety_buffer_2d:
                    is_conflict = True
                    conflict_type = "2D proximity"
            
            if is_conflict:
                conflicts.append({
                    "time": current_time,
                    "primary_drone_id": primary_mission.drone_id,
                    "primary_pos": (primary_wp_at_t.x, primary_wp_at_t.y, primary_wp_at_t.z),
                    "conflicting_drone_id": other_drone.drone_id,
                    "other_pos": (other_wp_at_t.x, other_wp_at_t.y, other_wp_at_t.z),
                    "distance_2d": dist_2d,
                    "distance_3d": dist_3d,
                    "type": conflict_type
                })
        
        current_time += time_resolution
        if current_time > check_end_time and current_time - time_resolution < check_end_time:
             current_time = check_end_time # Ensure the exact end time is checked
        elif current_time > check_end_time and check_start_time == check_end_time: # For instantaneous check
             break # Only one check needed


    # Optional: Post-process conflicts to merge continuous conflicts
    # For now, returns all discrete time-step conflicts
    return conflicts