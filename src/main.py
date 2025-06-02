from data_structures import PrimaryDroneMission, DroneMission, Waypoint
from conflict_checker import check_for_conflicts, ConflictInfo, \
                             MINIMUM_DISTANCE_THRESHOLD_2D, MINIMUM_DISTANCE_THRESHOLD_3D, \
                             VERTICAL_SEPARATION_THRESHOLD, TIME_STEP_RESOLUTION
from simulation_data import (
    get_sample_simulated_schedules_no_conflict,
    get_sample_simulated_schedules_with_conflict,
    get_stationary_conflict_schedule
)
from visualization import visualize_missions_static, animate_missions
from typing import List, Tuple

def deconfliction_query(
    primary_mission: PrimaryDroneMission,
    other_drone_schedules: List[DroneMission],
    safety_buffer_2d: float = MINIMUM_DISTANCE_THRESHOLD_2D,
    safety_buffer_3d: float = MINIMUM_DISTANCE_THRESHOLD_3D,
    vertical_sep: float = VERTICAL_SEPARATION_THRESHOLD,
    time_res: float = TIME_STEP_RESOLUTION
) -> Tuple[str, List[ConflictInfo]]:
    """
    Accepts the primary drone's mission and simulated flight schedules,
    returns a status ("clear" or "conflict detected") and conflict details.
    """
    conflicts = check_for_conflicts(
        primary_mission,
        other_drone_schedules,
        safety_buffer_2d,
        safety_buffer_3d,
        vertical_sep,
        time_res
    )
    
    if not conflicts:
        return "clear", []
    else:
        return "conflict detected", conflicts

def print_conflict_details(conflicts: List[ConflictInfo]):
    if not conflicts:
        print("No conflicts detected.")
        return
    
    print("\nConflict Details:")
    for i, c in enumerate(conflicts):
        p_pos_str = f"(x={c['primary_pos'][0]:.2f}, y={c['primary_pos'][1]:.2f}"
        if c['primary_pos'][2] is not None:
            p_pos_str += f", z={c['primary_pos'][2]:.2f})"
        else:
            p_pos_str += ")"

        o_pos_str = f"(x={c['other_pos'][0]:.2f}, y={c['other_pos'][1]:.2f}"
        if c['other_pos'][2] is not None:
            o_pos_str += f", z={c['other_pos'][2]:.2f})"
        else:
            o_pos_str += ")"

        dist_3d_str = f"{c['distance_3d']:.2f}m" if c['distance_3d'] is not None else "N/A"

        print(f"  Conflict {i+1}:")
        print(f"    Time: {c['time']:.2f}s")
        print(f"    Type: {c['type']}")
        print(f"    Primary ({c['primary_drone_id']}) Pos: {p_pos_str}")
        print(f"    Other ({c['conflicting_drone_id']}) Pos: {o_pos_str}")
        print(f"    Distance (2D): {c['distance_2d']:.2f}m, Distance (3D): {dist_3d_str}")
    print("-" * 30)


def run_scenario(
    scenario_name: str,
    primary_mission_params: dict,
    other_schedules_func, # A function that returns List[DroneMission]
    visualize: bool = True,
    animate: bool = True
):
    print(f"\n--- Running Scenario: {scenario_name} ---")
    
    primary_mission = PrimaryDroneMission(
        drone_id=primary_mission_params.get("drone_id", "PrimaryAlpha"),
        waypoint_coords=primary_mission_params["coords"],
        mission_overall_start_time=primary_mission_params["start_time"],
        mission_overall_end_time=primary_mission_params["end_time"]
    )
    print(f"Primary Mission: {primary_mission}")

    other_schedules = other_schedules_func()
    print("Other Drone Schedules:")
    for om in other_schedules:
        print(f"  - {om}")

    status, conflict_details = deconfliction_query(primary_mission, other_schedules)
    
    print(f"\nDeconfliction Status for '{scenario_name}': {status.upper()}")
    if status == "conflict detected":
        print_conflict_details(conflict_details)
    
    if visualize:
        visualize_missions_static(
            primary_mission, 
            other_schedules, 
            conflict_details, 
            title=f"{scenario_name} Overview",
            filename_suffix=scenario_name.lower().replace(" ", "_")
        )
    if animate:
        animate_missions(
            primary_mission,
            other_schedules,
            conflict_details,
            title=f"{scenario_name} Animation",
            filename_suffix=scenario_name.lower().replace(" ", "_")
        )
    print(f"--- End of Scenario: {scenario_name} ---\n")
    return status, conflict_details


if __name__ == "__main__":
    # --- Scenario 1: Clear Flight (2D primary) ---
    primary_2d_clear_params = {
        "coords": [(0,0), (100,100)],
        "start_time": 0.0,
        "end_time": 10.0,
        "drone_id": "Primary2D_Clear"
    }
    run_scenario(
        "2D Clear Flight",
        primary_2d_clear_params,
        get_sample_simulated_schedules_no_conflict,
        visualize=True, animate=True # Set animate to False if ffmpeg/Pillow issues
    )

    # --- Scenario 2: Conflict Flight (2D primary, mix of other drones) ---
    primary_2d_conflict_params = {
        "coords": [(0,50), (100,50)], # Straight line along y=50
        "start_time": 0.0,
        "end_time": 10.0,
        "drone_id": "Primary2D_Conflict"
    }
    run_scenario(
        "2D Conflict Flight",
        primary_2d_conflict_params,
        get_sample_simulated_schedules_with_conflict,
        visualize=True, animate=True
    )

    # --- Scenario 3: Clear Flight (3D primary) ---
    primary_3d_clear_params = {
        "coords": [(0,0,10), (100,100,25)], # Diagonal climb
        "start_time": 0.0,
        "end_time": 10.0,
        "drone_id": "Primary3D_Clear"
    }
    run_scenario(
        "3D Clear Flight",
        primary_3d_clear_params,
        get_sample_simulated_schedules_no_conflict, # Using no_conflict, but primary is now 3D
        visualize=True, animate=True
    )

    # --- Scenario 4: Conflict Flight (3D primary) ---
    primary_3d_conflict_params = {
        "coords": [(0,0,10), (100,100,15)], # Diagonal, relatively low climb
        "start_time": 0.0,
        "end_time": 10.0,
        "drone_id": "Primary3D_Conflict"
    }
    run_scenario(
        "3D Conflict Flight",
        primary_3d_conflict_params,
        get_sample_simulated_schedules_with_conflict, # These schedules have 3D elements
        visualize=True, animate=True
    )
    
    # --- Scenario 5: Stationary Conflict (Primary moves through stationary drone zone) ---
    primary_stationary_conflict_params = {
        "coords": [(0,50,10), (100,50,10)], # Primary flies straight at Z=10
        "start_time": 0.0,
        "end_time": 10.0,
        "drone_id": "Primary_Vs_Stationary"
    }
    run_scenario(
        "Stationary Conflict",
        primary_stationary_conflict_params,
        get_stationary_conflict_schedule, # DroneS is at (50,50,10) from t=0 to t=10
        visualize=True, animate=True
    )

    # --- Scenario 6: Single Point Primary Mission (Stationary Primary) ---
    # Primary is stationary, check against moving drones
    primary_single_point_params = {
        "coords": [(50, 50, 10)], # Stays at (50,50,10)
        "start_time": 0.0,
        "end_time": 10.0, # For this duration
        "drone_id": "Primary_Stationary"
    }
    run_scenario(
        "Single Point Primary (Stationary)",
        primary_single_point_params,
        get_sample_simulated_schedules_with_conflict, # DroneX will pass through (50,50)
        visualize=True, animate=True
    )