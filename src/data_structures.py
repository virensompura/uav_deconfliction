from typing import List, Optional, Tuple, Union

class Waypoint:
    """Represents a single point in space and time for a drone's trajectory."""
    def __init__(self, x: float, y: float, timestamp: float, z: Optional[float] = None):
        self.x = float(x)
        self.y = float(y)
        self.timestamp = float(timestamp)
        self.z = float(z) if z is not None else None # Optional for 3D

    def __repr__(self) -> str:
        if self.z is not None:
            return f"Waypoint(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, t={self.timestamp:.2f})"
        return f"Waypoint(x={self.x:.2f}, y={self.y:.2f}, t={self.timestamp:.2f})"

    @property
    def position(self) -> Tuple[float, float, Optional[float]]:
        """Returns the spatial coordinates (x, y, z)."""
        return (self.x, self.y, self.z)

    @property
    def is_3d(self) -> bool:
        return self.z is not None

class DroneMission:
    """Represents a drone's mission defined by a series of waypoints."""
    def __init__(self, waypoints: List[Waypoint], drone_id: str):
        if not waypoints:
            raise ValueError(f"DroneMission for {drone_id} must have at least one waypoint.")
        # Ensure waypoints are sorted by timestamp
        self.waypoints = sorted(waypoints, key=lambda wp: wp.timestamp)
        self.drone_id = drone_id

    def get_start_time(self) -> float:
        """Returns the timestamp of the first waypoint."""
        return self.waypoints[0].timestamp

    def get_end_time(self) -> float:
        """Returns the timestamp of the last waypoint."""
        return self.waypoints[-1].timestamp

    def is_mission_3d(self) -> bool:
        """Checks if any waypoint in the mission has a Z coordinate."""
        return any(wp.is_3d for wp in self.waypoints)

    def __repr__(self) -> str:
        return (f"DroneMission(id='{self.drone_id}', "
                f"waypoints_count={len(self.waypoints)}, "
                f"start_t={self.get_start_time():.2f}, end_t={self.get_end_time():.2f}, "
                f"is_3d={self.is_mission_3d()})")

class PrimaryDroneMission(DroneMission):
    """
    Represents the primary drone's mission.
    Timestamps for waypoints are calculated based on an overall mission window.
    """
    def __init__(self,
                 # List of (x,y) or (x,y,z) coordinates
                 waypoint_coords: List[Union[Tuple[float, float], Tuple[float, float, float]]],
                 mission_overall_start_time: float,
                 mission_overall_end_time: float,
                 drone_id: str = "PrimaryDrone"):

        if not waypoint_coords:
            raise ValueError("Primary mission must have at least one waypoint coordinate.")
        if mission_overall_end_time < mission_overall_start_time:
            raise ValueError("Mission overall end time must be after start time.")

        self.mission_overall_start_time = float(mission_overall_start_time)
        self.mission_overall_end_time = float(mission_overall_end_time)
        
        num_coords = len(waypoint_coords)
        waypoints: List[Waypoint] = []

        if num_coords == 1:
            # Single point mission: drone stays at this point for the duration
            coord = waypoint_coords[0]
            is_3d_coord = len(coord) == 3
            alt = coord[2] if is_3d_coord else None
            # Create two waypoints for a stationary mission: start and end at the same place
            waypoints.append(Waypoint(coord[0], coord[1], self.mission_overall_start_time, alt))
            if self.mission_overall_start_time != self.mission_overall_end_time:
                 waypoints.append(Waypoint(coord[0], coord[1], self.mission_overall_end_time, alt))

        else:
            total_mission_duration = self.mission_overall_end_time - self.mission_overall_start_time
            num_segments = num_coords - 1
            
            time_per_segment = total_mission_duration / num_segments if num_segments > 0 else 0
            
            current_time = self.mission_overall_start_time
            for i, coord_tuple in enumerate(waypoint_coords):
                is_3d_coord = len(coord_tuple) == 3
                x, y = coord_tuple[0], coord_tuple[1]
                z = coord_tuple[2] if is_3d_coord else None
                
                # For the last waypoint, ensure it's exactly at mission_overall_end_time
                if i == num_coords - 1:
                    actual_time = self.mission_overall_end_time
                else:
                    actual_time = self.mission_overall_start_time + i * time_per_segment
                
                # Prevent duplicate timestamps if time_per_segment is 0 (e.g. start_time == end_time)
                if waypoints and waypoints[-1].timestamp == actual_time and waypoints[-1].x == x and waypoints[-1].y == y and waypoints[-1].z == z:
                    continue

                waypoints.append(Waypoint(x, y, actual_time, z))
        
        super().__init__(waypoints, drone_id)

        # Final check on generated waypoints for primary mission
        if self.waypoints[-1].timestamp > self.mission_overall_end_time + 1e-6: # Epsilon for float comparison
            # This might happen if calculated segment times don't perfectly sum up, adjust last wp
            self.waypoints[-1].timestamp = self.mission_overall_end_time
            # Re-sort if the last timestamp adjustment caused an issue (shouldn't usually)
            self.waypoints = sorted(self.waypoints, key=lambda wp: wp.timestamp)

        if not self.waypoints: # Should be caught earlier, but as a safeguard
            raise ValueError(f"PrimaryDroneMission for {drone_id} ended up with no waypoints.")