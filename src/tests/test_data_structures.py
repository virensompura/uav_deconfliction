import pytest
from data_structures import Waypoint, DroneMission, PrimaryDroneMission

def test_waypoint_creation():
    wp_2d = Waypoint(1.0, 2.0, 10.0)
    assert wp_2d.x == 1.0
    assert wp_2d.y == 2.0
    assert wp_2d.timestamp == 10.0
    assert wp_2d.z is None
    assert not wp_2d.is_3d
    assert repr(wp_2d) == "Waypoint(x=1.00, y=2.00, t=10.00)"

    wp_3d = Waypoint(1.0, 2.0, 10.0, 30.0)
    assert wp_3d.z == 30.0
    assert wp_3d.is_3d
    assert wp_3d.position == (1.0, 2.0, 30.0)
    assert repr(wp_3d) == "Waypoint(x=1.00, y=2.00, z=30.00, t=10.00)"

def test_drone_mission_creation():
    wp1 = Waypoint(0,0,0)
    wp2 = Waypoint(10,0,5)
    mission = DroneMission([wp2, wp1], "TestDrone1") # Test sorting
    assert mission.drone_id == "TestDrone1"
    assert len(mission.waypoints) == 2
    assert mission.waypoints[0].timestamp == 0
    assert mission.waypoints[1].timestamp == 5
    assert mission.get_start_time() == 0
    assert mission.get_end_time() == 5
    assert not mission.is_mission_3d()

    wp3d = Waypoint(0,0,0,z=10)
    mission_3d = DroneMission([wp3d], "TestDrone3D")
    assert mission_3d.is_mission_3d()

    with pytest.raises(ValueError):
        DroneMission([], "EmptyDrone")

def test_primary_drone_mission_time_distribution():
    # 2D mission
    coords_2d = [(0,0), (100,0), (100,100)] # 2 segments
    start_t, end_t = 0.0, 10.0
    p_mission_2d = PrimaryDroneMission(coords_2d, start_t, end_t, "P_2D")
    
    assert len(p_mission_2d.waypoints) == 3
    assert p_mission_2d.waypoints[0].x == 0 and p_mission_2d.waypoints[0].y == 0 and p_mission_2d.waypoints[0].timestamp == 0.0
    assert p_mission_2d.waypoints[1].x == 100 and p_mission_2d.waypoints[1].y == 0 and p_mission_2d.waypoints[1].timestamp == 5.0 # Midpoint time
    assert p_mission_2d.waypoints[2].x == 100 and p_mission_2d.waypoints[2].y == 100 and p_mission_2d.waypoints[2].timestamp == 10.0 # End time
    assert not p_mission_2d.is_mission_3d()

    # 3D mission
    coords_3d = [(0,0,10), (50,50,15), (100,0,20)] # 2 segments
    p_mission_3d = PrimaryDroneMission(coords_3d, start_t, end_t, "P_3D")
    assert len(p_mission_3d.waypoints) == 3
    assert p_mission_3d.waypoints[0].z == 10 and p_mission_3d.waypoints[0].timestamp == 0.0
    assert p_mission_3d.waypoints[1].z == 15 and p_mission_3d.waypoints[1].timestamp == 5.0
    assert p_mission_3d.waypoints[2].z == 20 and p_mission_3d.waypoints[2].timestamp == 10.0
    assert p_mission_3d.is_mission_3d()

def test_primary_drone_mission_single_waypoint():
    # Single waypoint means stationary for the duration
    coords_single = [(10,20,5)]
    start_t, end_t = 5.0, 15.0
    p_mission_single = PrimaryDroneMission(coords_single, start_t, end_t, "P_Single")
    assert len(p_mission_single.waypoints) == 2 # Start and end waypoints at same location
    assert p_mission_single.waypoints[0].x == 10 and p_mission_single.waypoints[0].y == 20 and p_mission_single.waypoints[0].z == 5
    assert p_mission_single.waypoints[0].timestamp == 5.0
    assert p_mission_single.waypoints[1].x == 10 and p_mission_single.waypoints[1].y == 20 and p_mission_single.waypoints[1].z == 5
    assert p_mission_single.waypoints[1].timestamp == 15.0
    assert p_mission_single.get_start_time() == 5.0
    assert p_mission_single.get_end_time() == 15.0

def test_primary_drone_mission_zero_duration():
    coords = [(0,0), (10,10)]
    start_t, end_t = 5.0, 5.0 # Zero duration
    p_mission_zero_dur = PrimaryDroneMission(coords, start_t, end_t, "P_ZeroDur")
    assert len(p_mission_zero_dur.waypoints) == 2
    assert p_mission_zero_dur.waypoints[0].timestamp == 5.0
    assert p_mission_zero_dur.waypoints[1].timestamp == 5.0
    assert p_mission_zero_dur.get_start_time() == 5.0
    assert p_mission_zero_dur.get_end_time() == 5.0

    coords_single_zero_dur = [(0,0,0)]
    p_mission_single_zero_dur = PrimaryDroneMission(coords_single_zero_dur, start_t, end_t, "P_Single_ZeroDur")
    assert len(p_mission_single_zero_dur.waypoints) == 1 # For single coord and zero duration, one WP
    assert p_mission_single_zero_dur.waypoints[0].timestamp == 5.0

    
with pytest.raises(ValueError):
    PrimaryDroneMission([], 0, 10, "P_Empty")

with pytest.raises(ValueError):
    PrimaryDroneMission([(0,0)], 10, 0, "P_InvalidTime") # End before start