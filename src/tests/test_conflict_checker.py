import pytest
import numpy.testing as npt
from data_structures import Waypoint, DroneMission, PrimaryDroneMission
from conflict_checker import get_drone_position_at_time, check_for_conflicts, calculate_distance

# --- Test get_drone_position_at_time ---
@pytest.fixture
def sample_mission_2d():
    return DroneMission(
        waypoints=[Waypoint(0,0,0), Waypoint(100,0,10)], # Moves 10 units/sec in x
        drone_id="Test2D"
    )

@pytest.fixture
def sample_mission_3d():
    return DroneMission(
        waypoints=[Waypoint(0,0,0, z=10), Waypoint(100,0,10, z=30)], # Moves 10 units/sec in x, 2 units/sec in z
        drone_id="Test3D"
    )

def test_get_position_at_time_start(sample_mission_2d, sample_mission_3d):
    pos2d = get_drone_position_at_time(sample_mission_2d, 0)
    assert pos2d is not None
    npt.assert_allclose((pos2d.x, pos2d.y, pos2d.z), (0,0,None))

    pos3d = get_drone_position_at_time(sample_mission_3d, 0)
    assert pos3d is not None
    npt.assert_allclose((pos3d.x, pos3d.y, pos3d.z), (0,0,10))

def test_get_position_at_time_mid(sample_mission_2d, sample_mission_3d):
    pos2d = get_drone_position_at_time(sample_mission_2d, 5) # Halfway
    assert pos2d is not None
    npt.assert_allclose((pos2d.x, pos2d.y, pos2d.z), (50,0,None))

    pos3d = get_drone_position_at_time(sample_mission_3d, 5) # Halfway
    assert pos3d is not None
    npt.assert_allclose((pos3d.x, pos3d.y, pos3d.z), (50,0,20)) # (10+30)/2 = 20

def test_get_position_at_time_end(sample_mission_2d, sample_mission_3d):
    pos2d = get_drone_position_at_time(sample_mission_2d, 10)
    assert pos2d is not None
    npt.assert_allclose((pos2d.x, pos2d.y, pos2d.z), (100,0,None))

    pos3d = get_drone_position_at_time(sample_mission_3d, 10)
    assert pos3d is not None
    npt.assert_allclose((pos3d.x, pos3d.y, pos3d.z), (100,0,30))

def test_get_position_at_time_outside_bounds(sample_mission_2d):
    assert get_drone_position_at_time(sample_mission_2d, -1) is None
    assert get_drone_position_at_time(sample_mission_2d, 11) is None
    
def test_get_position_at_time_stationary():
    # DroneMission with two identical waypoints for duration
    mission_stat = DroneMission(
        waypoints=[Waypoint(10,10,0, z=5), Waypoint(10,10,5, z=5)], 
        drone_id="Stationary"
    )
    pos_stat_mid = get_drone_position_at_time(mission_stat, 2.5)
    assert pos_stat_mid is not None
    npt.assert_allclose((pos_stat_mid.x, pos_stat_mid.y, pos_stat_mid.z), (10,10,5))

    pos_stat_end = get_drone_position_at_time(mission_stat, 5)
    assert pos_stat_end is not None
    npt.assert_allclose((pos_stat_end.x, pos_stat_end.y, pos_stat_end.z), (10,10,5))

    # PrimaryDroneMission, single point defines stationary
    p_mission_stat = PrimaryDroneMission([(10,10,5)], 0, 5, "P_Stationary")
    pos_p_stat_mid = get_drone_position_at_time(p_mission_stat, 2.5)
    assert pos_p_stat_mid is not None
    npt.assert_allclose((pos_p_stat_mid.x, pos_p_stat_mid.y, pos_p_stat_mid.z), (10,10,5))


# --- Test calculate_distance ---
def test_calculate_distance():
    wpA1 = Waypoint(0,0,0)
    wpA2 = Waypoint(3,4,0) # 2D distance 5
    dist2d_A, dist3d_A = calculate_distance(wpA1, wpA2)
    assert dist2d_A == pytest.approx(5.0)
    assert dist3d_A is None

    wpB1 = Waypoint(0,0,0, z=0)
    wpB2 = Waypoint(3,4,0, z=12) # 3D distance sqrt(3^2+4^2+12^2) = sqrt(25+144) = sqrt(169) = 13
    dist2d_B, dist3d_B = calculate_distance(wpB1, wpB2)
    assert dist2d_B == pytest.approx(5.0)
    assert dist3d_B == pytest.approx(13.0)

    wpC1 = Waypoint(0,0,0, z=10) # One 3D, one 2D
    wpC2 = Waypoint(1,1,0)
    dist2d_C, dist3d_C = calculate_distance(wpC1, wpC2)
    assert dist2d_C == pytest.approx(npt.sqrt(1**2 + 1**2))
    assert dist3d_C is None # Because wpC2 is 2D

# --- Test check_for_conflicts ---
@pytest.fixture
def primary_straight_2d():
    # (0,0)@t=0 to (100,0)@t=10
    return PrimaryDroneMission([(0,0),(100,0)], 0, 10, "P_Straight2D")

@pytest.fixture
def primary_straight_3d():
    # (0,0,10)@t=0 to (100,0,10)@t=10
    return PrimaryDroneMission([(0,0,10),(100,0,10)], 0, 10, "P_Straight3D")

def test_no_conflict_far_apart(primary_straight_2d):
    other_far = DroneMission("OtherFar", [Waypoint(0,100,0), Waypoint(100,100,10)]) # Parallel, y=100
    conflicts = check_for_conflicts(primary_straight_2d, [other_far], safety_buffer_2d=10)
    assert not conflicts

def test_no_conflict_different_times(primary_straight_2d):
    other_late = DroneMission("OtherLate", [Waypoint(0,0,20), Waypoint(100,0,30)]) # Same path, but t=20-30
    conflicts = check_for_conflicts(primary_straight_2d, [other_late], safety_buffer_2d=10)
    assert not conflicts

def test_head_on_conflict_2d(primary_straight_2d):
    # Other: (100,0)@t=0 to (0,0)@t=10 (head-on)
    other_head_on = DroneMission("OtherHeadOn", [Waypoint(100,0,0), Waypoint(0,0,10)])
    conflicts = check_for_conflicts(primary_straight_2d, [other_head_on], safety_buffer_2d=10, time_resolution=0.1)
    assert conflicts
    # Conflict should occur around t=5 at x=50
    assert any(abs(c['time'] - 5.0) < 0.1 and abs(c['primary_pos'][0] - 50.0) < 1.0 for c in conflicts)
    assert conflicts[0]['type'] == "2D proximity"

def test_crossing_conflict_2d():
    primary = PrimaryDroneMission([(0,50),(100,50)], 0, 10, "PX_Cross") # Moves along y=50
    other = DroneMission("OY_Cross", [Waypoint(50,0,0), Waypoint(50,100,10)]) # Moves along x=50
    # They cross at (50,50) at t=5
    conflicts = check_for_conflicts(primary, [other], safety_buffer_2d=5, time_resolution=0.1)
    assert conflicts
    assert any(
        abs(c['time'] - 5.0) < 0.1 and \
        abs(c['primary_pos'][0] - 50.0) < 1.0 and \
        abs(c['primary_pos'][1] - 50.0) < 1.0
        for c in conflicts
    )

def test_3d_conflict_direct(primary_straight_3d): # Primary at z=10
    # Other drone at same path and time, but z=15 (within 15m 3D buffer, if vertical sep is large enough)
    other_close_3d = DroneMission("Other3DClose", [Waypoint(0,0,0, z=15), Waypoint(100,0,10, z=15)])
    # safety_buffer_3d=10, vertical_sep_threshold=3. Here, vertical sep is 5.
    # Direct 3D distance at t=5 (primary (50,0,10), other (50,0,15)) is 5.
    # If safety_buffer_3d is 7, this is a conflict.
    conflicts = check_for_conflicts(primary_straight_3d, [other_close_3d], 
                                    safety_buffer_3d=7, vertical_sep_threshold=3, time_resolution=0.1)
    assert conflicts
    assert conflicts[0]['type'] == "3D proximity"
    assert conflicts[0]['distance_3d'] == pytest.approx(5.0)


def test_3d_conflict_vertical_separation(primary_straight_3d): # Primary at z=10
    # Other drone horizontally very close (e.g. x=1 offset), but z=12.
    # Primary (50,0,10) at t=5. Other (51,0,12) at t=5.
    # Horizontal distance = 1. Vertical distance = 2.
    other_vert_sep = DroneMission("OtherVertSep", [Waypoint(1,0,0, z=12), Waypoint(101,0,10, z=12)])
    
    # Scenario 1: Vertical separation IS the issue
    # safety_buffer_2d=5, safety_buffer_3d=10, vertical_sep_threshold=3
    # Horizontal dist (1) < safety_buffer_2d (5). Vertical dist (2) < vertical_sep_threshold (3).
    # 3D distance = sqrt(1^2 + 2^2) = sqrt(5) = 2.23. This is < safety_buffer_3d (10)
    # So it should be a "3D proximity" conflict first.
    conflicts1 = check_for_conflicts(primary_straight_3d, [other_vert_sep], 
                                     safety_buffer_2d=5, safety_buffer_3d=10, 
                                     vertical_sep_threshold=3, time_resolution=0.1)
    assert conflicts1
    assert conflicts1[0]['type'] == "3D proximity" # Because 3D dist is less than 3D buffer
    assert conflicts1[0]['distance_3d'] == pytest.approx(npt.sqrt(1**2 + 2**2))


    # Scenario 2: Vertical separation IS the issue when 3D buffer is met but vert sep is not
    # safety_buffer_2d=5, safety_buffer_3d=2 (very small), vertical_sep_threshold=3
    # Primary (50,0,10), Other (51,0,12) at t=5. Horiz_dist=1, Vert_dist=2. 3D_dist=sqrt(5)=2.23.
    # 3D_dist (2.23) > safety_buffer_3d (2). So NO "3D proximity".
    # BUT horiz_dist (1) < safety_buffer_2d (5) AND vert_dist (2) < vertical_sep_threshold (3).
    # So, this should be an "Insufficient vertical separation" conflict.
    conflicts2 = check_for_conflicts(primary_straight_3d, [other_vert_sep], 
                                     safety_buffer_2d=5, safety_buffer_3d=2.0, # 3D buffer met
                                     vertical_sep_threshold=3, time_resolution=0.1)
    assert conflicts2
    assert conflicts2[0]['type'] == "Insufficient vertical separation"