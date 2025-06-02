# src/interface.py

from src.data_structures import Mission2D
from src.conflict_checker import detect_conflicts_2d, ConflictReport2D

def check_mission(primary: Mission2D, simulated_flights: List[Mission2D], safety_buffer: float = 5.0, dt: float = 0.1) -> ConflictReport2D:
    """
    Query interface for external users:
      - primary: Mission2D for the primary drone
      - simulated_flights: list of Mission2D for other drones
      - safety_buffer: minimum allowed separation (default 5.0 m)
      - dt: time resolution for sampling (default 0.1 s)
    Returns a ConflictReport2D with status and details.
    """
    return detect_conflicts_2d(primary, simulated_flights, safety_buffer, dt)
