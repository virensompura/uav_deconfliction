# UAV Strategic Deconfliction in Shared Airspace

## Project Overview

This project implements a strategic deconfliction system for Unmanned Aerial Vehicles (UAVs) or drones. The system serves as a final authority to verify whether a primary drone's planned waypoint mission is safe to execute in shared airspace, by checking for potential conflicts (in both space and time) against the simulated flight paths of multiple other drones.

The system supports:
-   2D and 3D (extra credit) mission planning for the primary drone.
-   Simulation of other drones with their own flight schedules.
-   Spatial checks against a defined safety buffer.
-   Temporal checks for overlapping presence in the same spatial area.
-   Detailed conflict explanation, including location, time, and involved drones.
-   Static and animated visualizations of drone trajectories and identified conflicts.

## Features

-   **Data Structures:** Defines `Waypoint`, `DroneMission`, and `PrimaryDroneMission` to represent flight plans.
-   **Conflict Checking:**
    -   Calculates drone positions at discrete time steps using linear interpolation.
    -   Checks for violations of 2D and 3D safety buffers.
    -   Considers vertical separation for 3D conflicts.
-   **Query Interface:** A Python function `deconfliction_query` that returns "clear" or "conflict detected" along with details.
-   **Visualization:**
    -   Static plots showing all drone paths and highlighted conflict points (saved as PNG).
    -   Animated simulations of drone movements over time, highlighting conflicts as they occur (saved as MP4 or GIF).
-   **Scenario Management:** The `main.py` script runs several pre-defined scenarios, including:
    -   2D and 3D conflict-free missions.
    -   2D and 3D missions with various conflict types (head-on, crossing, insufficient separation).
    -   Conflicts with stationary drones.
-   **Testing:** Includes unit tests for data structures and core conflict checking logic using `pytest`.

## Project Structure

```text
uav_deconfliction/
├── .gitignore # Specifies intentionally untracked files that Git should ignore
├── requirements.txt # Lists project dependencies
├── data_structures.py # Defines Waypoint, DroneMission, PrimaryDroneMission classes
├── conflict_checker.py # Contains logic for conflict detection and position interpolation
├── simulation_data.py # Provides sample flight schedules for simulated drones
├── visualization.py # Handles static and animated plotting of missions and conflicts
├── main.py # Main executable script to run deconfliction scenarios
├── tests/ # Directory for automated tests
│ ├── init.py
│ ├── test_data_structures.py
│ └── test_conflict_checker.py
└── outputs/ # Directory where generated plots and animations are saved (created automatically)
```

---


## Setup Instructions

1.  **Clone the Repository (if applicable):**
    ```bash
    git clone https://github.com/virensompura/uav_deconfliction.git
    cd uav_deconfliction
    ```

2.  **Create a Virtual Environment:**
    It's highly recommended to use a virtual environment to manage project dependencies.
    ```bash
    python3 -m venv uav_env
    ```
    *(Note: You can name your virtual environment anything, e.g., `venv`, `uav_env`)*

3.  **Activate the Virtual Environment:**
    -   On macOS and Linux:
        ```bash
        source uav_env/bin/activate
        ```
    -   On Windows (Git Bash or similar):
        ```bash
        source uav_env/Scripts/activate
        ```
    -   On Windows (Command Prompt):
        ```bash
        uav_env\Scripts\activate.bat
        ```
    Your command prompt should now indicate that the virtual environment is active (e.g., `(uav_env)`).

4.  **Install Dependencies:**
    Navigate to the project's root directory (the one containing `requirements.txt`) if you aren't already there.
    ```bash
    pip install -r requirements.txt
    ```
    This will install `numpy`, `matplotlib`, and `pytest`.

5.  **Install FFmpeg (Optional, for MP4 Animations):**
    For saving animations as MP4 files, `ffmpeg` is required. If not installed, the system will attempt to save animations as GIFs (which requires `Pillow`, installed via `matplotlib`).
    -   **Windows:** Download from [ffmpeg.org](https://ffmpeg.org/download.html), extract, and add the `bin` directory to your system's PATH environment variable.
    -   **macOS (using Homebrew):** `brew install ffmpeg`
    -   **Linux (e.g., Ubuntu/Debian):** `sudo apt update && sudo apt install ffmpeg`

## How to Run

1.  **Ensure your virtual environment is activated.**
2.  **Navigate to the `src/` directory** (if your `main.py` and other Python files are in a `src` subdirectory as per your previous example, otherwise adjust the path).
    If your `main.py` is in the root `uav_deconfliction` directory, you can skip the `cd src/` step.
    *Based on your last traceback, your files seem to be in a `src` folder:*
    ```bash
    cd src 
    ```
    *(If `main.py` is in the root, you'd just be in the `uav_deconfliction` directory)*

3.  **Run the Main Simulation Script:**
    ```bash
    python main.py
    ```
    The script will execute several pre-defined scenarios. For each scenario, it will:
    -   Print the primary mission and other drone schedules.
    -   Output the deconfliction status ("CLEAR" or "CONFLICT DETECTED").
    -   If conflicts are detected, print detailed information about each conflict.
    -   Generate and save a static plot (`.png`) and an animation (`.mp4` or `.gif`) to the `outputs/` directory.

## How to Run Tests

1.  **Ensure your virtual environment is activated.**
2.  **Navigate to the project's root directory** (the one containing the `tests/` directory and `pytest.ini` if you add one).
3.  **Run Pytest:**
    ```bash
    pytest
    ```
    This will discover and run all tests in the `tests/` directory.

## Key Configuration Constants

The following constants can be adjusted in `conflict_checker.py`:

-   `MINIMUM_DISTANCE_THRESHOLD_2D`: Minimum horizontal separation required (meters).
-   `MINIMUM_DISTANCE_THRESHOLD_3D`: Minimum 3D slant range separation required (meters).
-   `VERTICAL_SEPARATION_THRESHOLD`: Minimum vertical separation required if drones are horizontally close but meet the 3D slant range (meters).
-   `TIME_STEP_RESOLUTION`: The time increment (seconds) used for discretizing and checking drone positions. Smaller values increase accuracy but also computation time.

## Reflection & Justification

This section outlines the design decisions, architectural choices, AI integration, testing strategies, and scalability considerations for the UAV Strategic Deconfliction system.

### Design Decisions & Architectural Choices

1.  **Modularity:** The system was designed with modularity in mind, separating concerns into distinct Python files:
    *   `data_structures.py`: Core entities like `Waypoint`, `DroneMission`. This promotes reusability and clarity.
    *   `conflict_checker.py`: Encapsulates all logic related to identifying conflicts, including position interpolation and distance calculations. This isolates the core algorithm.
    *   `visualization.py`: Handles all graphical output, keeping presentation logic separate from the core deconfliction engine.
    *   `simulation_data.py`: Provides sample data, making it easy to define and switch between test scenarios.
    *   `main.py`: Acts as the orchestrator and entry point, demonstrating various scenarios.
    This separation improves maintainability, testability, and understandability.

2.  **Data Representation:**
    *   `Waypoint` objects form the fundamental building blocks, storing spatial coordinates (x, y, and optional z) and an absolute timestamp.
    *   `DroneMission` aggregates `Waypoint` objects. For the `PrimaryDroneMission`, timestamps are automatically calculated based on an overall mission window and an assumption of constant speed between waypoints, simplifying mission definition. Other drones have explicitly defined waypoints with timestamps.

3.  **Conflict Detection Algorithm:**
    *   **Time Discretization:** The core conflict detection relies on stepping through time at a defined `TIME_STEP_RESOLUTION`. At each step, the positions of all relevant drones are calculated.
    *   **Linear Interpolation:** Drone positions between waypoints are determined using linear interpolation. This is a common and computationally efficient approach for path approximation when detailed flight dynamics are not a primary concern for strategic deconfliction.
    *   **Safety Buffers:** Conflicts are flagged if the distance between drones falls below predefined 2D or 3D safety thresholds. An additional check for vertical separation is included for 3D missions to flag situations where drones are horizontally close but might otherwise meet the 3D slant range buffer if vertical separation is insufficient.
    *   **Simplicity vs. Completeness:** The time-stepping approach was chosen for its implementation simplicity and effectiveness for strategic (pre-flight) checks. While it might miss very brief conflicts occurring between time steps if the resolution is too coarse, it's a practical trade-off for this assignment.

4.  **Interface:** A simple Python function `deconfliction_query` serves as the main interface, returning a clear status and detailed conflict information. This allows for easy integration into other systems or scripts.

### Spatial and Temporal Checks Implementation

-   **`get_drone_position_at_time(mission, time_t)`:**
    -   This function is crucial. It first checks if the query time `time_t` is within the mission's operational window.
    -   It then iterates through the mission's waypoints to find the two waypoints (segment) that bracket `time_t`.
    -   Linear interpolation is applied to the x, y, (and z, if applicable) coordinates based on the fraction of time elapsed within that segment.
    -   Edge cases like `time_t` being exactly on a waypoint, or before the start/after the end of the mission, are handled. For stationary drones (single waypoint or identical start/end waypoints in a segment), it returns the constant position.
-   **`check_for_conflicts(...)`:**
    -   The function iterates from the primary mission's start time to its end time, incrementing by `TIME_STEP_RESOLUTION`.
    -   In each iteration:
        1.  The primary drone's position is calculated using `get_drone_position_at_time`.
        2.  For every other simulated drone, its position is also calculated for the same `current_time`.
        3.  If both drones are active (i.e., their positions are valid):
            *   The 2D horizontal distance and, if applicable, the full 3D distance are computed using `calculate_distance`.
            *   **Spatial Check:** The distances are compared against `MINIMUM_DISTANCE_THRESHOLD_2D` and `MINIMUM_DISTANCE_THRESHOLD_3D`.
            *   **Temporal Check:** This is inherently combined with the spatial check. A conflict is only registered if the spatial proximity occurs *at the same `current_time` step*, implying temporal overlap of their presence in the conflicting space.
            *   For 3D, an additional check ensures adequate `VERTICAL_SEPARATION_THRESHOLD` if drones are horizontally close but their 3D slant distance might otherwise seem safe.
        4.  If a conflict is found, a `ConflictInfo` dictionary is created with details (time, locations, involved drones, distances, conflict type).

### AI Integration *(Customize this heavily based on your actual usage)*

Throughout the development process, AI-assisted tools like **GitHub Copilot (integrated into VS Code)** and occasionally **Claude 3 Opus (via its web interface)** were leveraged to expedite work and explore solutions.

1.  **Boilerplate Code Generation:**
    *   **Copilot:** Used extensively for generating initial class structures (e.g., `Waypoint`, `DroneMission`), basic function signatures, and repetitive code blocks like `__init__` methods or `__repr__` methods.
        *   *Prompt Example (Implicit, via context):* Typing `class Waypoint:` and then starting `def __init__(self, x, y, timestamp, z=None):` would often trigger Copilot to suggest the entire `self.x = x ...` block.
        *   *Evaluation:* This significantly sped up initial setup. The generated code was generally correct for simple cases but always reviewed and often adjusted for type hinting stringency, default values, or specific logic nuances.
    *   **Claude:** Used for generating more complex function skeletons.
        *   *Prompt Example for `get_drone_position_at_time` (to Claude):* "Write a Python function that takes a list of waypoints (each with x, y, z, timestamp) and a target time. It should interpolate the drone's position (x, y, z) at that target time. Handle cases where the time is outside the mission or on a waypoint."
        *   *Evaluation:* Claude provided a good starting point for the interpolation logic. However, its initial version needed refinement for handling 2D vs. 3D waypoints consistently (e.g., when `z` is `None`), precise handling of boundary conditions (exactly at start/end times), and ensuring the returned object was a `Waypoint` with the *query time* as its timestamp.

2.  **Algorithm Brainstorming & Implementation Ideas:**
    *   **Copilot/Claude for Matplotlib:** When implementing `visualization.py`:
        *   *Prompt Example (Conceptual, for research/Claude):* "How to create a 3D animation of moving points in Matplotlib with static trajectory lines?"
        *   *Copilot (Contextual):* While writing the `animate_missions` function, Copilot suggested snippets for setting up `FuncAnimation`, updating artist data, and structuring the `update` function.
        *   *Evaluation:* AI suggestions were very helpful for Matplotlib, which has a complex API. Copilot often provided correct boilerplate for `FuncAnimation`. However, specific logic for updating multiple drones, dynamically setting plot limits, handling 2D/3D switching, and correctly managing artist visibility needed manual implementation and debugging. The `blit=True` optimization often required careful return value management from the `update` function, which AI didn't always get perfect.

3.  **Debugging Assistance:**
    *   **Claude:** When encountering tracebacks or unexpected behavior, pasting the code snippet and the error message into Claude often yielded explanations or suggestions for fixes.
        *   *Prompt Example:* "My Matplotlib animation shows an error: 'ValueError: The truth value of an array with more than one element is ambiguous.' Here's the relevant code: [code snippet for scatter plot color]. What's causing this and how can I fix it?"
        *   *Evaluation:* Claude correctly identified the issue with `color or 'black'` when `color` was a NumPy array and suggested using `isinstance` or checking if `color is None`, which led to the correct fix. This was a significant time saver.

4.  **Test Case Generation:**
    *   **Copilot:** While writing `pytest` files, Copilot suggested various test function names and basic assertion structures based on the function being tested.
        *   *Prompt Example (Implicit):* After writing `def test_no_conflict():`, Copilot would suggest scenarios like drones being far apart or at different times.
        *   *Evaluation:* Useful for generating a list of scenarios to cover. The actual test data (waypoint coordinates, timings) and specific assertion values always required manual definition to ensure correctness and target specific edge cases.

5.  **Documentation & README Generation:**
    *   **Claude/Copilot:** Used to generate initial drafts or sections of the README (like project structure or setup instructions based on existing code).
        *   *Evaluation:* Provided a good template, but required significant editing for accuracy, completeness, and to match the project's specific details.

**Critical Evaluation of AI Output:**
AI tools were accelerators, not replacements for critical thinking. Outputs were always treated as suggestions:
-   **Validation:** Code snippets were tested thoroughly.
-   **Refinement:** AI-generated code often needed adjustments for Pythonic style, adherence to project conventions, better error handling, or more robust logic for edge cases.
-   **Understanding:** It was crucial to understand *why* an AI suggestion worked or didn't, rather than blindly accepting it. This often involved consulting documentation or breaking down the problem further.

Overall, AI tools probably reduced development time by 20-30%, primarily by handling boilerplate, offering quick solutions to common problems (especially with Matplotlib), and assisting in debugging.

### Testing Strategy & Edge Cases

A combination of unit tests and scenario-based integration testing (via `main.py`) was employed.

1.  **Unit Tests (`pytest`):**
    *   Located in the `tests/` directory.
    *   `test_data_structures.py`: Validates the creation and basic properties of `Waypoint`, `DroneMission`, and `PrimaryDroneMission` (e.g., correct time distribution for primary missions, handling of 2D/3D waypoints, single waypoint missions, zero-duration missions).
    *   `test_conflict_checker.py`:
        *   Tests `get_drone_position_at_time` for various inputs: start/mid/end of segment, outside mission bounds, stationary drones.
        *   Tests `calculate_distance` for 2D and 3D cases.
        *   Tests `check_for_conflicts` with a variety of scenarios:
            *   **No Conflict:** Drones far apart spatially, or close spatially but at different times.
            *   **2D Conflicts:** Head-on, crossing paths.
            *   **3D Conflicts:** Direct proximity (violating 3D slant range), insufficient vertical separation when horizontally close.
            *   Missions with only one drone (no "other" drones to conflict with).

2.  **Edge Cases Considered:**
    *   Missions with a single waypoint (stationary drone).
    *   Missions with zero duration (primary mission start time equals end time).
    *   Waypoints with identical timestamps (drone hovering).
    *   Primary mission overall window being shorter than the time implied by its waypoints if they had explicit speeds (the current implementation recalculates primary waypoint times to fit the window).
    *   Empty list of other drones.
    *   Missions where one drone is 2D and the other is 3D.
    *   Conflict occurring exactly at the start or end of a mission segment or overall mission time.
    *   `TIME_STEP_RESOLUTION` impact (very small vs. very large).

3.  **Quality Assurance:**
    *   Iterative testing: Tests were run frequently during development.
    *   Code review (self-review in this solo project context) for logic and clarity.
    *   Visual validation: The plots and animations generated by `visualization.py` served as a crucial visual QA step to confirm that detected conflicts (or lack thereof) matched expectations based on the drone paths.
    *   Use of type hinting and linters (e.g., Pylint, Flake8, if configured) helps catch potential issues early.

### Scalability Discussion

The current system is designed for a small number of drones and strategic (pre-flight) deconfliction. Scaling to handle tens of thousands of commercial drones in real-time would require significant architectural changes:

1.  **Optimized Conflict Detection Algorithms:**
    *   **Geometric Algorithms:** Replace discrete time-stepping with continuous 4D (space-time) swept-volume intersection tests (e.g., intersection of 4D Minkowski sums of drone safety volumes and path segments). This is more complex but avoids missing conflicts between time steps and can be more efficient for certain scenarios.
    *   **Predictive Algorithms:** Incorporate uncertainty in drone navigation and predict future positions probabilistically.

2.  **Spatial Indexing:**
    *   Instead of pairwise checks between all drones (O(N^2) at each time step), use spatial data structures (e.g., k-d trees, R-trees, or grid-based partitioning like Geohashing) to quickly query for only nearby drones. This reduces the number of pairs to check significantly. Airspace could be divided into sectors.

3.  **Efficient Data Management & Ingestion:**
    *   **Real-time Data Pipelines:** For live drone tracking, use message queues like Apache Kafka or Pulsar to ingest high-frequency position updates.
    *   **Databases:**
        *   Store flight plans and airspace reservations in a database optimized for spatio-temporal queries (e.g., PostGIS extension for PostgreSQL, or specialized time-series databases).
        *   Use caching (e.g., Redis) for frequently accessed flight data or conflict hotspots.

4.  **Distributed Computing:**
    *   Distribute the conflict detection workload across multiple nodes using frameworks like Apache Spark, Dask, or Ray.
    *   Airspace could be partitioned, and each partition handled by a separate worker.

5.  **Asynchronous Processing & Microservices:**
    *   Deconfliction queries could be handled asynchronously.
    *   Break down the system into microservices (e.g., flight plan ingestion service, conflict detection service, notification service).

6.  **Dynamic Airspace Management:**
    *   Implement dynamic geofencing and no-fly zones.
    *   System for negotiating and allocating airspace reservations.

7.  **Advanced Conflict Resolution:**
    *   Beyond just detection, implement algorithms to suggest alternative routes, speeds, or departure times to resolve potential conflicts automatically or semi-automatically. This is a complex optimization problem.

8.  **Fault Tolerance and Resilience:**
    *   Redundancy in services and data storage.
    *   Robust error handling and retry mechanisms.
    *   Monitoring and alerting systems.

The current implementation's complexity is roughly O(P * M * (T_duration / T_resolution)), where P is the number of segments in the primary mission (for position calculation), M is the number of other drones, T_duration is the primary mission's time length, and T_resolution is the time step. For large M and fine T_resolution, this becomes computationally intensive.

## Future Enhancements (Potential Ideas)

Beyond the scalability improvements mentioned above, several other features could enhance the system:

1.  **Drone Performance Models:** Incorporate more realistic drone dynamics, such as acceleration/deceleration limits, turn radius, and battery constraints, instead of assuming constant speed and instant turns between waypoints.
2.  **Variable Speed Trajectories:** Allow primary mission definition to include desired speeds for different segments, rather than just distributing time equally.
3.  **Uncertainty Modeling:** Account for GPS inaccuracies or deviations from planned paths by expanding the safety buffer or using probabilistic conflict detection.
4.  **Weather Integration:** Consider weather conditions (e.g., wind) that might affect drone trajectories and safe operating envelopes.
5.  **Airspace Rules & Regulations:** Integrate a rules engine to check compliance with airspace regulations (e.g., altitude limits, no-fly zones, right-of-way rules).
6.  **User Interface (GUI/Web):** Develop a graphical user interface for easier mission planning, submission of deconfliction queries, and interactive visualization of results.
7.  **API for External Systems:** Provide a well-defined API (e.g., RESTful) for other drone management systems or flight planning tools to query the deconfliction service.
8.  **Conflict Resolution Advisor:** Implement algorithms to suggest minor adjustments to the primary mission (e.g., slight time shift, altitude change, minor re-route) to resolve detected conflicts.
9.  **Persistent Storage of Flight Plans:** Integrate with a database to store and manage flight plans and historical conflict data.
10. **Support for Different Deconfliction Strategies:**
    *   **Strategic:** Pre-flight planning (current focus).
    *   **Tactical:** In-flight, short-term conflict avoidance (would require much faster processing and direct communication capabilities).
11. **Enhanced 4D Visualization:** More interactive 4D (3D space + time slider) visualizations, potentially using libraries like Plotly Dash or web-based tools (e.g., CesiumJS).


## Contact

If you have questions, encounter issues, or want to discuss new features, please open an issue on GitHub or reach out to me :)

* **Viren Sompura** – [GitHub Profile](https://github.com/virensompura)
* **Email ID** – [Email](mailto:sompuraviren03@gmail.com)

Thank you for using and contributing to the UAV Deconfliction project!
