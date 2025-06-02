import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D # For 3D plotting
import numpy as np
import os

from data_structures import DroneMission, Waypoint
from conflict_checker import get_drone_position_at_time, ConflictInfo # For types
from typing import List, Optional, Tuple


OUTPUT_DIR = "outputs"
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)


def plot_single_drone_path_static(ax, mission: DroneMission, color=None, label_prefix="", linestyle='-', marker='.', alpha=0.7):
    """Plots a single drone's waypoints and segments statically."""
    xs = [wp.x for wp in mission.waypoints]
    ys = [wp.y for wp in mission.waypoints]
    
    is_3d_mission = mission.is_mission_3d()
    if is_3d_mission:
        # If any waypoint has z, all must be treated as 3D, defaulting missing z to 0 for plotting
        zs = [wp.z if wp.z is not None else 0 for wp in mission.waypoints]
    
    label = f"{label_prefix}{mission.drone_id}"

    # Main path plot (this is generally fine as ax.plot handles numpy color arrays)
    if is_3d_mission:
        ax.plot(xs, ys, zs, marker=marker, linestyle=linestyle, label=label, color=color, alpha=alpha)
    else:
        ax.plot(xs, ys, marker=marker, linestyle=linestyle, label=label, color=color, alpha=alpha)

    # Start and End markers
    if xs: # Check if there are waypoints to plot start/end for
        # Determine the color for scatter markers carefully
        scatter_marker_color = color
        # If color is None, or an empty string (but not a numpy array which is a valid color itself)
        if not isinstance(color, np.ndarray) and not color: 
            scatter_marker_color = 'black' # Default marker color

        start_label = f"{label} Start" if label else "Start"
        end_label = f"{label} End" if label else "End"
        
        # To avoid duplicate labels if called multiple times for same path type (e.g. conflict points)
        # This part needs to be handled by the main legend consolidation in visualize_missions_static
        # So we can pass label directly here.

        if is_3d_mission:
            ax.scatter(xs[0], ys[0], zs[0], color=scatter_marker_color, marker='o', s=30, label=start_label, alpha=alpha, zorder=5)
            ax.scatter(xs[-1], ys[-1], zs[-1], color=scatter_marker_color, marker='x', s=30, label=end_label, alpha=alpha, zorder=5)
        else: # 2D plot
            ax.scatter(xs[0], ys[0], color=scatter_marker_color, marker='o', s=30, label=start_label, alpha=alpha, zorder=5)
            ax.scatter(xs[-1], ys[-1], color=scatter_marker_color, marker='x', s=30, label=end_label, alpha=alpha, zorder=5)


def visualize_missions_static(
    primary_mission: DroneMission,
    other_schedules: List[DroneMission],
    conflicts: Optional[List[ConflictInfo]] = None,
    title: str = "Drone Missions Overview",
    filename_suffix: str = "static"
):
    """Generates a static plot of all drone trajectories and highlights conflicts."""
    
    # Determine if the overall plot should be 3D
    # If primary is 3D, or any other mission is 3D, make it a 3D plot.
    overall_is_3d = primary_mission.is_mission_3d() or \
                    any(other_m.is_mission_3d() for other_m in other_schedules)

    fig = plt.figure(figsize=(12, 10))
    
    if overall_is_3d:
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("X coordinate (m)")
        ax.set_ylabel("Y coordinate (m)")
        ax.set_zlabel("Z coordinate (Altitude m)")
    else:
        ax = fig.add_subplot(111)
        ax.set_xlabel("X coordinate (m)")
        ax.set_ylabel("Y coordinate (m)")
        ax.set_aspect('equal', adjustable='box')

    ax.set_title(title)

    # Plot primary mission
    plot_single_drone_path_static(ax, primary_mission, color='red', label_prefix="Primary: ", linestyle='-', marker='o', alpha=1.0)

    # Plot other drone missions
    # Generate distinct colors for other drones
    num_others = len(other_schedules)
    colors = plt.cm.viridis(np.linspace(0, 1, num_others)) if num_others > 0 else []


    for i, other_mission in enumerate(other_schedules):
        plot_single_drone_path_static(ax, other_mission, color=colors[i] if num_others > 0 else 'blue', label_prefix=f"Other_{i+1}: ", linestyle='--', marker='.')

    # Highlight conflicts
    conflict_labels_added = set()
    if conflicts:
        for i, conflict in enumerate(conflicts):
            # Use primary drone's conflicting position for the marker
            c_pos = conflict['primary_pos']
            cx, cy, cz = c_pos[0], c_pos[1], c_pos[2]
            
            label = "Conflict Point" if "Conflict Point" not in conflict_labels_added else None
            if label: conflict_labels_added.add("Conflict Point")

            if overall_is_3d:
                # If cz is None (e.g., a 2D conflict in a potentially 3D plot), plot at z=0 or a default plane
                plot_cz = cz if cz is not None else 0
                ax.scatter(cx, cy, plot_cz, color='magenta', s=80, marker='*', zorder=10, label=label, edgecolors='black')
            else:
                ax.scatter(cx, cy, color='magenta', s=80, marker='*', zorder=10, label=label, edgecolors='black')
    
    # Consolidate legend
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles)) # Remove duplicate labels
    ax.legend(by_label.values(), by_label.keys(), loc='best')
    
    ax.grid(True)
    filename = os.path.join(OUTPUT_DIR, f"{title.replace(' ', '_').lower()}_{filename_suffix}.png")
    plt.savefig(filename)
    print(f"Static plot saved to {filename}")
    plt.close(fig) # Close the figure to free memory


def animate_missions(
    primary_mission: DroneMission,
    other_schedules: List[DroneMission],
    conflicts: Optional[List[ConflictInfo]] = None,
    time_resolution_anim: float = 0.2, # Animation frame time step
    total_duration_override: Optional[float] = None, # Optional: to set a specific animation duration
    title: str = "Drone Mission Animation",
    filename_suffix: str = "animation"
):
    """Creates an animation of drone movements and saves it as GIF/MP4."""
    
    overall_is_3d = primary_mission.is_mission_3d() or \
                    any(other_m.is_mission_3d() for other_m in other_schedules)

    fig = plt.figure(figsize=(12, 10))
    if overall_is_3d:
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    else:
        ax = fig.add_subplot(111)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.set_aspect('equal', 'datalim')

    ax.set_title(title)
    
    all_missions = [primary_mission] + other_schedules
    
    # Determine plot bounds dynamically
    all_x = [wp.x for m in all_missions for wp in m.waypoints if m.waypoints]
    all_y = [wp.y for m in all_missions for wp in m.waypoints if m.waypoints]
    if not all_x or not all_y:
        print("No waypoints to animate.")
        plt.close(fig)
        return

    margin = 20 # Margin around the drone paths
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    if overall_is_3d:
        all_z = [wp.z for m in all_missions for wp in m.waypoints if m.waypoints and wp.z is not None]
        if all_z:
            ax.set_zlim(min(all_z) - margin, max(all_z) + margin)
        else: # If it's supposed to be 3D but no Z data, set a default Z range
            ax.set_zlim(-margin, margin)

    # Plot static paths for reference
    plot_single_drone_path_static(ax, primary_mission, color='red', label_prefix="P: ", linestyle=':', marker='', alpha=0.3)
    num_others = len(other_schedules)
    path_colors = plt.cm.viridis(np.linspace(0, 1, num_others)) if num_others > 0 else []
    for i, other_mission in enumerate(other_schedules):
        plot_single_drone_path_static(ax, other_mission, color=path_colors[i] if num_others > 0 else 'gray', label_prefix=f"O{i+1}: ", linestyle=':', marker='', alpha=0.3)

    # Drone representations (dynamic points)
    drone_points = []
    # Primary Drone
    if overall_is_3d:
        p_point, = ax.plot([], [], [], 'o', color='red', markersize=10, label="Primary Drone")
    else:
        p_point, = ax.plot([], [], 'o', color='red', markersize=10, label="Primary Drone")
    drone_points.append(p_point)

    # Other Drones
    for i in range(num_others):
        if overall_is_3d:
            o_point, = ax.plot([], [], [], 'o', color=path_colors[i], markersize=8, label=f"Other Drone {i+1}")
        else:
            o_point, = ax.plot([], [], 'o', color=path_colors[i], markersize=8, label=f"Other Drone {i+1}")
        drone_points.append(o_point)

    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.5))
    
    # Conflict marker (initially invisible)
    if overall_is_3d:
        conflict_marker, = ax.plot([], [], [], '*', color='magenta', markersize=25, alpha=0.0, markeredgecolor='black', label="Current Conflict")
    else:
        conflict_marker, = ax.plot([], [], '*', color='magenta', markersize=25, alpha=0.0, markeredgecolor='black', label="Current Conflict")


    # Determine time range for animation
    mission_start_times = [m.get_start_time() for m in all_missions if m.waypoints]
    mission_end_times = [m.get_end_time() for m in all_missions if m.waypoints]
    
    anim_start_time = min(mission_start_times) if mission_start_times else 0
    anim_end_time = max(mission_end_times) if mission_end_times else 10 # Default if no missions
    if total_duration_override:
        anim_end_time = anim_start_time + total_duration_override

    # Generate frames based on animation time resolution
    frames = np.arange(anim_start_time, anim_end_time + time_resolution_anim, time_resolution_anim)

    def update(frame_time):
        time_text.set_text(f'Time: {frame_time:.2f}s')
        
        # Update drone positions
        for i, mission in enumerate(all_missions):
            pos_wp = get_drone_position_at_time(mission, frame_time)
            point_artist = drone_points[i]
            if pos_wp:
                x, y, z_val = pos_wp.x, pos_wp.y, pos_wp.z
                if overall_is_3d:
                    point_artist.set_data_3d([x], [y], [z_val if z_val is not None else 0])
                else:
                    point_artist.set_data([x], [y])
                point_artist.set_visible(True)
            else:
                point_artist.set_visible(False)
        
        # Check for active conflict at this frame_time
        active_conflict_pos = None
        if conflicts:
            for conflict in conflicts:
                # A conflict is "active" if the frame_time is very close to the conflict time
                if abs(conflict['time'] - frame_time) < (time_resolution_anim / 2.0): # Within half a frame step
                    active_conflict_pos = conflict['primary_pos'] # Show on primary drone's conflict pos
                    break
        
        if active_conflict_pos:
            cx, cy, cz_conflict = active_conflict_pos
            if overall_is_3d:
                conflict_marker.set_data_3d([cx], [cy], [cz_conflict if cz_conflict is not None else 0])
            else:
                conflict_marker.set_data([cx], [cy])
            conflict_marker.set_alpha(1.0)
        else:
            conflict_marker.set_alpha(0.0)
            
        artists_to_return = drone_points + [time_text, conflict_marker]
        return artists_to_return

    ani = FuncAnimation(fig, update, frames=frames, blit=True, interval=max(1, int(time_resolution_anim * 1000)), repeat=False)
    
    # Consolidate legend
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='best')
    
    # Try saving as MP4, fallback to GIF
    mp4_filename = os.path.join(OUTPUT_DIR, f"{title.replace(' ', '_').lower()}_{filename_suffix}.mp4")
    gif_filename = os.path.join(OUTPUT_DIR, f"{title.replace(' ', '_').lower()}_{filename_suffix}.gif")

    try:
        ani.save(mp4_filename, writer='ffmpeg', fps=max(1, int(1/time_resolution_anim)))
        print(f"Animation saved to {mp4_filename}")
    except Exception as e_mp4:
        print(f"Could not save animation as MP4 (ffmpeg might be missing or error: {e_mp4}).")
        print("Trying to save as GIF instead...")
        try:
            ani.save(gif_filename, writer=PillowWriter(fps=max(1, int(1/time_resolution_anim))))
            print(f"Animation saved to {gif_filename}")
        except Exception as e_gif:
            print(f"Could not save animation as GIF: {e_gif}")
            print("Consider installing ffmpeg for MP4 or Pillow for GIF.")
            # plt.show() # Optionally show if saving fails
    plt.close(fig) # Close the figure