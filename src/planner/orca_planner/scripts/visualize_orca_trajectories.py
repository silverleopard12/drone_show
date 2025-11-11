#!/usr/bin/env python3
"""
Visualize ORCA-generated trajectories
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import sys


def load_trajectories(filename):
    """Load trajectories from ORCA output file"""
    trajectories = []
    current_traj = None

    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()

                # Skip comments and empty lines
                if not line or line.startswith('#'):
                    continue

                if line.startswith('DRONE'):
                    # Save previous trajectory
                    if current_traj is not None and len(current_traj['points']) > 0:
                        trajectories.append(current_traj)

                    # Start new trajectory
                    drone_id = int(line.split()[1])
                    current_traj = {
                        'id': drone_id,
                        'points': []
                    }

                elif line.startswith('POINTS'):
                    continue

                else:
                    # Parse trajectory point
                    parts = line.split()
                    if len(parts) >= 7:
                        t, x, y, z, vx, vy, vz = map(float, parts[:7])
                        yaw = float(parts[7]) if len(parts) > 7 else 0.0

                        current_traj['points'].append({
                            't': t, 'x': x, 'y': y, 'z': z,
                            'vx': vx, 'vy': vy, 'vz': vz,
                            'yaw': yaw
                        })

            # Save last trajectory
            if current_traj is not None and len(current_traj['points']) > 0:
                trajectories.append(current_traj)

    except FileNotFoundError:
        print(f"Error: File not found: {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading file: {e}")
        sys.exit(1)

    return trajectories


def plot_trajectories_3d(trajectories):
    """Plot all trajectories in 3D"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    colors = plt.cm.rainbow(np.linspace(0, 1, len(trajectories)))

    for traj, color in zip(trajectories, colors):
        points = traj['points']
        if not points:
            continue

        xs = [p['x'] for p in points]
        ys = [p['y'] for p in points]
        zs = [p['z'] for p in points]

        # Plot trajectory
        ax.plot(xs, ys, zs, color=color, linewidth=2,
                alpha=0.7, label=f"Drone {traj['id']}")

        # Mark start (circle) and end (square)
        ax.scatter(xs[0], ys[0], zs[0], color=color, marker='o',
                   s=150, edgecolors='black', linewidths=2)
        ax.scatter(xs[-1], ys[-1], zs[-1], color=color, marker='s',
                   s=150, edgecolors='black', linewidths=2)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('ORCA Swarm Trajectories', fontsize=14, fontweight='bold')

    # Set equal aspect ratio
    max_range = 0
    for dim in ['x', 'y', 'z']:
        all_vals = []
        for traj in trajectories:
            all_vals.extend([p[dim] for p in traj['points']])
        if all_vals:
            r = (max(all_vals) - min(all_vals)) / 2
            max_range = max(max_range, r)

    if trajectories and trajectories[0]['points']:
        mid_x = np.mean([p['x'] for traj in trajectories for p in traj['points']])
        mid_y = np.mean([p['y'] for traj in trajectories for p in traj['points']])
        mid_z = np.mean([p['z'] for traj in trajectories for p in traj['points']])

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Legend (only first 10 drones to avoid clutter)
    if len(trajectories) <= 10:
        ax.legend(loc='upper left', fontsize=8)

    plt.tight_layout()
    plt.show()


def plot_distance_over_time(trajectories):
    """Plot minimum distance between drones over time"""
    if len(trajectories) < 2:
        print("Need at least 2 drones to plot distances")
        return

    # Find common time range
    max_time = min([traj['points'][-1]['t'] for traj in trajectories])
    min_time = max([traj['points'][0]['t'] for traj in trajectories])

    if max_time <= min_time:
        print("No overlapping time range")
        return

    # Sample time points
    dt = 0.1
    times = np.arange(min_time, max_time, dt)
    min_distances = []
    mean_distances = []

    for t in times:
        positions = []
        for traj in trajectories:
            # Interpolate position at time t
            points = traj['points']
            for i in range(len(points) - 1):
                if points[i]['t'] <= t <= points[i+1]['t']:
                    alpha = (t - points[i]['t']) / (points[i+1]['t'] - points[i]['t'])
                    x = (1 - alpha) * points[i]['x'] + alpha * points[i+1]['x']
                    y = (1 - alpha) * points[i]['y'] + alpha * points[i+1]['y']
                    z = (1 - alpha) * points[i]['z'] + alpha * points[i+1]['z']
                    positions.append(np.array([x, y, z]))
                    break

        # Compute pairwise distances
        if len(positions) >= 2:
            distances = []
            for i in range(len(positions)):
                for j in range(i + 1, len(positions)):
                    dist = np.linalg.norm(positions[i] - positions[j])
                    distances.append(dist)

            min_distances.append(min(distances))
            mean_distances.append(np.mean(distances))
        else:
            min_distances.append(0)
            mean_distances.append(0)

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Minimum distance
    ax1.plot(times, min_distances, linewidth=2, color='red', label='Min distance')
    ax1.axhline(y=1.0, color='orange', linestyle='--', linewidth=2,
                label='Typical safety distance')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Minimum Distance (m)', fontsize=12)
    ax1.set_title('Minimum Distance Between Any Two Drones',
                  fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)

    # Mean distance
    ax2.plot(times, mean_distances, linewidth=2, color='blue', label='Mean distance')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Mean Distance (m)', fontsize=12)
    ax2.set_title('Mean Pairwise Distance', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    plt.tight_layout()
    plt.show()


def plot_velocity_profile(trajectories):
    """Plot velocity profiles over time"""
    fig, ax = plt.subplots(figsize=(12, 6))

    colors = plt.cm.rainbow(np.linspace(0, 1, len(trajectories)))

    for traj, color in zip(trajectories, colors):
        points = traj['points']
        times = [p['t'] for p in points]
        speeds = [np.sqrt(p['vx']**2 + p['vy']**2 + p['vz']**2) for p in points]

        ax.plot(times, speeds, color=color, linewidth=1.5,
                alpha=0.7, label=f"Drone {traj['id']}")

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title('Velocity Profiles', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Only show legend if not too many drones
    if len(trajectories) <= 10:
        ax.legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    plt.show()


def print_statistics(trajectories):
    """Print trajectory statistics"""
    print("\n" + "="*60)
    print("ORCA Trajectory Statistics")
    print("="*60)

    print(f"Number of drones: {len(trajectories)}")

    if not trajectories or not trajectories[0]['points']:
        print("No trajectory data")
        return

    # Duration
    max_duration = max([traj['points'][-1]['t'] for traj in trajectories])
    print(f"Mission duration: {max_duration:.2f} seconds")

    # Average trajectory length
    avg_length = np.mean([len(traj['points']) for traj in trajectories])
    print(f"Average trajectory points: {avg_length:.0f}")

    # Speed statistics
    all_speeds = []
    for traj in trajectories:
        for p in traj['points']:
            speed = np.sqrt(p['vx']**2 + p['vy']**2 + p['vz']**2)
            all_speeds.append(speed)

    print(f"\nSpeed statistics:")
    print(f"  Mean: {np.mean(all_speeds):.3f} m/s")
    print(f"  Max:  {np.max(all_speeds):.3f} m/s")
    print(f"  Min:  {np.min(all_speeds):.3f} m/s")

    # Distance traveled
    for traj in trajectories[:5]:  # First 5 drones
        dist = 0.0
        points = traj['points']
        for i in range(len(points) - 1):
            p1 = np.array([points[i]['x'], points[i]['y'], points[i]['z']])
            p2 = np.array([points[i+1]['x'], points[i+1]['y'], points[i+1]['z']])
            dist += np.linalg.norm(p2 - p1)
        print(f"Drone {traj['id']} distance: {dist:.2f} m")

    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Visualize ORCA-generated trajectories')
    parser.add_argument('filename', type=str,
                       help='Trajectory file to visualize')
    parser.add_argument('--plot-type', type=str, default='all',
                       choices=['3d', 'distance', 'velocity', 'all'],
                       help='Type of plot to show')
    parser.add_argument('--no-stats', action='store_true',
                       help='Do not print statistics')

    args = parser.parse_args()

    print(f"Loading trajectories from: {args.filename}")
    trajectories = load_trajectories(args.filename)

    if not trajectories:
        print("No trajectories found in file")
        return

    print(f"Loaded {len(trajectories)} trajectories")

    # Print statistics
    if not args.no_stats:
        print_statistics(trajectories)

    # Show plots
    if args.plot_type in ['3d', 'all']:
        plot_trajectories_3d(trajectories)

    if args.plot_type in ['distance', 'all']:
        plot_distance_over_time(trajectories)

    if args.plot_type in ['velocity', 'all']:
        plot_velocity_profile(trajectories)


if __name__ == '__main__':
    main()
