#!/usr/bin/env python3
"""
Script to visualize saved trajectories
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse


def load_trajectories(filename):
    """Load trajectories from file"""
    trajectories = []
    current_drone = None
    current_traj = {'id': -1, 'points': []}

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            if line.startswith('DRONE'):
                # Save previous trajectory
                if current_traj['id'] >= 0:
                    trajectories.append(current_traj)

                # Start new trajectory
                drone_id = int(line.split()[1])
                current_traj = {'id': drone_id, 'points': []}

            elif line.startswith('POINTS'):
                continue

            else:
                # Parse point data
                parts = line.split()
                if len(parts) >= 7:
                    t, x, y, z, vx, vy, vz = map(float, parts[:7])
                    current_traj['points'].append({
                        't': t, 'x': x, 'y': y, 'z': z,
                        'vx': vx, 'vy': vy, 'vz': vz
                    })

    # Save last trajectory
    if current_traj['id'] >= 0:
        trajectories.append(current_traj)

    return trajectories


def plot_trajectories_3d(trajectories):
    """Plot all trajectories in 3D"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = plt.cm.rainbow(np.linspace(0, 1, len(trajectories)))

    for traj, color in zip(trajectories, colors):
        points = traj['points']
        if not points:
            continue

        xs = [p['x'] for p in points]
        ys = [p['y'] for p in points]
        zs = [p['z'] for p in points]

        ax.plot(xs, ys, zs, color=color, linewidth=2, label=f"Drone {traj['id']}")

        # Mark start and end
        ax.scatter(xs[0], ys[0], zs[0], color=color, marker='o', s=100)
        ax.scatter(xs[-1], ys[-1], zs[-1], color=color, marker='s', s=100)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Drone Trajectories')
    ax.legend()

    plt.show()


def plot_distance_over_time(trajectories):
    """Plot minimum distance between drones over time"""
    if len(trajectories) < 2:
        print("Need at least 2 drones to plot distances")
        return

    # Find common time range
    max_time = min([traj['points'][-1]['t'] for traj in trajectories])

    # Sample time points
    dt = 0.1
    times = np.arange(0, max_time, dt)
    min_distances = []

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
                    positions.append([x, y, z])
                    break

        # Compute minimum distance
        if len(positions) >= 2:
            min_dist = float('inf')
            for i in range(len(positions)):
                for j in range(i + 1, len(positions)):
                    dist = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
                    min_dist = min(min_dist, dist)
            min_distances.append(min_dist)
        else:
            min_distances.append(0)

    # Plot
    plt.figure(figsize=(10, 6))
    plt.plot(times, min_distances, linewidth=2)
    plt.axhline(y=1.0, color='r', linestyle='--', label='Safety distance')
    plt.xlabel('Time (s)')
    plt.ylabel('Minimum Distance (m)')
    plt.title('Minimum Distance Between Drones Over Time')
    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize drone trajectories')
    parser.add_argument('filename', type=str, help='Trajectory file to visualize')
    parser.add_argument('--plot-type', type=str, default='3d',
                       choices=['3d', 'distance', 'both'],
                       help='Type of plot to show')

    args = parser.parse_args()

    print(f"Loading trajectories from: {args.filename}")
    trajectories = load_trajectories(args.filename)

    if not trajectories:
        print("No trajectories found in file")
        return

    print(f"Loaded {len(trajectories)} trajectories")

    if args.plot_type in ['3d', 'both']:
        plot_trajectories_3d(trajectories)

    if args.plot_type in ['distance', 'both']:
        plot_distance_over_time(trajectories)


if __name__ == '__main__':
    main()
