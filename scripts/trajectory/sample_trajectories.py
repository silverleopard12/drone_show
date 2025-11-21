#!/usr/bin/env python3
"""
Enhanced trajectory sampling script for ego-swarm
Samples completed trajectories from each drone and exports to DroneShow format

Usage:
    python3 sample_trajectories.py --num_drones 36 --sampling_rate 50
"""

import rclpy
from rclpy.node import Node
from traj_utils.msg import Bspline
from geometry_msgs.msg import Point
import numpy as np
from pathlib import Path
from datetime import datetime
import argparse


class BSplineEvaluator:
    """Evaluates B-spline trajectories using de Boor's algorithm"""

    def __init__(self, control_points, knots, order):
        """
        Args:
            control_points: List of [x, y, z] positions (N x 3)
            knots: Knot vector (length = N + order + 1)
            order: B-spline order (typically 3 for cubic)
        """
        self.control_points = np.array(control_points)  # Shape: (N, 3)
        self.knots = np.array(knots)
        self.p = order  # degree
        self.n = len(control_points) - 1  # number of control points - 1
        self.m = len(knots) - 1  # number of knots - 1

    def evaluate_de_boor(self, u):
        """
        Evaluate B-spline at parameter u using de Boor's algorithm

        This is the proper implementation matching ego-swarm's C++ code

        Args:
            u: Parameter value in [knots[p], knots[m-p]]

        Returns:
            np.array: [x, y, z] position at parameter u
        """
        # Clamp u to valid range
        u_min = self.knots[self.p]
        u_max = self.knots[self.m - self.p]
        ub = np.clip(u, u_min, u_max)

        # Find knot span: find k such that u is in [u_k, u_{k+1})
        k = self.p
        while k < self.m - self.p:
            if self.knots[k + 1] >= ub:
                break
            k += 1

        # de Boor's algorithm
        # Initialize with control points
        d = []
        for i in range(self.p + 1):
            idx = k - self.p + i
            if idx >= 0 and idx < len(self.control_points):
                d.append(self.control_points[idx].copy())
            else:
                d.append(np.zeros(3))

        # Recursive computation
        for r in range(1, self.p + 1):
            for i in range(self.p, r - 1, -1):
                idx_alpha = i + k - self.p

                # Compute alpha
                denominator = self.knots[idx_alpha + 1 + self.p - r] - self.knots[idx_alpha]
                if abs(denominator) < 1e-10:
                    alpha = 0.0
                else:
                    alpha = (ub - self.knots[idx_alpha]) / denominator

                # Update d[i]
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]

        return d[self.p]

    def evaluate(self, t):
        """
        Evaluate B-spline at time t (in seconds from start)

        Args:
            t: Time in seconds (0 to duration)

        Returns:
            np.array: [x, y, z] position at time t
        """
        # Convert time t to parameter u
        # u = t + knots[p] (this matches evaluateDeBoorT in C++ code)
        u = t + self.knots[self.p]
        return self.evaluate_de_boor(u)

    def get_duration(self):
        """Get total duration of the trajectory"""
        if len(self.knots) <= self.p:
            return 0.0
        # Duration = knots[m-p] - knots[p]
        return self.knots[self.m - self.p] - self.knots[self.p]


class TrajectoryData:
    """Stores trajectory data for a single drone"""

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.samples = []  # List of (timestamp, x, y, z)
        self.trajectory_count = 0

    def add_samples(self, samples):
        """Add sampled trajectory points"""
        self.samples.extend(samples)
        self.trajectory_count += 1

    def get_duration(self):
        """Get total trajectory duration"""
        if not self.samples:
            return 0.0
        return self.samples[-1][0] - self.samples[0][0]

    def get_length(self):
        """Calculate total path length"""
        if len(self.samples) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(self.samples)):
            p1 = np.array(self.samples[i-1][1:])
            p2 = np.array(self.samples[i][1:])
            length += np.linalg.norm(p2 - p1)
        return length


class TrajectorySampler(Node):
    """ROS2 node that samples trajectories from all drones"""

    def __init__(self, num_drones, sampling_rate):
        super().__init__('trajectory_sampler')

        self.num_drones = num_drones
        self.sampling_dt = 1.0 / sampling_rate  # Convert Hz to seconds

        # Store trajectory data for each drone
        self.drone_data = {i: TrajectoryData(i) for i in range(num_drones)}

        # Global start time (set when first message arrives)
        self.global_start_time = None

        # Create subscribers for each drone
        self._subscriptions = []
        for drone_id in range(num_drones):
            topic = f'/drone_{drone_id}_planning/bspline'
            sub = self.create_subscription(
                Bspline,
                topic,
                lambda msg, d=drone_id: self.bspline_callback(msg, d),
                10
            )
            self._subscriptions.append(sub)

        self.get_logger().info(f'Trajectory Sampler initialized')
        self.get_logger().info(f'  Drones: {num_drones}')
        self.get_logger().info(f'  Sampling rate: {sampling_rate} Hz (dt={self.sampling_dt:.3f}s)')

    def bspline_callback(self, msg, drone_id):
        """Process incoming B-spline trajectory"""

        # Set global start time from first message
        if self.global_start_time is None:
            self.global_start_time = self.get_clock().now()
            self.get_logger().info(f'Global start time set')

        # Extract control points
        control_points = [[pt.x, pt.y, pt.z] for pt in msg.pos_pts]

        if not control_points or not msg.knots:
            self.get_logger().warn(f'Drone {drone_id}: Empty trajectory received')
            return

        # Create B-spline evaluator
        bspline = BSplineEvaluator(control_points, msg.knots, msg.order)
        duration = bspline.get_duration()

        if duration <= 0:
            self.get_logger().warn(f'Drone {drone_id}: Invalid duration {duration:.3f}')
            return

        # Calculate message time offset from global start
        msg_time = rclpy.time.Time.from_msg(msg.start_time)
        msg_offset = (msg_time.nanoseconds - self.global_start_time.nanoseconds) / 1e9

        # Sample the trajectory
        samples = []
        t = 0.0
        while t <= duration:
            pos = bspline.evaluate(t)
            timestamp = msg_offset + t
            samples.append((timestamp, pos[0], pos[1], pos[2]))
            t += self.sampling_dt

        # Always include final point
        if t - self.sampling_dt < duration:
            pos = bspline.evaluate(duration)
            samples.append((msg_offset + duration, pos[0], pos[1], pos[2]))

        # Store samples
        self.drone_data[drone_id].add_samples(samples)

        self.get_logger().info(
            f'Drone {drone_id}: Sampled trajectory #{self.drone_data[drone_id].trajectory_count} '
            f'({len(samples)} points, duration={duration:.2f}s)'
        )


def export_droneshow_format(drone_data, output_dir):
    """Export in DroneShow trajectory format (line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b)"""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Color palette for different drones
    colors = [
        (255, 0, 0),     # Red
        (0, 255, 0),     # Green
        (0, 0, 255),     # Blue
        (255, 255, 0),   # Yellow
        (255, 0, 255),   # Magenta
        (0, 255, 255),   # Cyan
        (255, 128, 0),   # Orange
        (128, 0, 255),   # Purple
    ]

    for drone_id, data in drone_data.items():
        if not data.samples:
            continue

        # Use drone_id + 1 for filename (node_1.txt instead of node_0.txt)
        filename = output_dir / f'node_{drone_id + 1}.txt'
        color = colors[drone_id % len(colors)]

        with open(filename, 'w') as f:
            for line_num, (t, x, y, z) in enumerate(data.samples, 1):
                # Format: line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b
                f.write(f'{line_num},{drone_id},{t:.2f},move,{x:.2f},{y:.2f},{z:.1f},0.0,{color[0]},{color[1]},{color[2]}\n')

        print(f'  Drone {drone_id}: {len(data.samples)} points -> {filename}')


def print_statistics(drone_data):
    """Print trajectory statistics"""

    print('\nTrajectory Statistics:')
    print('=' * 80)

    total_samples = 0
    total_duration = 0.0
    total_length = 0.0

    for drone_id in sorted(drone_data.keys()):
        data = drone_data[drone_id]

        if not data.samples:
            print(f'  Drone {drone_id:2d}: No trajectory data')
            continue

        duration = data.get_duration()
        length = data.get_length()

        total_samples += len(data.samples)
        total_duration = max(total_duration, duration)
        total_length += length

        print(f'  Drone {drone_id:2d}: {len(data.samples):4d} samples, '
              f'{duration:6.2f}s duration, {length:7.2f}m length, '
              f'{data.trajectory_count} trajectory segments')

    print('=' * 80)
    print(f'  Total: {total_samples} samples, {total_duration:.2f}s max duration, {total_length:.2f}m total length')
    print()


def main():
    parser = argparse.ArgumentParser(description='Sample trajectories from ego-swarm drones')
    parser.add_argument('--num_drones', type=int, default=36, help='Number of drones')
    parser.add_argument('--sampling_rate', type=float, default=50.0, help='Sampling rate in Hz')
    parser.add_argument('--output_dir', type=str, default=None,
                       help='Output directory (default: trajectories_YYYYMMDD_HHMMSS)')
    parser.add_argument('--duration', type=float, default=60.0,
                       help='Recording duration in seconds')

    args = parser.parse_args()

    # Create output directory with timestamp
    if args.output_dir is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output_dir = f'trajectories_{timestamp}'

    print('=' * 80)
    print('Trajectory Sampler for ego-swarm')
    print('=' * 80)
    print(f'Configuration:')
    print(f'  Number of drones: {args.num_drones}')
    print(f'  Sampling rate: {args.sampling_rate} Hz')
    print(f'  Recording duration: {args.duration}s')
    print(f'  Output directory: {args.output_dir}')
    print(f'  Output format: DroneShow')
    print('=' * 80)
    print()

    # Initialize ROS2
    rclpy.init()

    # Create sampler node
    sampler = TrajectorySampler(args.num_drones, args.sampling_rate)

    # Run for specified duration
    print(f'Recording trajectories...')
    print(f'Press Ctrl+C to stop early\n')

    try:
        import time
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < args.duration:
            rclpy.spin_once(sampler, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\nRecording interrupted by user')

    # Shutdown ROS2
    try:
        sampler.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

    # Print statistics
    print_statistics(sampler.drone_data)

    # Export data
    print(f'Exporting trajectories to: {args.output_dir}')
    print('-' * 80)

    export_droneshow_format(sampler.drone_data, args.output_dir)

    print('-' * 80)
    print(f'✓ Export complete!')
    print()


if __name__ == '__main__':
    main()
