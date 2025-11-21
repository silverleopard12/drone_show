#!/usr/bin/env python3

"""
CAT-ORA Assignment Calculator
Computes optimal robot-to-goal assignments using the CAT-ORA algorithm
and outputs them in a format compatible with scenario launch files.
"""

import rclpy
from rclpy.node import Node
from catora_formation.srv import GetAssignment
from geometry_msgs.msg import Point
import sys


class CatoraAssignmentCalculator(Node):
    def __init__(self):
        super().__init__('catora_assignment_calculator')
        self.client = self.create_client(GetAssignment, '/catora_formation/get_assignment')

        self.get_logger().info('Waiting for CAT-ORA service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('CAT-ORA service available!')

    def compute_assignment(self, initial_configs, goal_configs):
        """
        Compute optimal assignment using CAT-ORA algorithm

        Args:
            initial_configs: List of (x, y, z) tuples for initial positions
            goal_configs: List of (x, y, z) tuples for goal positions

        Returns:
            List of integers representing the assignment mapping
        """
        request = GetAssignment.Request()

        # Convert to Point messages
        request.initial_configurations = [
            Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            for pos in initial_configs
        ]
        request.goal_configurations = [
            Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            for pos in goal_configs
        ]

        self.get_logger().info(f'Computing assignment for {len(initial_configs)} robots...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.success:
            self.get_logger().info(f'Assignment computed successfully!')
            self.get_logger().info(f'Mapping: {response.mapping}')
            return response.mapping
        else:
            self.get_logger().error(f'Failed to compute assignment: {response.message}')
            return None


def print_drone_configs(initial_positions, goal_positions, assignment):
    """Print drone configurations in the format used by launch files"""
    print("\n" + "="*80)
    print("DRONE CONFIGURATIONS (Copy to launch file)")
    print("="*80)
    print("drone_configs = [")

    for i, (init_pos, goal_idx) in enumerate(zip(initial_positions, assignment)):
        goal_pos = goal_positions[goal_idx]

        # Calculate distance
        dx = goal_pos[0] - init_pos[0]
        dy = goal_pos[1] - init_pos[1]
        dz = goal_pos[2] - init_pos[2]
        distance = (dx**2 + dy**2 + dz**2) ** 0.5

        print(f"    {{'drone_id': {i}, "
              f"'init_x': {init_pos[0]}, 'init_y': {init_pos[1]}, 'init_z': {init_pos[2]}, "
              f"'target_x': {goal_pos[0]}, 'target_y': {goal_pos[1]}, 'target_z': {goal_pos[2]}}},  "
              f"# Distance: {distance:.2f}m")

    print("]")
    print("="*80 + "\n")


def example_25_drones():
    """Example for 25 drones: Grid to Triangle formation"""

    # Initial positions - 5x5 Grid
    initial_positions = []
    for i in range(5):  # 5 rows (Y direction)
        for j in range(5):  # 5 columns (Z direction)
            initial_positions.append((
                3.0,
                8.0 + i * 4.0,  # Y: 8, 12, 16, 20, 24
                10.0 + j * 4.0  # Z: 10, 14, 18, 22, 26
            ))

    # Target positions - Triangle formation
    goal_positions = [
        (3.0, 20.7, 16.5), (3.0, 27.1, 13.3), (3.0, 30.3, 16.5),
        (3.0, 27.1, 10.1), (3.0, 36.7, 19.7), (3.0, 23.9, 13.3),
        (3.0, 23.9, 16.5), (3.0, 30.3, 13.3), (3.0, 33.5, 22.9),
        (3.0, 36.7, 22.9), (3.0, 17.5, 19.7), (3.0, 20.7, 19.7),
        (3.0, 23.9, 19.7), (3.0, 33.5, 16.5), (3.0, 33.5, 19.7),
        (3.0, 14.3, 22.9), (3.0, 17.5, 22.9), (3.0, 20.7, 22.9),
        (3.0, 30.3, 19.7), (3.0, 30.3, 22.9), (3.0, 23.9, 22.9),
        (3.0, 27.1, 22.9), (3.0, 27.1, 16.5), (3.0, 27.1, 19.7),
        (3.0, 39.9, 22.9)
    ]

    return initial_positions, goal_positions


def main(args=None):
    rclpy.init(args=args)

    calculator = CatoraAssignmentCalculator()

    # Get example configuration
    initial_positions, goal_positions = example_25_drones()

    # Compute assignment
    assignment = calculator.compute_assignment(initial_positions, goal_positions)

    if assignment:
        # Print configurations
        print_drone_configs(initial_positions, goal_positions, assignment)

        # Calculate statistics
        distances = []
        for i, goal_idx in enumerate(assignment):
            init_pos = initial_positions[i]
            goal_pos = goal_positions[goal_idx]
            dx = goal_pos[0] - init_pos[0]
            dy = goal_pos[1] - init_pos[1]
            dz = goal_pos[2] - init_pos[2]
            distance = (dx**2 + dy**2 + dz**2) ** 0.5
            distances.append(distance)

        print(f"\nAssignment Statistics:")
        print(f"  Max distance: {max(distances):.2f}m")
        print(f"  Avg distance: {sum(distances)/len(distances):.2f}m")
        print(f"  Min distance: {min(distances):.2f}m")

    calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
