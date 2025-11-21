#!/usr/bin/env python3
"""
CAT-ORA Allocator
Collision-Aware Time-Optimal formation Reshaping Algorithm

Two modes:
1. ROS2 Service Mode: Interfaces with ROS2 CAT-ORA planner service (requires service running)
2. Standalone Mode: Pure Python implementation (no ROS2 required) - DEFAULT
"""

import numpy as np
import sys
import os

# Try to import ROS2 dependencies (optional)
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    ROS2_AVAILABLE = True
except ImportError:
    pass

# Import standalone allocator
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from CATORA_Allocator_Standalone import CATORAStandaloneAllocator


class CATORAAllocator:
    """
    CAT-ORA 기반 할당 알고리즘

    Two modes:
    - ROS2 Service Mode: ROS2 서비스를 통해 CAT-ORA 플래너와 통신
    - Standalone Mode: Python standalone 구현 (기본값)
    """

    def __init__(self, use_ros2_service=False):
        """
        Args:
            use_ros2_service: True면 ROS2 서비스 사용, False면 standalone 사용
        """
        self.use_ros2_service = use_ros2_service

        if use_ros2_service:
            if not ROS2_AVAILABLE:
                raise RuntimeError("ROS2 not available. Install ROS2 or use standalone mode (use_ros2_service=False)")

            self._init_ros2_service()
        else:
            # Standalone mode
            self.standalone_allocator = CATORAStandaloneAllocator(
                max_velocity=2.0,
                max_acceleration=2.0,
                min_distance=1.0
            )
            print("CAT-ORA Standalone mode initialized (no ROS2 required)")

    def _init_ros2_service(self):
        """Initialize ROS2 service client (only if use_ros2_service=True)"""
        if not hasattr(self, '_node'):
            from rclpy.node import Node

            class CATORANode(Node):
                def __init__(self):
                    super().__init__('catora_allocator')

            self._node = CATORANode()

        # Import service definition
        try:
            from catora_planner.srv import GetAssignment
            self.GetAssignment = GetAssignment
        except ImportError:
            print("ERROR: Failed to import catora_planner.srv.GetAssignment")
            print("Make sure catora_planner is built and sourced:")
            print("  cd ~/ego_swarm/ego-planner-swarm")
            print("  colcon build --packages-select catora_planner")
            print("  source install/setup.bash")
            raise

        # Create service client
        self.client = self._node.create_client(GetAssignment, '/catora_planner/get_assignment')

        print('Waiting for CAT-ORA service...')
        timeout_sec = 10.0
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            print(f'ERROR: CAT-ORA service not available after {timeout_sec}s')
            print('Make sure catora_planner_node is running:')
            print('  ros2 launch catora_planner catora_planner.launch.py')
            raise RuntimeError('CAT-ORA service not available')

        print('CAT-ORA service available!')

    def calculate_distance(self, pos1, pos2):
        """두 점 사이의 유클리드 거리 계산"""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def allocate(self, drones, goals):
        """
        CAT-ORA를 사용하여 드론을 목표에 할당

        Args:
            drones: 드론 위치 리스트 [[x,y,z], ...]
            goals: 목표 위치 리스트 [[x,y,z], ...]

        Returns:
            assignment: 드론 인덱스 → 목표 인덱스 매핑 리스트
                       assignment[i] = j 는 드론 i가 목표 j로 할당됨을 의미
        """
        if self.use_ros2_service:
            return self._allocate_ros2_service(drones, goals)
        else:
            return self._allocate_standalone(drones, goals)

    def _allocate_standalone(self, drones, goals):
        """Standalone Python 구현으로 할당"""
        print(f'Computing CAT-ORA assignment for {len(drones)} drones... (Standalone mode)')

        # Call standalone allocator
        assignment = self.standalone_allocator.allocate(drones, goals, verbose=True)

        return assignment

    def _allocate_ros2_service(self, drones, goals):
        """ROS2 서비스를 통해 할당"""
        print(f'Computing CAT-ORA assignment for {len(drones)} drones... (ROS2 service mode)')

        # Create service request
        request = self.GetAssignment.Request()

        # Convert to Point messages
        request.initial_configurations = [
            Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            for pos in drones
        ]
        request.goal_configurations = [
            Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            for pos in goals
        ]

        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)

        response = future.result()

        if not response.success:
            print(f'ERROR: CAT-ORA assignment failed: {response.message}')
            raise RuntimeError(f'CAT-ORA assignment failed: {response.message}')

        assignment = list(response.mapping)

        print('CAT-ORA assignment computed successfully!')

        # Print allocation result
        self._print_allocation_result(drones, goals, assignment)

        return assignment

    def _print_allocation_result(self, drones, goals, assignment):
        """할당 결과 출력"""
        print("\n=== CAT-ORA Allocation Result ===")
        distances = []

        for drone_id, goal_id in enumerate(assignment):
            distance = self.calculate_distance(drones[drone_id], goals[goal_id])
            distances.append(distance)
            print(f"Drone {drone_id} → Goal {goal_id}: {distance:.2f}m")

        print(f"\nTotal distance: {sum(distances):.2f}m")
        print(f"Average distance: {np.mean(distances):.2f}m")
        print(f"Max distance (bottleneck): {max(distances):.2f}m")
        print(f"Min distance: {min(distances):.2f}m")
        print(f"Std deviation: {np.std(distances):.2f}m")
        print("=" * 35)


def create_allocator(use_ros2_service=False):
    """
    Create a CAT-ORA allocator instance

    Args:
        use_ros2_service: True면 ROS2 서비스 사용, False면 standalone 사용 (기본값)

    Returns:
        CATORAAllocator instance
    """
    if use_ros2_service:
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available. Use standalone mode (use_ros2_service=False)")

        if not rclpy.ok():
            rclpy.init()

    allocator = CATORAAllocator(use_ros2_service=use_ros2_service)
    return allocator


def test_allocator():
    """Test the CATORA allocator with sample data (standalone mode)"""
    print("=== CAT-ORA Allocator Test (Standalone Mode) ===\n")

    allocator = CATORAAllocator(use_ros2_service=False)

    # Test data - 3 drones
    drones = [
        [0.0, 0.0, 3.0],
        [0.0, 1.0, 3.0],
        [0.0, 2.0, 3.0],
    ]

    goals = [
        [10.0, 2.0, 3.0],
        [10.0, 1.0, 3.0],
        [10.0, 0.0, 3.0],
    ]

    print(f"Drones: {drones}")
    print(f"Goals: {goals}\n")

    assignment = allocator.allocate(drones, goals)

    print(f"\n✓ Final assignment: {assignment}")


if __name__ == '__main__':
    test_allocator()
