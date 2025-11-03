#!/usr/bin/env python3
"""
Mission Timer Node
Tracks mission start time and each drone's arrival time
Reports statistics when all drones reach their targets
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from functools import partial
import math
import time


class MissionTimer(Node):
    def __init__(self):
        super().__init__('mission_timer')

        # Parameters
        self.declare_parameter('num_drones', 36)
        self.declare_parameter('arrival_threshold', 0.5)  # 0.5m from target

        self.num_drones = self.get_parameter('num_drones').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value

        # Load target positions from parameters
        self.targets = {}
        for drone_id in range(self.num_drones):
            self.declare_parameter(f'drone_{drone_id}_target_x', 0.0)
            self.declare_parameter(f'drone_{drone_id}_target_y', 0.0)
            self.declare_parameter(f'drone_{drone_id}_target_z', 0.0)

            target_x = self.get_parameter(f'drone_{drone_id}_target_x').value
            target_y = self.get_parameter(f'drone_{drone_id}_target_y').value
            target_z = self.get_parameter(f'drone_{drone_id}_target_z').value

            self.targets[drone_id] = (target_x, target_y, target_z)

        self.get_logger().info(f'Mission Timer started for {self.num_drones} drones')

        # State tracking
        self.mission_started = False
        self.start_time = None
        self.drones_arrived = {}  # drone_id -> arrival_time
        self.drones_announced = set()  # Track which arrivals we've announced

        # Subscribe to trigger
        self.trigger_sub = self.create_subscription(
            PoseStamped,
            '/traj_start_trigger',
            self.trigger_callback,
            10
        )

        # Subscribe to all drone odometry
        self.odom_subs = []
        for drone_id in range(self.num_drones):
            topic = f'/drone_{drone_id}_visual_slam/odom'

            # Use functools.partial to properly capture drone_id
            callback = partial(self.odom_callback, drone_id=drone_id)

            sub = self.create_subscription(
                Odometry,
                topic,
                callback,
                10
            )
            self.odom_subs.append(sub)

        self.get_logger().info(f'Monitoring {self.num_drones} drones for arrival')

        # Timer to check completion
        self.check_timer = self.create_timer(1.0, self.check_completion)

    def trigger_callback(self, msg):
        """Mission start trigger received"""
        if not self.mission_started:
            self.mission_started = True
            self.start_time = time.time()
            self.get_logger().info('=' * 70)
            self.get_logger().info('MISSION STARTED - Timer begins now!')
            self.get_logger().info('=' * 70)

    def odom_callback(self, msg, drone_id):
        """Check if drone has arrived at target"""
        if not self.mission_started:
            return

        if drone_id in self.drones_arrived:
            return  # Already arrived

        # Get current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Get target position
        tx, ty, tz = self.targets[drone_id]

        # Calculate distance to target
        dist = math.sqrt((x - tx)**2 + (y - ty)**2 + (z - tz)**2)

        # Check if arrived
        if dist < self.arrival_threshold:
            arrival_time = time.time() - self.start_time
            self.drones_arrived[drone_id] = arrival_time

            if drone_id not in self.drones_announced:
                self.drones_announced.add(drone_id)
                self.get_logger().info(
                    f'Drone {drone_id:2d} ARRIVED at target in {arrival_time:.2f}s '
                    f'({len(self.drones_arrived)}/{self.num_drones})'
                )

    def check_completion(self):
        """Check if all drones have arrived"""
        if not self.mission_started:
            return

        if len(self.drones_arrived) >= self.num_drones:
            # All drones arrived!
            total_time = time.time() - self.start_time
            arrival_times = list(self.drones_arrived.values())

            self.get_logger().info('')
            self.get_logger().info('=' * 70)
            self.get_logger().info('ALL DRONES ARRIVED - MISSION COMPLETE!')
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'Total mission time: {total_time:.2f} seconds')
            self.get_logger().info(f'First arrival:      {min(arrival_times):.2f} seconds')
            self.get_logger().info(f'Last arrival:       {max(arrival_times):.2f} seconds')
            self.get_logger().info(f'Average arrival:    {sum(arrival_times)/len(arrival_times):.2f} seconds')
            self.get_logger().info('=' * 70)

            # Stop checking
            self.check_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MissionTimer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
