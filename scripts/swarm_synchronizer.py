#!/usr/bin/env python3
"""
Swarm Synchronizer Node
Waits 5 seconds after first drone spawn, then triggers synchronized start
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time


class SwarmSynchronizer(Node):
    def __init__(self):
        super().__init__('swarm_synchronizer')

        # Parameters
        self.declare_parameter('num_drones', 36)
        self.declare_parameter('wait_time', 5.0)  # Wait 5 seconds after first spawn

        self.num_drones = self.get_parameter('num_drones').value
        self.wait_time = self.get_parameter('wait_time').value

        self.get_logger().info(f'Swarm Synchronizer started - will trigger {self.wait_time}s after first spawn')

        # Track spawn start time
        self.spawn_start_time = None
        self.triggered = False

        # Subscribe to first drone odometry to detect spawn start
        self.first_drone_sub = self.create_subscription(
            Odometry,
            '/drone_0_visual_slam/odom',
            self.first_drone_callback,
            10
        )

        # Publisher for trigger
        self.trigger_pub = self.create_publisher(PoseStamped, '/traj_start_trigger', 10)

        # Timer to check if wait time elapsed
        self.check_timer = self.create_timer(0.5, self.check_trigger_time)

    def first_drone_callback(self, msg):
        """First drone spawned - start countdown"""
        if self.spawn_start_time is None:
            self.spawn_start_time = time.time()
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'First drone spawned! Waiting {self.wait_time} seconds for all drones...')
            self.get_logger().info('=' * 70)

    def check_trigger_time(self):
        """Check if wait time has elapsed and trigger if so"""
        if self.triggered or self.spawn_start_time is None:
            return

        elapsed = time.time() - self.spawn_start_time
        if elapsed >= self.wait_time:
            # Publish trigger
            trigger_msg = PoseStamped()
            trigger_msg.header.stamp = self.get_clock().now().to_msg()
            trigger_msg.header.frame_id = 'world'

            self.get_logger().info('=' * 70)
            self.get_logger().info('TRIGGERING SYNCHRONIZED START!')
            self.get_logger().info('=' * 70)

            # Publish multiple times to ensure all drones receive it
            for _ in range(10):
                self.trigger_pub.publish(trigger_msg)
                time.sleep(0.1)

            self.triggered = True
            self.check_timer.cancel()
            self.get_logger().info('Trigger sent. Mission started!')
            self.get_logger().info('Synchronizer shutting down - job complete!')

            # Shutdown node after trigger
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
