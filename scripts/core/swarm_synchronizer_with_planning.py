#!/usr/bin/env python3
"""
Swarm Synchronizer Node - Waits for all drones to complete initial trajectory planning
Monitors planning status of all drones and triggers synchronized start when all are ready
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
import time


class SwarmSynchronizerWithPlanning(Node):
    def __init__(self):
        super().__init__('swarm_synchronizer_with_planning')

        # Parameters
        self.declare_parameter('num_drones', 36)
        self.declare_parameter('timeout', 120.0)  # 120 seconds timeout for planning

        self.num_drones = self.get_parameter('num_drones').value
        self.timeout = self.get_parameter('timeout').value

        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Swarm Synchronizer started - waiting for {self.num_drones} drones to complete planning')
        self.get_logger().info('=' * 80)

        # Track planning status
        self.planning_ready = {}  # {drone_id: bool}
        self.start_time = time.time()
        self.triggered = False

        # Subscribe to planning ready status from each drone
        self.ready_subs = []
        for drone_id in range(self.num_drones):
            sub = self.create_subscription(
                Bool,
                f'/drone_{drone_id}_planning/ready',
                lambda msg, did=drone_id: self.ready_callback(msg, did),
                10
            )
            self.ready_subs.append(sub)
            self.planning_ready[drone_id] = False

        # Publisher for trigger
        self.trigger_pub = self.create_publisher(PoseStamped, '/traj_start_trigger', 10)

        # Timer to check if all ready and report status
        self.check_timer = self.create_timer(1.0, self.check_status)

    def ready_callback(self, msg, drone_id):
        """Called when a drone reports planning ready status"""
        if not self.planning_ready[drone_id] and msg.data:
            self.planning_ready[drone_id] = True
            ready_count = sum(self.planning_ready.values())
            self.get_logger().info(f'✓ Drone {drone_id} planning complete! ({ready_count}/{self.num_drones} ready)')

    def check_status(self):
        """Check planning status and trigger when all ready"""
        if self.triggered:
            return

        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            ready_count = sum(self.planning_ready.values())
            self.get_logger().error('=' * 80)
            self.get_logger().error(f'TIMEOUT! Only {ready_count}/{self.num_drones} drones ready after {self.timeout}s')
            self.get_logger().error('Not ready drones: ' + str([i for i, ready in self.planning_ready.items() if not ready]))
            self.get_logger().error('=' * 80)
            raise SystemExit

        # Check if all drones are ready
        ready_count = sum(self.planning_ready.values())
        if ready_count == self.num_drones:
            self.trigger_synchronized_start()
        elif ready_count > 0 and int(elapsed) % 5 == 0:  # Report every 5 seconds
            self.get_logger().info(f'Progress: {ready_count}/{self.num_drones} drones ready ({elapsed:.1f}s elapsed)')

    def trigger_synchronized_start(self):
        """Send trigger to all drones to start mission"""
        if self.triggered:
            return

        elapsed = time.time() - self.start_time

        self.get_logger().info('')
        self.get_logger().info('=' * 80)
        self.get_logger().info('🎯 ALL DRONES PLANNING COMPLETE!')
        self.get_logger().info(f'   Total planning time: {elapsed:.2f} seconds')
        self.get_logger().info('   TRIGGERING SYNCHRONIZED START!')
        self.get_logger().info('=' * 80)
        self.get_logger().info('')

        # Publish trigger
        trigger_msg = PoseStamped()
        trigger_msg.header.stamp = self.get_clock().now().to_msg()
        trigger_msg.header.frame_id = 'world'

        # Publish multiple times to ensure all drones receive it
        for _ in range(10):
            self.trigger_pub.publish(trigger_msg)
            time.sleep(0.1)

        self.triggered = True
        self.check_timer.cancel()

        self.get_logger().info('✓ Trigger sent successfully!')
        self.get_logger().info('  Mission started - all drones executing trajectories')
        self.get_logger().info('  Synchronizer shutting down - job complete!')

        # Shutdown node after trigger
        time.sleep(1.0)
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSynchronizerWithPlanning()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
