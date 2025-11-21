#!/usr/bin/env python3
"""
Offline Mission Analysis - Analyzes mission from ROS2 bag
No need for real-time node, much more efficient
"""

import sys
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def analyze_mission(bag_path, num_drones, targets, arrival_threshold=0.5):
    """
    Analyze mission completion times from bag file

    Args:
        bag_path: Path to ROS2 bag
        num_drones: Number of drones
        targets: Dict {drone_id: (x, y, z)}
        arrival_threshold: Distance threshold (m)
    """

    # Setup bag reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Track mission state
    start_time = None
    drones_arrived = {}

    print(f"Analyzing bag: {bag_path}")
    print(f"Tracking {num_drones} drones...")
    print("=" * 70)

    # Read all messages
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # Check for start trigger
        if topic == '/traj_start_trigger':
            if start_time is None:
                msg = deserialize_message(data, PoseStamped)
                start_time = timestamp
                print(f"Mission started at: {timestamp / 1e9:.2f}s")

        # Check drone odometry
        for drone_id in range(num_drones):
            odom_topic = f'/drone_{drone_id}_visual_slam/odom'

            if topic == odom_topic and drone_id not in drones_arrived:
                if start_time is None:
                    continue  # Mission not started yet

                msg = deserialize_message(data, Odometry)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z

                # Get target
                tx, ty, tz = targets[drone_id]

                # Calculate distance
                dist = math.sqrt((x - tx)**2 + (y - ty)**2 + (z - tz)**2)

                if dist < arrival_threshold:
                    arrival_time = (timestamp - start_time) / 1e9  # Convert to seconds
                    drones_arrived[drone_id] = arrival_time
                    print(f"Drone {drone_id:2d} arrived in {arrival_time:.2f}s ({len(drones_arrived)}/{num_drones})")

    # Print results
    if len(drones_arrived) == 0:
        print("No arrivals detected!")
        return

    arrival_times = list(drones_arrived.values())

    print("")
    print("=" * 70)
    print("MISSION ANALYSIS COMPLETE")
    print("=" * 70)
    print(f"Drones arrived:     {len(drones_arrived)}/{num_drones}")
    print(f"First arrival:      {min(arrival_times):.2f} seconds")
    print(f"Last arrival:       {max(arrival_times):.2f} seconds")
    print(f"Average arrival:    {sum(arrival_times)/len(arrival_times):.2f} seconds")
    print("=" * 70)

    if len(drones_arrived) < num_drones:
        missing = set(range(num_drones)) - set(drones_arrived.keys())
        print(f"Missing drones: {sorted(missing)}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_mission.py <bag_path>")
        print("Example: python3 analyze_mission.py rosbag2_2023_11_03-10_30_00/")
        sys.exit(1)

    bag_path = sys.argv[1]

    # Example: 36 drone scenario (modify as needed)
    num_drones = 36
    targets = {
        0: (3.0, 20.5, 19.7),
        1: (3.0, 26.9, 19.7),
        # ... add all 36 targets
    }

    analyze_mission(bag_path, num_drones, targets)
