#!/usr/bin/env python3
"""
Calculate fair Hungarian assignment for 25 drones
From FORMATION_25_A (5x5 Grid) to FORMATION_25_B (Triangle)
Saves results to txt file for reference
"""
import sys
import os
from datetime import datetime

# Add scripts directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Allocation.Fair_Hungarian_Allocator import HungarianAllocator
from formations_large import FORMATION_25_A, FORMATION_25_B


def main():
    print("=" * 80)
    print("Fair Hungarian Assignment for 25 Drones")
    print("From FORMATION_25_A (5x5 Grid) to FORMATION_25_B (Triangle)")
    print("=" * 80)

    # Create allocator
    allocator = HungarianAllocator(max_iterations=100)

    # Calculate fair assignment (min-max optimization)
    print("\nCalculating fair assignment (minimizing max distance)...")
    assignment = allocator.allocate_fair(FORMATION_25_A, FORMATION_25_B)

    # Prepare output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"assignment_25_drones_{timestamp}.txt"
    output_path = os.path.join(os.path.dirname(__file__), output_file)

    with open(output_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("Fair Hungarian Assignment for 25 Drones\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 80 + "\n\n")

        f.write("Formation A (Init): 5x5 Grid (spacing 4.0m Y, 4.0m Z)\n")
        f.write("Formation B (Target): Triangle (spacing 3.2m)\n\n")

        f.write("=" * 80 + "\n")
        f.write("Drone Configurations for scenario_swarm_25.launch.py:\n")
        f.write("=" * 80 + "\n\n")

        f.write("drone_configs = [\n")

        for drone_id in range(25):
            init_pos = FORMATION_25_A[drone_id]
            target_idx = assignment[drone_id]
            target_pos = FORMATION_25_B[target_idx]

            distance = allocator.calculate_distance(init_pos, target_pos)

            config_line = (
                f"    {{'drone_id': {drone_id}, "
                f"'init_x': {init_pos[0]}, 'init_y': {init_pos[1]}, 'init_z': {init_pos[2]}, "
                f"'target_x': {target_pos[0]}, 'target_y': {target_pos[1]}, 'target_z': {target_pos[2]}}},  "
                f"# Distance: {distance:.2f}m\n"
            )

            f.write(config_line)
            print(config_line.rstrip())

        f.write("]\n")
        print("]")

        f.write("\n" + "=" * 80 + "\n")
        f.write("Assignment Mapping (Drone ID -> Target ID):\n")
        f.write("=" * 80 + "\n")

        for drone_id, target_id in enumerate(assignment):
            f.write(f"Drone {drone_id:2d} -> Target {target_id:2d}\n")

    print("\n" + "=" * 80)
    print(f"Assignment saved to: {output_path}")
    print("=" * 80)


if __name__ == '__main__':
    main()
