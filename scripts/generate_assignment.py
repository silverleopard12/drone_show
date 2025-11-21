#!/usr/bin/env python3
"""
Unified assignment generator for drone formations
Supports multiple allocation algorithms:
- Fair Hungarian (min-max optimization)
- CAT-ORA (collision-aware time-optimal)

Results stored in Allocation/{algorithm}_{num_drones}/ directories
"""
import sys
import os
from datetime import datetime
import argparse

# Add scripts directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Allocation.algorithms.Fair_Hungarian_Allocator import HungarianAllocator
from planning.formations_large import (
    FORMATION_25_A, FORMATION_25_B,
    FORMATION_36_A, FORMATION_36_B
)

FORMATIONS = {
    25: {
        'A': FORMATION_25_A,
        'B': FORMATION_25_B,
        'description': '5x5 Grid -> Triangle'
    },
    36: {
        'A': FORMATION_36_A,
        'B': FORMATION_36_B,
        'description': '6x6 Grid -> Triangle'
    }
}

ALGORITHMS = {
    'hungarian': 'Fair Hungarian (min-max distance)',
    'catora': 'CAT-ORA (collision-aware time-optimal)'
}


def get_assignment_path(algorithm, num_drones):
    """Get the path to the assignment file for given algorithm and number of drones"""
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # New naming: Fair_Hungarian_25, Fair_Hungarian_36, CATORA_25, CATORA_36
    if algorithm == 'hungarian':
        folder_name = f"Fair_Hungarian_{num_drones}"
    elif algorithm == 'catora':
        folder_name = f"CATORA_{num_drones}"
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")

    allocation_dir = os.path.join(script_dir, "Allocation", "results", folder_name)
    os.makedirs(allocation_dir, exist_ok=True)
    return os.path.join(allocation_dir, f"assignment_{num_drones}_drones.txt")


def generate_assignment_hungarian(num_drones, formation_a, formation_b):
    """Generate assignment using Fair Hungarian algorithm"""
    allocator = HungarianAllocator(max_iterations=100)
    assignment = allocator.allocate_fair(formation_a, formation_b)

    distances = []
    for drone_id in range(num_drones):
        init_pos = formation_a[drone_id]
        target_idx = assignment[drone_id]
        target_pos = formation_b[target_idx]
        distance = allocator.calculate_distance(init_pos, target_pos)
        distances.append(distance)

    return assignment, distances


def generate_assignment_catora(num_drones, formation_a, formation_b):
    """Generate assignment using CAT-ORA algorithm (Standalone Python mode)"""
    # Import CATORA allocator
    try:
        from Allocation.algorithms.CATORA_Allocator import create_allocator
    except ImportError as e:
        print(f"❌ Error importing CATORA allocator: {e}")
        raise

    # Create CATORA allocator (standalone mode - no ROS2 required)
    allocator = create_allocator(use_ros2_service=False)

    # Call CATORA standalone
    assignment = allocator.allocate(formation_a, formation_b)

    # Calculate distances
    distances = []
    for drone_id in range(num_drones):
        init_pos = formation_a[drone_id]
        target_idx = assignment[drone_id]
        target_pos = formation_b[target_idx]
        distance = allocator.calculate_distance(init_pos, target_pos)
        distances.append(distance)

    return assignment, distances


def generate_assignment(algorithm, num_drones, force_regenerate=False):
    """Generate or load assignment for given algorithm and number of drones"""

    if algorithm not in ALGORITHMS:
        print(f"❌ Error: Unknown algorithm '{algorithm}'")
        print(f"Available: {list(ALGORITHMS.keys())}")
        return None

    if num_drones not in FORMATIONS:
        print(f"❌ Error: No formations defined for {num_drones} drones")
        print(f"Available: {list(FORMATIONS.keys())}")
        return None

    print("=" * 80)
    print(f"{algorithm.upper()} Assignment for {num_drones} Drones")
    print(f"Algorithm: {ALGORITHMS[algorithm]}")
    print(f"Formation: {FORMATIONS[num_drones]['description']}")
    print("=" * 80)

    output_path = get_assignment_path(algorithm, num_drones)

    # Check if assignment already exists
    if os.path.exists(output_path) and not force_regenerate:
        print(f"\n✓ Existing assignment found: {output_path}")

        # Read and display basic info
        with open(output_path, 'r') as f:
            lines = f.readlines()
            for line in lines[:10]:  # Show first 10 lines
                print(line.rstrip())

        print(f"\nUsing existing assignment (use --force to regenerate)")
        print("=" * 80)
        return output_path

    # Generate new assignment
    print(f"\n{'Regenerating' if os.path.exists(output_path) else 'Generating'} assignment...")

    formation_a = FORMATIONS[num_drones]['A']
    formation_b = FORMATIONS[num_drones]['B']

    # Call appropriate algorithm
    if algorithm == 'hungarian':
        assignment, distances = generate_assignment_hungarian(num_drones, formation_a, formation_b)
    elif algorithm == 'catora':
        assignment, distances = generate_assignment_catora(num_drones, formation_a, formation_b)
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")

    # Save to file
    with open(output_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write(f"{algorithm.upper()} Assignment for {num_drones} Drones\n")
        f.write(f"Algorithm: {ALGORITHMS[algorithm]}\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Formation: {FORMATIONS[num_drones]['description']}\n")
        f.write("=" * 80 + "\n\n")

        f.write(f"Drone Configurations for scenario_swarm_{num_drones}.launch.py:\n")
        f.write("=" * 80 + "\n\n")

        f.write("drone_configs = [\n")

        for drone_id in range(num_drones):
            init_pos = formation_a[drone_id]
            target_idx = assignment[drone_id]
            target_pos = formation_b[target_idx]
            distance = distances[drone_id]

            config_line = (
                f"    {{'drone_id': {drone_id}, "
                f"'init_x': {init_pos[0]}, 'init_y': {init_pos[1]}, 'init_z': {init_pos[2]}, "
                f"'target_x': {target_pos[0]}, 'target_y': {target_pos[1]}, 'target_z': {target_pos[2]}}},  "
                f"# Distance: {distance:.2f}m\n"
            )

            f.write(config_line)

        f.write("]\n")

        f.write("\n" + "=" * 80 + "\n")
        f.write("Assignment Mapping (Drone ID -> Target ID):\n")
        f.write("=" * 80 + "\n")

        for drone_id, target_id in enumerate(assignment):
            f.write(f"Drone {drone_id:2d} -> Target {target_id:2d}\n")

        f.write("\n" + "=" * 80 + "\n")
        f.write("Statistics:\n")
        f.write("=" * 80 + "\n")
        f.write(f"Algorithm: {algorithm.upper()}\n")
        f.write(f"Total drones: {num_drones}\n")
        f.write(f"Min distance: {min(distances):.2f}m\n")
        f.write(f"Max distance: {max(distances):.2f}m\n")
        f.write(f"Avg distance: {sum(distances)/len(distances):.2f}m\n")

    print(f"\n✓ Assignment saved to: {output_path}")
    print(f"\nStatistics:")
    print(f"  Algorithm: {algorithm.upper()}")
    print(f"  Min distance: {min(distances):.2f}m")
    print(f"  Max distance: {max(distances):.2f}m")
    print(f"  Avg distance: {sum(distances)/len(distances):.2f}m")
    print("=" * 80)

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate or load drone formation assignments',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate Hungarian assignment for 36 drones
  python3 generate_assignment.py --algorithm hungarian --num_drones 36

  # Generate CAT-ORA assignment for 25 drones
  python3 generate_assignment.py --algorithm catora --num_drones 25

  # Force regenerate
  python3 generate_assignment.py --algorithm hungarian --num_drones 36 --force

  # List all existing assignments
  python3 generate_assignment.py --list

Output directories:
  Allocation/Fair_Hungarian_25/
  Allocation/Fair_Hungarian_36/
  Allocation/CATORA_25/
  Allocation/CATORA_36/
        """
    )

    parser.add_argument('--algorithm', type=str, default='hungarian',
                       choices=['hungarian', 'catora'],
                       help='Allocation algorithm (default: hungarian)')
    parser.add_argument('--num_drones', type=int, default=36,
                       help='Number of drones (default: 36)')
    parser.add_argument('--force', action='store_true',
                       help='Force regenerate even if assignment exists')
    parser.add_argument('--list', action='store_true',
                       help='List all existing assignments')

    args = parser.parse_args()

    if args.list:
        print("=" * 80)
        print("Existing Assignments:")
        print("=" * 80)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        allocation_dir = os.path.join(script_dir, "Allocation")

        found_any = False
        for algorithm in ALGORITHMS.keys():
            for num in sorted(FORMATIONS.keys()):
                path = get_assignment_path(algorithm, num)
                if os.path.exists(path):
                    # Get file modification time
                    mtime = os.path.getmtime(path)
                    mtime_str = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')

                    if algorithm == 'hungarian':
                        algo_label = "Fair Hungarian"
                    elif algorithm == 'catora':
                        algo_label = "CAT-ORA"
                    else:
                        algo_label = algorithm

                    print(f"  [{algo_label}] {num} drones: {path}")
                    print(f"    Last modified: {mtime_str}")
                    found_any = True

        if not found_any:
            print("  No assignments found. Generate one with:")
            print("    python3 generate_assignment.py --algorithm hungarian --num_drones 36")
            print("    python3 generate_assignment.py --algorithm catora --num_drones 25")

        print("=" * 80)
        return

    # Generate or load assignment
    generate_assignment(args.algorithm, args.num_drones, args.force)


if __name__ == '__main__':
    main()
