#!/usr/bin/env python3
"""
Pad trajectory files to the same length by hovering at final position
All files will have the same number of lines as the longest file
"""

import os
import sys
from pathlib import Path

def pad_trajectory_files(input_dir):
    """Pad all node_*.txt files to same length with hovering"""

    input_path = Path(input_dir)
    if not input_path.exists():
        print(f"Error: Directory {input_dir} does not exist")
        return

    # Find all node_*.txt files
    node_files = sorted(input_path.glob('node_*.txt'))
    if not node_files:
        print(f"No node_*.txt files found in {input_dir}")
        return

    print(f"Found {len(node_files)} trajectory files")
    print("=" * 70)

    # Read all files and find max length
    file_data = {}
    max_lines = 0

    for filepath in node_files:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            file_data[filepath] = lines
            max_lines = max(max_lines, len(lines))
            print(f"{filepath.name}: {len(lines)} lines")

    print("=" * 70)
    print(f"Maximum length: {max_lines} lines")
    print(f"Padding all files to {max_lines} lines...")
    print("=" * 70)

    # Pad each file
    for filepath, lines in file_data.items():
        if len(lines) == max_lines:
            print(f"{filepath.name}: Already at max length, skipping")
            continue

        # Parse last line to get final state
        last_line = lines[-1].strip()
        parts = last_line.split(',')

        if len(parts) < 11:
            print(f"{filepath.name}: Invalid format, skipping")
            continue

        # Extract components: line_num, drone_id, timestamp, move, x, y, z, yaw, r, g, b
        drone_id = parts[1]
        last_timestamp = float(parts[2])
        x, y, z = parts[4], parts[5], parts[6]
        yaw = parts[7]
        r, g, b = parts[8], parts[9], parts[10]

        # Calculate time step from last two lines
        if len(lines) >= 2:
            second_last = lines[-2].strip().split(',')
            second_last_timestamp = float(second_last[2])
            dt = last_timestamp - second_last_timestamp
        else:
            dt = 0.02  # Default 50Hz

        # Pad with hovering lines
        lines_to_add = max_lines - len(lines)
        current_line_num = len(lines)
        current_timestamp = last_timestamp

        with open(filepath, 'a') as f:
            for i in range(lines_to_add):
                current_line_num += 1
                current_timestamp += dt
                # Format: line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b
                hover_line = f"{current_line_num},{drone_id},{current_timestamp:.2f},move,{x},{y},{z},{yaw},{r},{g},{b}\n"
                f.write(hover_line)

        print(f"{filepath.name}: Padded {lines_to_add} lines (hover at {x},{y},{z})")

    print("=" * 70)
    print("✓ All files padded to same length!")

    # Verify
    print("\nVerification:")
    for filepath in node_files:
        line_count = sum(1 for _ in open(filepath))
        status = "✓" if line_count == max_lines else "✗"
        print(f"  {status} {filepath.name}: {line_count} lines")


if __name__ == '__main__':
    if len(sys.argv) > 1:
        input_dir = sys.argv[1]
    else:
        # Default to most recent trajectories directory
        input_dir = 'trajectories_25_drones_20251120_215805'

    print(f"Padding trajectory files in: {input_dir}")
    print()
    pad_trajectory_files(input_dir)
