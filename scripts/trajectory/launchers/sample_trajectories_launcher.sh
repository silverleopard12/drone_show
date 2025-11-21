#!/bin/bash
# Launch script for trajectory sampling
# Usage: ./sample_trajectories_launcher.sh [num_drones] [sampling_rate] [duration]

NUM_DRONES=${1:-36}
SAMPLING_RATE=${2:-50.0}
DURATION=${3:-60.0}

echo "=================================="
echo "Trajectory Sampling Launcher"
echo "=================================="
echo "Configuration:"
echo "  Drones: $NUM_DRONES"
echo "  Sampling rate: $SAMPLING_RATE Hz"
echo "  Duration: $DURATION seconds"
echo "  Format: DroneShow"
echo "=================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2 environment if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 environment..."
    source /opt/ros/humble/setup.bash
    source "$SCRIPT_DIR/../install/setup.bash"
fi

# Run the trajectory sampler (now in trajectory/ subdirectory)
python3 "$SCRIPT_DIR/../sample_trajectories.py" \
    --num_drones "$NUM_DRONES" \
    --sampling_rate "$SAMPLING_RATE" \
    --duration "$DURATION"
