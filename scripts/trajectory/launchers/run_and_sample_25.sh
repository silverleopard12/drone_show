#!/bin/bash
# Run 25-drone scenario and sample trajectories

echo "========================================"
echo "25-Drone Swarm Trajectory Sampling"
echo "========================================"

cd /home/pjh/ego_swarm/ego-planner-swarm

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch scenario in background
echo "Launching 25-drone scenario..."
ros2 launch ego_planner scenario_swarm_25.launch.py > /tmp/swarm_25_launch.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# Wait for nodes to start
echo "Waiting for nodes to initialize (15 seconds)..."
sleep 15

# Check if nodes are running
NUM_NODES=$(ros2 node list 2>/dev/null | grep -c ego_planner || echo "0")
echo "Found $NUM_NODES planner nodes"

if [ "$NUM_NODES" -lt 20 ]; then
    echo "Warning: Expected ~25 nodes, only found $NUM_NODES"
    echo "Check /tmp/swarm_25_launch.log for errors"
fi

# Wait for planning to complete
echo "Waiting for trajectory planning (30 seconds)..."
sleep 30

# Start trajectory sampling
echo ""
echo "========================================"
echo "Starting Trajectory Sampling"
echo "========================================"

python3 scripts/trajectory/sample_trajectories.py \
    --num_drones 25 \
    --sampling_rate 50.0 \
    --duration 60.0 \
    --format all \
    --output_dir trajectories_25_drones

SAMPLE_STATUS=$?

# Stop launch
echo ""
echo "Stopping simulation..."
kill $LAUNCH_PID 2>/dev/null
sleep 2
pkill -f "scenario_swarm_25" 2>/dev/null

echo ""
echo "========================================"
if [ $SAMPLE_STATUS -eq 0 ]; then
    echo "✓ Trajectory sampling complete!"
    echo "  Output: trajectories_25_drones/"
else
    echo "✗ Trajectory sampling failed"
    echo "  Check logs for details"
fi
echo "========================================"
