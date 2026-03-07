#!/bin/bash

# Script to test all controllers on all trajectories
# Usage: ./run_all_tests.sh [trajectory]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default trajectory
TRAJECTORY=${1:-figure8}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Multi-Controller Testing Suite${NC}"
echo -e "${BLUE}Trajectory: ${TRAJECTORY}${NC}"
echo -e "${BLUE}========================================${NC}"

# Source workspace
echo -e "\n${YELLOW}Sourcing workspace...${NC}"
cd /home/baraa-lazkani/tahsen
source install/setup.bash

# Controllers to test
CONTROLLERS=("pid" "stanley" "lqr" "smc")

# Test duration (seconds)
DURATION=60

# Function to run a single controller test
run_controller_test() {
    local controller=$1
    local trajectory=$2

    echo -e "\n${GREEN}========================================${NC}"
    echo -e "${GREEN}Testing: ${controller} controller${NC}"
    echo -e "${GREEN}Trajectory: ${trajectory}${NC}"
    echo -e "${GREEN}========================================${NC}"

    # Launch the controller
    echo -e "${YELLOW}Launching ${controller} controller...${NC}"
    timeout ${DURATION}s ros2 launch gazebo_ackermann_steering_vehicle path_follower.launch.py \
        controller:=${controller} \
        trajectory:=${trajectory} || true

    # Give time for cleanup
    sleep 2

    # Check if output file exists
    OUTPUT_FILE="src/gazebo_ackermann_steering_vehicle/test_data/results/${controller}_${trajectory}.csv"
    if [ -f "$OUTPUT_FILE" ]; then
        echo -e "${GREEN}✓ Test completed. Output saved to: ${OUTPUT_FILE}${NC}"

        # Count number of data points
        LINE_COUNT=$(wc -l < "$OUTPUT_FILE")
        echo -e "${GREEN}  Data points: $((LINE_COUNT - 1))${NC}"
    else
        echo -e "${RED}✗ Warning: Output file not found${NC}"
    fi

    # Kill any remaining processes
    pkill -f "path_follower_node" || true
    pkill -f "gazebo" || true
    sleep 2
}

# Run tests for all controllers
for controller in "${CONTROLLERS[@]}"; do
    run_controller_test "$controller" "$TRAJECTORY"
done

echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}All tests completed!${NC}"
echo -e "${BLUE}========================================${NC}"

# List generated files
echo -e "\n${YELLOW}Generated result files:${NC}"
ls -lh src/gazebo_ackermann_steering_vehicle/test_data/results/*.csv 2>/dev/null || echo "No CSV files found"

echo -e "\n${GREEN}To generate comparison plots, run:${NC}"
echo -e "  python3 src/gazebo_ackermann_steering_vehicle/scripts/compare_controllers.py \\"
echo -e "    src/gazebo_ackermann_steering_vehicle/test_data/results ${TRAJECTORY}"
