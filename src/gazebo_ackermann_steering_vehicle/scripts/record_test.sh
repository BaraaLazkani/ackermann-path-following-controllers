#!/bin/bash

# Script to run a controller test with optional video recording
# Usage: ./record_test.sh <controller> <trajectory> [record]

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Parameters
CONTROLLER=${1:-pid}
TRAJECTORY=${2:-figure8}
RECORD=${3:-false}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Controller Test with Recording${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Controller: ${CONTROLLER}${NC}"
echo -e "${BLUE}Trajectory: ${TRAJECTORY}${NC}"
echo -e "${BLUE}Recording: ${RECORD}${NC}"
echo -e "${BLUE}========================================${NC}"

# Source workspace
cd /home/baraa-lazkani/tahsen
source install/setup.bash

# Create directories
RESULTS_DIR="src/gazebo_ackermann_steering_vehicle/test_data/results"
VIDEO_DIR="${RESULTS_DIR}/videos"
mkdir -p "${VIDEO_DIR}"

# Test duration
DURATION=60

# Output files
CSV_FILE="${RESULTS_DIR}/${CONTROLLER}_${TRAJECTORY}.csv"
VIDEO_FILE="${VIDEO_DIR}/${CONTROLLER}_${TRAJECTORY}.mp4"
BAG_FILE="${VIDEO_DIR}/${CONTROLLER}_${TRAJECTORY}"

echo -e "\n${YELLOW}Output files:${NC}"
echo -e "  CSV: ${CSV_FILE}"
if [ "$RECORD" = "true" ]; then
    echo -e "  ROS2 Bag: ${BAG_FILE}"
    echo -e "  Video: ${VIDEO_FILE} (if conversion successful)"
fi

# Start video recording if requested
RECORD_PID=""
if [ "$RECORD" = "true" ]; then
    echo -e "\n${YELLOW}Starting ROS2 bag recording...${NC}"

    # Record camera topic and TF
    ros2 bag record \
        -o "${BAG_FILE}" \
        /camera/image_raw \
        /tf \
        /tf_static \
        --max-cache-size 0 &
    RECORD_PID=$!

    echo -e "${GREEN}Recording started (PID: ${RECORD_PID})${NC}"
    sleep 2
fi

# Launch the controller
echo -e "\n${GREEN}Launching ${CONTROLLER} controller...${NC}"
timeout ${DURATION}s ros2 launch gazebo_ackermann_steering_vehicle path_follower.launch.py \
    controller:=${CONTROLLER} \
    trajectory:=${TRAJECTORY} || true

# Stop recording if it was started
if [ ! -z "$RECORD_PID" ]; then
    echo -e "\n${YELLOW}Stopping recording...${NC}"
    kill $RECORD_PID 2>/dev/null || true
    sleep 2
fi

# Check if output file exists
if [ -f "$CSV_FILE" ]; then
    LINE_COUNT=$(wc -l < "$CSV_FILE")
    echo -e "\n${GREEN}✓ Test completed successfully${NC}"
    echo -e "${GREEN}  CSV data points: $((LINE_COUNT - 1))${NC}"
else
    echo -e "\n${RED}✗ Warning: CSV file not found${NC}"
fi

# Kill any remaining processes
pkill -f "path_follower_node" || true
pkill -f "gazebo" || true
sleep 2

echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}Test Complete!${NC}"
echo -e "${BLUE}========================================${NC}"

# Provide next steps
echo -e "\n${YELLOW}Next steps:${NC}"
echo -e "  1. View results:"
echo -e "     python3 src/gazebo_ackermann_steering_vehicle/scripts/plot_results.py ${CSV_FILE}"
echo -e ""
echo -e "  2. Compare with other controllers:"
echo -e "     python3 src/gazebo_ackermann_steering_vehicle/scripts/compare_controllers.py \\"
echo -e "       src/gazebo_ackermann_steering_vehicle/test_data/results ${TRAJECTORY}"

if [ "$RECORD" = "true" ] && [ -d "${BAG_FILE}" ]; then
    echo -e ""
    echo -e "  3. View recorded bag:"
    echo -e "     ros2 bag info ${BAG_FILE}"
    echo -e ""
    echo -e "  4. Play back recording:"
    echo -e "     ros2 bag play ${BAG_FILE}"
fi
