#!/bin/bash

# Script to organize test results into a structured directory

set -e

# Colors
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Organizing Test Results${NC}"
echo -e "${BLUE}========================================${NC}"

# Base directory
BASE_DIR="/home/baraa-lazkani/tahsen/src/gazebo_ackermann_steering_vehicle"
RESULTS_DIR="${BASE_DIR}/test_data/results"
ORGANIZED_DIR="${BASE_DIR}/test_data/organized_results"

# Create organized directory structure
echo -e "\n${YELLOW}Creating directory structure...${NC}"
mkdir -p "${ORGANIZED_DIR}"/{figure8,lane_change,circle,s_curve}/{csv,plots,summary}

# Move/copy CSV files to organized structure
echo -e "\n${YELLOW}Organizing CSV files...${NC}"
for trajectory in figure8 lane_change circle s_curve; do
    if ls "${RESULTS_DIR}"/*_${trajectory}.csv 1> /dev/null 2>&1; then
        cp "${RESULTS_DIR}"/*_${trajectory}.csv "${ORGANIZED_DIR}/${trajectory}/csv/" 2>/dev/null || true
        echo -e "${GREEN}✓ Organized ${trajectory} CSV files${NC}"
    fi
done

# Move/copy plot files
echo -e "\n${YELLOW}Organizing plot files...${NC}"
for trajectory in figure8 lane_change circle s_curve; do
    if ls "${RESULTS_DIR}"/*_${trajectory}.png 1> /dev/null 2>&1; then
        cp "${RESULTS_DIR}"/*_${trajectory}.png "${ORGANIZED_DIR}/${trajectory}/plots/" 2>/dev/null || true
        echo -e "${GREEN}✓ Organized ${trajectory} plot files${NC}"
    fi
    if ls "${RESULTS_DIR}"/comparison_${trajectory}.png 1> /dev/null 2>&1; then
        cp "${RESULTS_DIR}"/comparison_${trajectory}.png "${ORGANIZED_DIR}/${trajectory}/plots/" 2>/dev/null || true
    fi
done

# Create README for each trajectory
for trajectory in figure8 lane_change circle s_curve; do
    README="${ORGANIZED_DIR}/${trajectory}/README.md"

    echo "# ${trajectory^^} Trajectory Results" > "${README}"
    echo "" >> "${README}"
    echo "## Test Configuration" >> "${README}"
    echo "- **Trajectory**: ${trajectory}" >> "${README}"
    echo "- **Controllers Tested**: PID, Stanley, LQR, Sliding Mode" >> "${README}"
    echo "- **Date**: $(date '+%Y-%m-%d %H:%M:%S')" >> "${README}"
    echo "" >> "${README}"

    echo "## Files" >> "${README}"
    echo "" >> "${README}"
    echo "### CSV Data Files" >> "${README}"
    if ls "${ORGANIZED_DIR}/${trajectory}/csv"/*.csv 1> /dev/null 2>&1; then
        for file in "${ORGANIZED_DIR}/${trajectory}/csv"/*.csv; do
            filename=$(basename "$file")
            size=$(du -h "$file" | cut -f1)
            lines=$(wc -l < "$file")
            echo "- \`${filename}\` - ${size}, $((lines-1)) data points" >> "${README}"
        done
    else
        echo "- No CSV files found" >> "${README}"
    fi
    echo "" >> "${README}"

    echo "### Plots" >> "${README}"
    if ls "${ORGANIZED_DIR}/${trajectory}/plots"/*.png 1> /dev/null 2>&1; then
        for file in "${ORGANIZED_DIR}/${trajectory}/plots"/*.png; do
            filename=$(basename "$file")
            echo "- \`${filename}\`" >> "${README}"
        done
    else
        echo "- No plot files found" >> "${README}"
    fi
    echo "" >> "${README}"

    echo "## Quick Analysis" >> "${README}"
    echo "" >> "${README}"
    echo "To view individual controller results:" >> "${README}"
    echo "\`\`\`bash" >> "${README}"
    echo "python3 scripts/plot_results.py test_data/organized_results/${trajectory}/csv/pid_${trajectory}.csv" >> "${README}"
    echo "\`\`\`" >> "${README}"
    echo "" >> "${README}"
    echo "To compare all controllers:" >> "${README}"
    echo "\`\`\`bash" >> "${README}"
    echo "python3 scripts/compare_controllers.py test_data/organized_results/${trajectory}/csv ${trajectory}" >> "${README}"
    echo "\`\`\`" >> "${README}"

    echo -e "${GREEN}✓ Created README for ${trajectory}${NC}"
done

# Create master summary
MASTER_SUMMARY="${ORGANIZED_DIR}/MASTER_SUMMARY.md"
echo "# Multi-Controller Test Results Summary" > "${MASTER_SUMMARY}"
echo "" >> "${MASTER_SUMMARY}"
echo "**Generated**: $(date '+%Y-%m-%d %H:%M:%S')" >> "${MASTER_SUMMARY}"
echo "" >> "${MASTER_SUMMARY}"

echo "## Directory Structure" >> "${MASTER_SUMMARY}"
echo "\`\`\`" >> "${MASTER_SUMMARY}"
tree -L 2 "${ORGANIZED_DIR}" 2>/dev/null || find "${ORGANIZED_DIR}" -maxdepth 2 -type d >> "${MASTER_SUMMARY}"
echo "\`\`\`" >> "${MASTER_SUMMARY}"
echo "" >> "${MASTER_SUMMARY}"

echo "## Available Trajectories" >> "${MASTER_SUMMARY}"
for trajectory in figure8 lane_change circle s_curve; do
    csv_count=$(ls "${ORGANIZED_DIR}/${trajectory}/csv"/*.csv 2>/dev/null | wc -l)
    plot_count=$(ls "${ORGANIZED_DIR}/${trajectory}/plots"/*.png 2>/dev/null | wc -l)

    if [ $csv_count -gt 0 ] || [ $plot_count -gt 0 ]; then
        echo "### ${trajectory^^}" >> "${MASTER_SUMMARY}"
        echo "- CSV files: ${csv_count}" >> "${MASTER_SUMMARY}"
        echo "- Plots: ${plot_count}" >> "${MASTER_SUMMARY}"
        echo "- Details: See \`${trajectory}/README.md\`" >> "${MASTER_SUMMARY}"
        echo "" >> "${MASTER_SUMMARY}"
    fi
done

echo "## Controllers Tested" >> "${MASTER_SUMMARY}"
echo "1. **PID Controller** - Proportional-Integral-Derivative control" >> "${MASTER_SUMMARY}"
echo "2. **Stanley Controller** - Velocity-adaptive path tracking" >> "${MASTER_SUMMARY}"
echo "3. **LQR Controller** - Linear Quadratic Regulator (optimal control)" >> "${MASTER_SUMMARY}"
echo "4. **Sliding Mode Controller** - Robust nonlinear control" >> "${MASTER_SUMMARY}"
echo "" >> "${MASTER_SUMMARY}"

echo "## Quick Start" >> "${MASTER_SUMMARY}"
echo "" >> "${MASTER_SUMMARY}"
echo "View comparison for a specific trajectory:" >> "${MASTER_SUMMARY}"
echo "\`\`\`bash" >> "${MASTER_SUMMARY}"
echo "cd /home/baraa-lazkani/tahsen/src/gazebo_ackermann_steering_vehicle" >> "${MASTER_SUMMARY}"
echo "python3 scripts/compare_controllers.py test_data/organized_results/figure8/csv figure8" >> "${MASTER_SUMMARY}"
echo "\`\`\`" >> "${MASTER_SUMMARY}"

echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}Organization Complete!${NC}"
echo -e "${BLUE}========================================${NC}"

echo -e "\n${GREEN}Results organized in:${NC}"
echo -e "  ${ORGANIZED_DIR}"

echo -e "\n${GREEN}Master summary:${NC}"
echo -e "  ${MASTER_SUMMARY}"

echo -e "\n${YELLOW}View the summary:${NC}"
echo -e "  cat ${MASTER_SUMMARY}"
