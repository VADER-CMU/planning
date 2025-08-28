#!/bin/bash

# Test script to verify planner_tester setup in VADER-CMU repository
echo "=========================================="
echo "Testing Planner Tester Setup"
echo "=========================================="

# Check if we're in the right directory
if [[ ! -f "CMakeLists.txt" ]]; then
    echo "Error: Please run this script from the vader_planner directory"
    exit 1
fi

echo "✓ CMakeLists.txt found"

# Check if required files exist
REQUIRED_FILES=(
    "src/planner_tester.cpp"
    "launch/planner_tester.launch"
    "config/planner_tester_ompl.yaml"
    "scripts/run_planner_tester.sh"
    "srv/pose_plan.srv"
    "srv/joint_plan.srv"
    "srv/exec_plan.srv"
    "srv/single_straight_plan.srv"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        echo "✓ $file exists"
    else
        echo "✗ $file missing"
        exit 1
    fi
done

# Check if scripts are executable
SCRIPTS=(
    "scripts/run_planner_tester.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [[ -x "$script" ]]; then
        echo "✓ $script is executable"
    else
        echo "✗ $script is not executable"
        exit 1
    fi
done

echo ""
echo "=== Setup Test Results ==="
echo "✓ All planner_tester components are ready!"
echo ""
echo "You can now:"
echo "1. Build the project: catkin_make"
echo "2. Launch the tester: roslaunch vader_planner planner_tester.launch"
echo "3. Run automated tests: ./scripts/run_planner_tester.sh"
echo ""
echo "=== File Locations ==="
echo "Planner Tester C++: src/planner_tester.cpp"
echo "Launch File: launch/planner_tester.launch"
echo "Config File: config/planner_tester_ompl.yaml"
echo "Test Script: scripts/run_planner_tester.sh"
echo "Service Files: srv/*.srv"
echo "README: README_PLANNER_TESTER.md" 