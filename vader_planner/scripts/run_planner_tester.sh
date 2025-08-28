#!/bin/bash

# Planner Tester Automation Script
# This script runs inside the Docker container to automate RRT* parameter testing
# Usage: ./run_planner_tester.sh [num_trials]

set -e

# Default number of trials
NUM_TRIALS=${1:-50}

echo "=========================================="
echo "Planner Tester - RRT* Parameter Testing"
echo "=========================================="
echo "Number of trials: $NUM_TRIALS"
echo "Current directory: $(pwd)"
echo "=========================================="

# Navigate to workspace
cd /root/catkin_ws

echo "Building project..."
# Build the project
catkin_make

echo "Sourcing workspace..."
# Source the workspace
source devel/setup.bash

echo "Clearing previous results..."
# Clear previous results file
rm -f ompl_testing_results.csv

echo "Launching testing environment..."
# Launch the testing environment in background
roslaunch xarm_planner planner_tester.launch robot_dof:=7 num_trials:=$NUM_TRIALS no_gui_plan:=true &

# Wait for launch to complete
echo "Waiting for launch to complete..."
sleep 15

# Check if planner_tester node is running
if ! pgrep -f "planner_tester" > /dev/null; then
    echo "ERROR: planner_tester node is not running!"
    echo "Checking ROS node list:"
    rosnode list
    exit 1
fi

echo "Planner tester node is running. Starting parameter tests..."
echo "This will run $NUM_TRIALS trials with different RRT* parameters."
echo "Results will be saved to: ompl_testing_results.csv"
echo ""

# Run the planner tester with parameter testing
echo "Executing parameter tests..."
rosrun xarm_planner planner_tester test

echo ""
echo "=========================================="
echo "Parameter testing completed!"
echo "=========================================="

# Check if results file was created
if [ -f "ompl_testing_results.csv" ]; then
    echo "Results saved to: ompl_testing_results.csv"
    echo ""
    echo "Results summary:"
    echo "Total lines (including header): $(wc -l < ompl_testing_results.csv)"
    echo "Successful trials: $(grep -c "true" ompl_testing_results.csv || echo "0")"
    echo "Failed trials: $(grep -c "false" ompl_testing_results.csv || echo "0")"
    
    # Show first few lines of results
    echo ""
    echo "First few results:"
    head -5 ompl_testing_results.csv
else
    echo "ERROR: Results file was not created!"
    exit 1
fi

echo ""
echo "=========================================="
echo "Testing complete! You can now analyze the results."
echo "=========================================="

# Stop the background processes
echo "Cleaning up..."
pkill -f "planner_tester"
pkill -f "move_group"
pkill -f "rviz"

echo "Cleanup complete." 