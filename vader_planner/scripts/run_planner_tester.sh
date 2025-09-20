# #!/bin/bash


# set -e

# NUM_TRIALS=${1:-50}


# echo "Planner Testing"
# echo "Number of trials: $NUM_TRIALS"


# cd /root/catkin_ws

# echo "Building project..."

# catkin_make

# source devel/setup.bash


# rm -f ompl_testing_results.csv

# echo "Launching testing environment..."

# roslaunch xarm_planner planner_tester.launch robot_dof:=7 num_trials:=$NUM_TRIALS no_gui_plan:=true &

# sleep 15


# if ! pgrep -f "planner_tester" > /dev/null; then
#     echo "ERROR: planner_tester node is not running!"
#     echo "Checking ROS node list:"
#     rosnode list
#     exit 1
# fi

# echo "Planner tester node is running."

# rosrun xarm_planner planner_tester test


# echo "Parameter testing completed!"


# if [ -f "ompl_testing_results.csv" ]; then
#     echo "Results saved to: ompl_testing_results.csv"
#     echo "Results summary:"
#     echo "Total lines (including header): $(wc -l < ompl_testing_results.csv)"
#     echo "Successful trials: $(grep -c "true" ompl_testing_results.csv || echo "0")"
#     echo "Failed trials: $(grep -c "false" ompl_testing_results.csv || echo "0")"
    
   
# else
#     echo "ERROR: Results file was not created!"
#     exit 1
# fi


# echo "Testing complete!"

# pkill -f "planner_tester"
# pkill -f "move_group"
# pkill -f "rviz"

# echo "Buh Byeee" 

#!/bin/bash

set -e

NUM_TRIALS=${1:-50}

echo "Planner Testing"
echo "Number of trials: $NUM_TRIALS"

# Get the script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR" && while [[ "$PWD" != "/" && ! -f "src/CMakeLists.txt" ]]; do cd ..; done; pwd)"

if [[ ! -f "$WORKSPACE_ROOT/src/CMakeLists.txt" ]]; then
    echo "ERROR: Could not find catkin workspace root. Please run this script from within a catkin workspace."
    exit 1
fi

echo "Using workspace: $WORKSPACE_ROOT"
cd "$WORKSPACE_ROOT"

sudo apt-get install ros-noetic-moveit-servo


echo "Building project..."
catkin_make

source devel/setup.bash

# Clean up any existing results file
rm -f wall_planner_testing_results.csv

echo "Launching testing environment..."

# Launch with dynamic package detection
PACKAGE_NAME="vader_planner"
if ! rospack find "$PACKAGE_NAME" >/dev/null 2>&1; then
    echo "ERROR: Package $PACKAGE_NAME not found. Make sure it's built and sourced."
    exit 1
fi

roslaunch "$PACKAGE_NAME" planner_tester.launch robot_dof:=7 num_trials:=$NUM_TRIALS no_gui_plan:=true &
LAUNCH_PID=$!

echo "Waiting for nodes to initialize..."
sleep 15

# Check if launch is still running
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "ERROR: Launch process died unexpectedly!"
    exit 1
fi

# Check if planner_tester node is running
if ! pgrep -f "planner_tester" > /dev/null; then
    echo "ERROR: planner_tester node is not running!"
    echo "Checking ROS node list:"
    rosnode list
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo "Planner tester node is running."

# Run the test
echo "Starting parameter testing..."
rosrun "$PACKAGE_NAME" wall_planner_tester test

echo "Parameter testing completed!"

# Find results file - check both workspace root and package directory
RESULTS_FILE="/home/docker_ws/wall_planner_testing_results.csv"

if [ -f "$RESULTS_FILE" ]; then
    echo "Results saved to: $RESULTS_FILE"
    echo "Results summary:"
    echo "Total lines (including header): $(wc -l < "$RESULTS_FILE")"
    
    # Count successes and failures more robustly
    SUCCESS_COUNT=$(tail -n +2 "$RESULTS_FILE" | grep -c ",true$" || echo "0")
    FAILURE_COUNT=$(tail -n +2 "$RESULTS_FILE" | grep -c ",false$" || echo "0")
    
    echo "Successful trials: $SUCCESS_COUNT"
    echo "Failed trials: $FAILURE_COUNT"
    
    # Calculate success rate
    TOTAL_TRIALS=$((SUCCESS_COUNT + FAILURE_COUNT))
    if [ $TOTAL_TRIALS -gt 0 ]; then
        SUCCESS_RATE=$(echo "scale=2; $SUCCESS_COUNT * 100 / $TOTAL_TRIALS" | bc -l 2>/dev/null || echo "N/A")
        echo "Success rate: $SUCCESS_RATE%"
    fi
else
    echo "ERROR: Results file was not created!"
    echo "Expected location: $RESULTS_FILE"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo "Testing complete!"

# Clean shutdown
echo "Shutting down nodes..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Force kill any remaining processes
pkill -f "wall_planner_tester\|planner_tester" 2>/dev/null || true
pkill -f "move_group" 2>/dev/null || true
pkill -f "rviz" 2>/dev/null || true

echo "All done!"