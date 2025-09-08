#!/bin/bash


set -e

NUM_TRIALS=${1:-50}


echo "Planner Testing"
echo "Number of trials: $NUM_TRIALS"


cd /root/catkin_ws

echo "Building project..."

catkin_make

source devel/setup.bash


rm -f ompl_testing_results.csv

echo "Launching testing environment..."

roslaunch xarm_planner planner_tester.launch robot_dof:=7 num_trials:=$NUM_TRIALS no_gui_plan:=true &

sleep 15


if ! pgrep -f "planner_tester" > /dev/null; then
    echo "ERROR: planner_tester node is not running!"
    echo "Checking ROS node list:"
    rosnode list
    exit 1
fi

echo "Planner tester node is running."

rosrun xarm_planner planner_tester test


echo "Parameter testing completed!"


if [ -f "ompl_testing_results.csv" ]; then
    echo "Results saved to: ompl_testing_results.csv"
    echo "Results summary:"
    echo "Total lines (including header): $(wc -l < ompl_testing_results.csv)"
    echo "Successful trials: $(grep -c "true" ompl_testing_results.csv || echo "0")"
    echo "Failed trials: $(grep -c "false" ompl_testing_results.csv || echo "0")"
    
   
else
    echo "ERROR: Results file was not created!"
    exit 1
fi


echo "Testing complete!"

pkill -f "planner_tester"
pkill -f "move_group"
pkill -f "rviz"

echo "Buh Byeee" 