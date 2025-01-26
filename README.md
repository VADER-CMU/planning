# planning

Holds planning workspace for VADER.

Check out vader_planning/src/vader_helloworld.cpp for a very basic motion planner node using MoveIt, and vader_planning/launch/ for an example launch file (which you need to copy into the xarm submodule, for now) to launch rviz with a single xarm7 and have it move with the vader_helloworld stuff.

## Setup

Someone verify this.

Clone this repository first with 
```bash
git clone --recursive git@github.com:VADER-CMU/planning.git #should fetch all submodules? someone double check
```

Update the submodules(?)
```bash
git submodule update --init --remote
cd src
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd xarm/xarm_ros2
git submodule update --init --remote #may be overkill?

```

Build the packages. At the root level, do:
```bash
source /opt/ros/humble/setup.bash
# If you have a prev failed build, it may help to remove the build artifacts
rm -rf build
colcon build
source install/setup.bash
```

Run stuff under ros2. For more information regarding the xarm2 library submodule, see https://github.com/xArm-Developer/xarm_ros2/tree/humble.

## References and Citations

https://github.com/xArm-Developer/xarm_ros2/tree/humble (ROS2 Humble)