# Package Introduction


For OMPL - RRT* params, refer to these links: [Link1](https://ompl.kavrakilab.org/RRTstar_8h_source.html) [Link2](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTstar.html) 
# Planner Tester Overview
To run the planner tester directly, use ./src/xarm_ros/vader_planner/scripts/run_planner_tester.sh
## What it does
The planner tester is a ROS-based tool that evaluates robot arm motion planning algorithms by testing them in challenging obstacle environments.

## Key Components

### Environment Setup
- **Ground plane**: Flat surface to prevent arm from going below ground
- **Strategic obstacles**: Small blocks placed between robot and goal
- **Fixed goal**: Target position for the arm
- **Deterministic seed**: Same obstacle layout every run for consistent testing

### Block Placement Strategy
- Blocks positioned along path from robot base to goal
- An empty corridor around the arm radius maintained for guaranteed reachability
- Blocks placed perpendicular to direct path (left/right sides)
- Forces planner to navigate around obstacles while keeping goal reachable

### Planning Services
1. **Pose Planning** (`xarm_pose_plan`): Move end-effector to target position/orientation
2. **Joint Planning** (`xarm_joint_plan`): Move joints to specific angles
3. **Cartesian Planning** (`xarm_straight_plan`): Move in straight line to target

### Testing Modes
- **Interactive Mode**: Responds to service requests for individual planning tasks
- **Automated Testing**: Tests multiple RRT* parameter combinations (50 trials)
  - Varies: range, goal bias, rewire factor, max neighbors, collision checking, etc.
  - Outputs results to CSV file with timing and success metrics


## Usage
```bash
# Interactive mode
rosrun xarm_planner planner_tester

# Automated parameter testing
rosrun xarm_planner planner_tester test
```

Results saved to `ompl_testing_results.csv` for analysis.








