# Planner Tester - RRT* Parameter Testing System

## Overview

The Planner Tester is a comprehensive testing system designed to evaluate RRT* (Rapidly-exploring Random Tree Star) planner performance across different parameter combinations. It automatically tests 50 intelligent parameter combinations and logs detailed metrics for analysis.

## Features

- **RRT* Focused**: Specifically designed for testing RRT* planner parameters
- **Intelligent Sampling**: Uses Latin Hypercube sampling for optimal parameter space coverage
- **Comprehensive Metrics**: Collects planning time, path length, smoothness, and more
- **Automated Testing**: Runs 50 trials automatically with different parameter sets
- **Collision Environment**: Creates randomized collision objects around the robot arm
- **Fixed Goal Testing**: Uses a consistent goal pose amidst collision objects for fair comparison

## System Requirements

- ROS Noetic
- MoveIt
- Docker (for containerized execution)
- xArm robot simulation environment

## Architecture

### Components

1. **`planner_tester.cpp`**: Main C++ executable for parameter testing
2. **`planner_tester.launch`**: Launch file for testing environment setup
3. **`planner_tester_ompl.yaml`**: OMPL parameters configuration
4. **`run_planner_tester.sh`**: Automation script for Docker execution

### Key Classes

- **`PlannerTester`**: Main testing class (inherits from `XArmSimplePlanner`)
- **`RRTStarParams`**: Structure for RRT* parameters
- **`PlanningMetrics`**: Structure for collected metrics

## RRT* Parameters Tested

### Core Parameters
- **`range`**: Maximum distance for new node connections [0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
- **`goal_bias`**: Probability of sampling toward goal [0.01, 0.05, 0.1, 0.15, 0.2, 0.25]
- **`rewire_factor`**: Factor for rewiring radius [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
- **`max_nearest_neighbors`**: Maximum neighbors to consider [5, 10, 15, 20, 25, 30]

### Additional Parameters
- **`delay_collision_checking`**: Boolean for collision checking optimization
- **`use_k_nearest`**: Boolean for neighbor selection method
- **`max_states`**: Maximum states to store
- **`use_informed_sampling`**: Boolean for informed sampling
- **`sample_rejection_attempts`**: Number of sampling attempts
- **`use_rejection_sampling`**: Boolean for rejection sampling

## Metrics Collected

### Performance Metrics
- **`planning_time`**: Time taken to find solution (seconds)
- **`path_length`**: Total joint space path length (radians)
- **`path_smoothness`**: Measure of path smoothness [0,1]

### Quality Metrics
- **`success`**: Whether planning succeeded (boolean)
- **`solution_cost`**: Cost of found solution
- **`iterations`**: Number of iterations before solution
- **`tree_size`**: Final tree size

### Computational Metrics
- **`path_segments`**: Number of waypoints in path
- **`memory_usage`**: Memory consumption (MB)
- **`collision_checks`**: Number of collision detection calls
- **`sampling_attempts`**: Number of sampling attempts

## Usage

### Quick Start (Docker)

1. **Start Docker container**:
   ```bash
   ./run.sh
   ```

2. **Inside container, run the tester**:
   ```bash
   ./src/xarm_ros/xarm_planner/scripts/run_planner_tester.sh
   ```

3. **Results will be saved to**: `ompl_testing_results.csv`

### Manual Execution

1. **Build the project**:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

2. **Launch testing environment**:
   ```bash
   roslaunch xarm_planner planner_tester.launch robot_dof:=7
   ```

3. **Run parameter tests**:
   ```bash
   rosrun xarm_planner planner_tester test
   ```

### Customization

#### Modify Number of Trials
```bash
# In launch file
<param name="num_trials" value="100" />

# Or via command line
./run_planner_tester.sh 100
```

#### Modify Fixed Goal Pose
```bash
# In launch file
<arg name="fixed_goal_x" default="0.4" />
<arg name="fixed_goal_y" default="0.1" />
<arg name="fixed_goal_z" default="0.5" />
```

#### Modify Parameter Ranges
Edit `config/planner_tester_ompl.yaml` to change parameter values and ranges.

## Output Format

### CSV Structure
```csv
trial_id,timestamp,range,goal_bias,rewire_factor,max_nearest_neighbors,delay_collision_checking,use_k_nearest,max_states,use_informed_sampling,sample_rejection_attempts,use_rejection_sampling,planning_time,path_length,path_smoothness,success,solution_cost,iterations,tree_size,path_segments,memory_usage,collision_checks,sampling_attempts
```

### Example Output
```csv
1,2025-01-27_10-30-15,0.1,0.05,1.1,10,true,true,1000,true,100,false,2.45,1.23,0.85,true,1.23,1500,2500,15,45.2,5000,1200
```

## Analysis

### Post-Processing
The CSV output can be analyzed using:
- **Python pandas**: For statistical analysis
- **Excel/LibreOffice**: For visualization
- **R**: For advanced statistical analysis

### Key Insights to Look For
1. **Parameter Sensitivity**: Which parameters most affect planning time?
2. **Success Rate**: Which parameter combinations have highest success rates?
3. **Performance Trade-offs**: Relationship between planning time and path quality
4. **Optimal Ranges**: Best parameter values for your specific use case

## Troubleshooting

### Common Issues

1. **Planning Timeouts**:
   - Increase `planning_timeout` in launch file
   - Reduce collision object complexity

2. **Memory Issues**:
   - Reduce `max_states` parameter
   - Reduce number of collision objects

3. **Launch Failures**:
   - Check ROS master is running
   - Verify MoveIt configuration files

### Debug Mode
Enable verbose output by setting `output="screen"` in launch file and check ROS logs:
```bash
rosnode info /planner_tester
rostopic echo /move_group/status
```

## Performance Considerations

### Testing Time
- **50 trials**: ~2-4 hours (depending on parameter complexity)
- **Per trial**: ~2-5 minutes (planning + metrics collection)

### Resource Usage
- **Memory**: ~100-500 MB per trial
- **CPU**: High during planning, low during execution
- **Storage**: ~1-5 MB for results CSV

## Future Enhancements

### Planned Features
1. **Adaptive Sampling**: Focus on promising parameter regions
2. **Multi-Objective Optimization**: Balance planning time vs. path quality
3. **Statistical Analysis**: Built-in significance testing
4. **Parameter Interpolation**: Estimate performance for untested combinations

### Extensibility
The system is designed to be easily extended for:
- Other OMPL planners (PRM, FMT, etc.)
- Different robot configurations
- Custom metrics and evaluation criteria

## Contributing

To extend the planner tester:
1. Add new parameters to `RRTStarParams` structure
2. Implement parameter setting in `setRRTStarParameters()`
3. Add new metrics to `PlanningMetrics` structure
4. Update CSV logging in `logResults()`

## License

This project follows the same license as the xArm ROS package (BSD License).

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS logs and error messages
3. Verify MoveIt and OMPL configurations
4. Check parameter value ranges and constraints 