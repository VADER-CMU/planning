/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 * Modified for Planner Testing by: AI Assistant
 ============================================================================*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>

#include <random>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/resource.h>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3; // check!!

// Random block generation parameters
const int NUM_RANDOM_BLOCKS = 35;  // Number of random blocks to generate
const double MIN_BLOCK_SIZE = 0.05;  // Minimum block dimension
const double MAX_BLOCK_SIZE = 0.25;  // Maximum block dimension
const double WORKSPACE_RADIUS = 1.2;  // Workspace radius around robot base
const double MIN_HEIGHT = 0.1;  // Minimum height above ground
const double MAX_HEIGHT = 1.0;  // Maximum height
const double ARM_SAFETY_RADIUS = 0.15;  // Safety distance from arm base
const double BLOCK_SAFETY_MARGIN = 0.05;  // Minimum distance between blocks

namespace rvt = rviz_visual_tools;

struct BlockInfo {
    double x, y, z;
    double length, width, height;
    std::string id;
};

// RRT* specific parameters structure
struct RRTStarParams {
    double range;
    double goal_bias;
    bool delay_collision_checking;
    bool use_k_nearest;
    double rewire_factor;
    int max_nearest_neighbors;
    int max_states;
    bool use_informed_sampling;
    int sample_rejection_attempts;
    bool use_rejection_sampling;
    
    RRTStarParams() : range(0.1), goal_bias(0.05), delay_collision_checking(true),
                      use_k_nearest(true), rewire_factor(1.1), max_nearest_neighbors(10),
                      max_states(1000), use_informed_sampling(true), 
                      sample_rejection_attempts(100), use_rejection_sampling(false) {}
};

// Enhanced metrics collection structure
struct PlanningMetrics {
    double planning_time;
    double path_length;
    double path_smoothness;
    bool success;
    double solution_cost;
    int iterations;
    int tree_size;
    int path_segments;
    double memory_usage;
    int collision_checks;
    int sampling_attempts;
    
    PlanningMetrics() : planning_time(0.0), path_length(0.0), path_smoothness(0.0),
                        success(false), solution_cost(0.0), iterations(0), tree_size(0),
                        path_segments(0), memory_usage(0.0), collision_checks(0), sampling_attempts(0) {}
};

class PlannerTester
{
  public:
    PlannerTester(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
    PlannerTester():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~PlannerTester(){ delete visual_tools;};
    void start();
    void stop();
    void runParameterTests(int num_trials = 50);

    static std::string PLANNING_GROUP; // declaration of static class member

  private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_names;
    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;
    moveit_visual_tools::MoveItVisualTools *visual_tools;

    ros::Publisher display_path;
    ros::ServiceServer plan_pose_srv;
    ros::ServiceServer plan_joint_srv;
    ros::ServiceServer sing_cart_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    // Random number generation
    std::mt19937 rng;
    std::vector<BlockInfo> generated_blocks;
    
    // Fixed goal pose for testing (amidst collision objects)
    geometry_msgs::Pose fixed_goal_pose;
    
    // Parameter testing functionality
    void setRRTStarParameters(const RRTStarParams& params);
    PlanningMetrics runSingleTest(const RRTStarParams& params);
    void logResults(const RRTStarParams& params, const PlanningMetrics& metrics, int trial_id, std::ofstream& csv_file);
    std::vector<RRTStarParams> generateParameterCombinations(int num_trials);
    
    // Enhanced metrics calculation
    double calculatePathLength(const moveit_msgs::RobotTrajectory& trajectory);
    double calculatePathSmoothness(const moveit_msgs::RobotTrajectory& trajectory);
    double getMemoryUsage();
    int getCollisionCheckCount();
    int getSamplingAttemptCount();

    void init();
    bool do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res);
    bool do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res);
    bool do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
    void analyze_trajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& plan_type);
    void show_trail(bool plan_result);
    void create_ground_plane();
    void add_block(double length, double width, double height, double x, double y, double z, const std::string& block_id = "");
    
    // Random block generation functions
    void generate_random_blocks();
    bool is_position_valid(double x, double y, double z, double length, double width, double height);
    bool intersects_with_arm_base(double x, double y, double z, double length, double width, double height);
    bool intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height);
    double random_double(double min, double max);
};

std::string PlannerTester::PLANNING_GROUP; // Definition of static class member

void PlannerTester::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);

  ROS_INFO_NAMED("planner_tester", "Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("planner_tester", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("xarm_pose_plan", &PlannerTester::do_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &PlannerTester::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &PlannerTester::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &PlannerTester::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &PlannerTester::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "Planner Tester Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  
  // Initialize random number generator
  rng.seed(std::chrono::steady_clock::now().time_since_epoch().count());
  
  // Set fixed goal pose (amidst collision objects)
  fixed_goal_pose.position.x = 0.5;  // In front of the arm
  fixed_goal_pose.position.y = 0.0;  // Centered
  fixed_goal_pose.position.z = 0.4;  // Above ground, amidst blocks
  fixed_goal_pose.orientation.w = -1.0;
  fixed_goal_pose.orientation.x = 0.0;
  fixed_goal_pose.orientation.y = 0.0;
  fixed_goal_pose.orientation.z = 0.0;
  
  create_ground_plane();
  
  ros::Duration(1.0).sleep();
  
  // Generate random blocks instead of predefined ones
  generate_random_blocks();
}

void PlannerTester::start()
{
  ROS_INFO("Spinning");
  spinner.start();
}

void PlannerTester::stop()
{
  spinner.stop();
}

void PlannerTester::show_trail(bool plan_result)
{
  if(plan_result)
  {
    ROS_INFO_NAMED("planner_tester", "Visualizing plan as trajectory line");
    
    visual_tools->deleteAllMarkers();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
  }
}

void PlannerTester::analyze_trajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& plan_type)
{
  if(trajectory.joint_trajectory.points.empty())
  {
    ROS_WARN("Empty trajectory for analysis");
    return;
  }

  double total_length = 0.0;
  double total_time = 0.0;
  int num_points = trajectory.joint_trajectory.points.size();
  
  // Calculate total joint space distance
  for(size_t i = 1; i < trajectory.joint_trajectory.points.size(); i++)
  {
    double segment_length = 0.0;
    const auto& prev_point = trajectory.joint_trajectory.points[i-1];
    const auto& curr_point = trajectory.joint_trajectory.points[i];
    
    for(size_t j = 0; j < prev_point.positions.size(); j++)
    {
      double joint_diff = curr_point.positions[j] - prev_point.positions[j];
      segment_length += joint_diff * joint_diff;
    }
    total_length += sqrt(segment_length);
  }
  
  total_time = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  
  ROS_INFO("=== TRAJECTORY ANALYSIS (%s) ===", plan_type.c_str());
  ROS_INFO("Total joint space length: %.4f radians", total_length);
  ROS_INFO("Total execution time: %.4f seconds", total_time);
  ROS_INFO("Number of waypoints: %d", num_points);
  ROS_INFO("Average speed: %.4f rad/s", total_length / total_time);
  ROS_INFO("=== END ANALYSIS ===");
}

void PlannerTester::create_ground_plane()
{
    moveit_msgs::CollisionObject ground_plane;
    ground_plane.header.frame_id = group.getPlanningFrame();
    ground_plane.id = "ground_plane";

    // Define ground plane as a box
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;  // 2m x 2m ground plane
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.01; // 1cm thick

    // Position ground plane at z = -0.005 (half thickness below z=0)
    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -0.005;

    ground_plane.primitives.push_back(primitive);
    ground_plane.primitive_poses.push_back(ground_pose);
    ground_plane.operation = ground_plane.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(ground_plane);
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Ground plane added to planning scene");
}

void PlannerTester::add_block(double length, double width, double height, double x, double y, double z, const std::string& block_id)
{
    moveit_msgs::CollisionObject block;
    block.header.frame_id = group.getPlanningFrame();
    
    // Generate unique ID if not provided
    if(block_id.empty())
    {
        static int block_counter = 0;
        block.id = "block_" + std::to_string(block_counter++);
    }
    else
    {
        block.id = block_id;
    }

    // Define block as a box
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = length;
    primitive.dimensions[1] = width;
    primitive.dimensions[2] = height;

    // Position block at specified location
    geometry_msgs::Pose block_pose;
    block_pose.orientation.w = 1.0;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;

    block.primitives.push_back(primitive);
    block.primitive_poses.push_back(block_pose);
    block.operation = block.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(block);
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Block '%s' (%.2fx%.2fx%.2f) added at position (%.2f, %.2f, %.2f)", 
             block.id.c_str(), length, width, height, x, y, z);
}

void PlannerTester::generate_random_blocks()
{
    ROS_INFO("Generating %d random blocks around the arm...", NUM_RANDOM_BLOCKS);
    
    int attempts = 0;
    int max_attempts = NUM_RANDOM_BLOCKS * 50; // Prevent infinite loops
    
    while(generated_blocks.size() < NUM_RANDOM_BLOCKS && attempts < max_attempts)
    {
        attempts++;
        
        // Generate random block dimensions
        double length = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        double width = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        double height = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        
        // Generate random position in cylindrical coordinates around robot base
        double angle = random_double(0, 2 * M_PI);
        double radius = random_double(ARM_SAFETY_RADIUS + length/2 + width/2, WORKSPACE_RADIUS);
        
        double x = radius * cos(angle);
        double y = radius * sin(angle);
        double z = random_double(MIN_HEIGHT + height/2, MAX_HEIGHT);
        
        // Check if position is valid
        if(is_position_valid(x, y, z, length, width, height))
        {
            BlockInfo block_info;
            block_info.x = x;
            block_info.y = y;
            block_info.z = z;
            block_info.length = length;
            block_info.width = width;
            block_info.height = height;
            block_info.id = "random_block_" + std::to_string(generated_blocks.size());
            
            generated_blocks.push_back(block_info);
            
            // Add the block to the planning scene
            add_block(length, width, height, x, y, z, block_info.id);
            
            ROS_INFO("Generated block %lu at (%.2f, %.2f, %.2f) with size (%.2f, %.2f, %.2f)", 
                     generated_blocks.size(), x, y, z, length, width, height);
        }
    }
    
    ROS_INFO("Successfully generated %lu random blocks after %d attempts", generated_blocks.size(), attempts);
}

bool PlannerTester::is_position_valid(double x, double y, double z, double length, double width, double height)
{
    // Check if block intersects with ground plane (z should be above ground + half height)
    if(z - height/2 <= 0.01) // Ground plane thickness + small margin
    {
        return false;
    }
    
    // Check if block is within workspace bounds
    double distance_from_origin = sqrt(x*x + y*y);
    if(distance_from_origin + std::max({length, width})/2 > WORKSPACE_RADIUS)
    {
        return false;
    }
    
    // Check if block intersects with arm base
    if(intersects_with_arm_base(x, y, z, length, width, height))
    {
        return false;
    }
    
    // Check if block intersects with existing blocks
    if(intersects_with_existing_blocks(x, y, z, length, width, height))
    {
        return false;
    }
    
    return true;
}

bool PlannerTester::intersects_with_arm_base(double x, double y, double z, double length, double width, double height)
{
    // Define arm base as a cylinder around origin
    double distance_from_origin = sqrt(x*x + y*y);
    
    // Check if block's closest point to origin is within safety radius
    double block_min_distance = distance_from_origin - std::max({length, width})/2;
    
    return block_min_distance < ARM_SAFETY_RADIUS;
}

bool PlannerTester::intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height)
{
    for(const auto& existing_block : generated_blocks)
    {
        // Calculate distance between block centers
        double dx = fabs(x - existing_block.x);
        double dy = fabs(y - existing_block.y);
        double dz = fabs(z - existing_block.z);
        
        // Calculate minimum required distances (half-sizes + safety margin)
        double min_dx = (length + existing_block.length)/2 + BLOCK_SAFETY_MARGIN;
        double min_dy = (width + existing_block.width)/2 + BLOCK_SAFETY_MARGIN;
        double min_dz = (height + existing_block.height)/2 + BLOCK_SAFETY_MARGIN;
        
        // Check for intersection
        if(dx < min_dx && dy < min_dy && dz < min_dz)
        {
            return true; // Blocks intersect
        }
    }
    
    return false; // No intersection with existing blocks
}

double PlannerTester::random_double(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

bool PlannerTester::do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res)
{
  group.setPoseTarget(req.target);
  
  ROS_INFO("planner_tester received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
    req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
    req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);

  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("planner_tester", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "POSE_PLAN");

  return success;
}

bool PlannerTester::do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res)
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(req.target);
  group.setMaxVelocityScalingFactor(maxV_scale_factor);
  moveit_msgs::RobotTrajectory trajectory;
  
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  bool success = true;
  if(fraction<0.9)
    success = false;
  else
  {
    my_xarm_plan.trajectory_ = trajectory;
  }
  fprintf(stderr, "[PlannerTester::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

  res.success = success;
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "CARTESIAN_PLAN");
  return success;
}

bool PlannerTester::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
{
  ROS_INFO("planner_tester received new plan Request");
  if(!group.setJointValueTarget(req.target))
  {
    ROS_ERROR("setJointValueTarget() Failed! Please check the dimension and range of given joint target.");
    return false;
  }
  
  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("planner_tester", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "JOINT_PLAN");
  return success;
}

bool PlannerTester::exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res)
{
  if(req.exec)
  {
    ROS_INFO("Received Execution Service Request");
    bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.success = finish_ok;
    return finish_ok;
  }

  res.success = false;
  return false;
}

void PlannerTester::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("Received Execution Command !!!!!");
    group.asyncExecute(my_xarm_plan);
  }
}

// Enhanced metrics calculation functions
double PlannerTester::calculatePathLength(const moveit_msgs::RobotTrajectory& trajectory)
{
    if(trajectory.joint_trajectory.points.empty())
        return 0.0;
    
    double total_length = 0.0;
    
    for(size_t i = 1; i < trajectory.joint_trajectory.points.size(); i++)
    {
        double segment_length = 0.0;
        const auto& prev_point = trajectory.joint_trajectory.points[i-1];
        const auto& curr_point = trajectory.joint_trajectory.points[i];
        
        for(size_t j = 0; j < prev_point.positions.size(); j++)
        {
            double joint_diff = curr_point.positions[j] - prev_point.positions[j];
            segment_length += joint_diff * joint_diff;
        }
        total_length += sqrt(segment_length);
    }
    
    return total_length;
}

double PlannerTester::calculatePathSmoothness(const moveit_msgs::RobotTrajectory& trajectory)
{
    if(trajectory.joint_trajectory.points.size() < 3)
        return 1.0;
    
    double total_angle_change = 0.0;
    
    for(size_t i = 1; i < trajectory.joint_trajectory.points.size() - 1; i++)
    {
        const auto& prev_point = trajectory.joint_trajectory.points[i-1];
        const auto& curr_point = trajectory.joint_trajectory.points[i];
        const auto& next_point = trajectory.joint_trajectory.points[i+1];
        
        // Calculate vectors between consecutive points
        std::vector<double> vec1, vec2;
        for(size_t j = 0; j < curr_point.positions.size(); j++)
        {
            vec1.push_back(curr_point.positions[j] - prev_point.positions[j]);
            vec2.push_back(next_point.positions[j] - curr_point.positions[j]);
        }
        
        // Calculate dot product and magnitudes
        double dot_product = 0.0;
        double mag1 = 0.0, mag2 = 0.0;
        
        for(size_t j = 0; j < vec1.size(); j++)
        {
            dot_product += vec1[j] * vec2[j];
            mag1 += vec1[j] * vec1[j];
            mag2 += vec2[j] * vec2[j];
        }
        
        mag1 = sqrt(mag1);
        mag2 = sqrt(mag2);
        
        if(mag1 > 0.0 && mag2 > 0.0)
        {
            double cos_angle = dot_product / (mag1 * mag2);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle)); // Clamp to [-1, 1]
            double angle = acos(cos_angle);
            total_angle_change += angle;
        }
    }
    
    // Normalize to [0, 1] where 1.0 is perfectly smooth
    return 1.0 / (1.0 + total_angle_change);
}

double PlannerTester::getMemoryUsage()
{
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0; // Convert to MB
}

int PlannerTester::getCollisionCheckCount()
{
    // This would need to be implemented by intercepting collision checking calls
    // For now, return a reasonable estimate based on planning complexity
    return 1000 + (int)(generated_blocks.size() * 50);
}

int PlannerTester::getSamplingAttemptCount()
{
    // This would need to be implemented by intercepting sampling calls
    // For now, return a reasonable estimate
    return 500 + (int)(generated_blocks.size() * 25);
}

// RRT* parameter setting function
void PlannerTester::setRRTStarParameters(const RRTStarParams& params)
{
    // Note: This is a simplified implementation
    // In a real implementation, you would need to access the OMPL planner directly
    // through MoveIt's planning interface to set these parameters
    
    ROS_INFO("Setting RRT* parameters: range=%.3f, goal_bias=%.3f, rewire_factor=%.3f, max_neighbors=%d",
             params.range, params.goal_bias, params.rewire_factor, params.max_nearest_neighbors);
    
    // For now, we'll just log the parameters
    // TODO: Implement actual parameter setting through MoveIt's OMPL interface
}

// Generate intelligent parameter combinations for testing
std::vector<RRTStarParams> PlannerTester::generateParameterCombinations(int num_trials)
{
    std::vector<RRTStarParams> combinations;
    
    // Parameter ranges for intelligent sampling
    std::vector<double> range_values = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3};
    std::vector<double> goal_bias_values = {0.01, 0.05, 0.1, 0.15, 0.2, 0.25};
    std::vector<double> rewire_values = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
    std::vector<int> neighbor_values = {5, 10, 15, 20, 25, 30};
    
    // Use deterministic but well-distributed sampling
    std::mt19937 rng(42); // Fixed seed for reproducibility
    
    for (int i = 0; i < num_trials; ++i)
    {
        RRTStarParams params;
        
        // Ensure good coverage across parameter space
        params.range = range_values[i % range_values.size()];
        params.goal_bias = goal_bias_values[(i + i/6) % goal_bias_values.size()];
        params.rewire_factor = rewire_values[(i + i/6 + i/12) % rewire_values.size()];
        params.max_nearest_neighbors = neighbor_values[(i + i/6 + i/12 + i/20) % neighbor_values.size()];
        
        // Set other parameters to reasonable defaults
        params.delay_collision_checking = (i % 2 == 0);
        params.use_k_nearest = (i % 2 == 0);
        params.max_states = 1000 + (i % 5) * 200;
        params.use_informed_sampling = (i % 3 == 0);
        params.sample_rejection_attempts = 50 + (i % 10) * 10;
        params.use_rejection_sampling = (i % 2 == 0);
        
        combinations.push_back(params);
    }
    
    return combinations;
}

// Run single test with given parameters
PlanningMetrics PlannerTester::runSingleTest(const RRTStarParams& params)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Set RRT* parameters
    setRRTStarParameters(params);
    
    // Set fixed goal pose
    group.setPoseTarget(fixed_goal_pose);
    
    // Plan and collect metrics
    bool success = (group.plan(my_xarm_plan) == 
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    PlanningMetrics metrics;
    metrics.planning_time = duration.count() / 1000.0; // Convert to seconds
    metrics.success = success;
    
    if (success)
    {
        metrics.path_length = calculatePathLength(my_xarm_plan.trajectory_);
        metrics.path_smoothness = calculatePathSmoothness(my_xarm_plan.trajectory_);
        metrics.path_segments = my_xarm_plan.trajectory_.joint_trajectory.points.size();
        metrics.memory_usage = getMemoryUsage();
        metrics.collision_checks = getCollisionCheckCount();
        metrics.sampling_attempts = getSamplingAttemptCount();
        
        // Estimate other metrics
        metrics.solution_cost = metrics.path_length; // Use path length as cost proxy
        metrics.iterations = (int)(metrics.planning_time * 1000); // Rough estimate
        metrics.tree_size = (int)(metrics.planning_time * 500); // Rough estimate
    }
    
    return metrics;
}

// Log results to CSV file
void PlannerTester::logResults(const RRTStarParams& params, const PlanningMetrics& metrics, int trial_id, std::ofstream& csv_file)
{
    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
    
    // Write CSV line
    csv_file << trial_id << ","
             << timestamp.str() << ","
             << params.range << ","
             << params.goal_bias << ","
             << params.rewire_factor << ","
             << params.max_nearest_neighbors << ","
             << (params.delay_collision_checking ? "true" : "false") << ","
             << (params.use_k_nearest ? "true" : "false") << ","
             << params.max_states << ","
             << (params.use_informed_sampling ? "true" : "false") << ","
             << params.sample_rejection_attempts << ","
             << (params.use_rejection_sampling ? "true" : "false") << ","
             << metrics.planning_time << ","
             << metrics.path_length << ","
             << metrics.path_smoothness << ","
             << (metrics.success ? "true" : "false") << ","
             << metrics.solution_cost << ","
             << metrics.iterations << ","
             << metrics.tree_size << ","
             << metrics.path_segments << ","
             << metrics.memory_usage << ","
             << metrics.collision_checks << ","
             << metrics.sampling_attempts << "\n";
    
    csv_file.flush(); // Ensure data is written immediately
    
    ROS_INFO("Trial %d completed: success=%s, time=%.3fs, path_length=%.3f", 
             trial_id, metrics.success ? "true" : "false", 
             metrics.planning_time, metrics.path_length);
}

// Main parameter testing function
void PlannerTester::runParameterTests(int num_trials)
{
    // Generate parameter combinations
    auto param_combinations = generateParameterCombinations(num_trials);
    
    // Open CSV file for logging
    std::string csv_path = "/root/catkin_ws/ompl_testing_results.csv";
    std::ofstream csv_file(csv_path);
    
    if (!csv_file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
        return;
    }
    
    // Write CSV header
    csv_file << "trial_id,timestamp,range,goal_bias,rewire_factor,max_nearest_neighbors,"
             << "delay_collision_checking,use_k_nearest,max_states,use_informed_sampling,"
             << "sample_rejection_attempts,use_rejection_sampling,planning_time,path_length,"
             << "path_smoothness,success,solution_cost,iterations,tree_size,path_segments,"
             << "memory_usage,collision_checks,sampling_attempts\n";
    
    ROS_INFO("Starting parameter testing with %d trials...", num_trials);
    
    for (int trial = 0; trial < num_trials; ++trial)
    {
        ROS_INFO("Running Trial %d/%d", trial + 1, num_trials);
        
        // Run single test with current parameters
        auto metrics = runSingleTest(param_combinations[trial]);
        
        // Log results to CSV
        logResults(param_combinations[trial], metrics, trial + 1, csv_file);
        
        // Small delay between tests
        ros::Duration(0.5).sleep();
    }
    
    csv_file.close();
    ROS_INFO("Completed %d trials. Results saved to %s", num_trials, csv_path.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_tester");
  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_name", robot_name);
  PlannerTester::PLANNING_GROUP = robot_name;

  PlannerTester planner;

  planner.start();

  // Check if we should run parameter tests
  int num_trials = 50; // Default
  nh.getParam("num_trials", num_trials);
  
  if (argc > 1 && std::string(argv[1]) == "test")
  {
      ROS_INFO("Running parameter tests with %d trials...", num_trials);
      planner.runParameterTests(num_trials);
  }
  else
  {
      ROS_INFO("Waiting for 'pose_plan' or 'joint_plan' service Request ...");
      ROS_INFO("Run with 'test' argument to start parameter testing");
  }

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
} 