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
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <unistd.h>
#include <ros/package.h>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3; // check!!

const int NUM_RANDOM_BLOCKS = 12;
const double MIN_BLOCK_SIZE = 0.02;
const double MAX_BLOCK_SIZE = 0.04;
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


struct PlanningMetrics {
    double planning_time;
    double path_length;
    bool success;
    
    PlanningMetrics() : planning_time(0.0), path_length(0.0), success(false) {}
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
    ros::Subscriber exec_plan_sub; 
    ros::ServiceServer exec_plan_srv; 


    std::mt19937 rng;
    std::vector<BlockInfo> generated_blocks;
    
    
    geometry_msgs::Pose fixed_goal_pose;
    
    std::string yaml_config_path;
    
    void setRRTStarParameters(const RRTStarParams& params);
    PlanningMetrics runSingleTest(const RRTStarParams& params);
    void logResults(const RRTStarParams& params, const PlanningMetrics& metrics, int trial_id, std::ofstream& csv_file);
    std::vector<RRTStarParams> generateParameterCombinations(int num_trials);
    void updateYAMLConfig(const RRTStarParams& params);
    bool loadYAMLConfig();
    
    
    double calculatePathLength(const moveit_msgs::RobotTrajectory& trajectory);

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
    

    void generate_random_blocks();
    bool is_position_valid(double x, double y, double z, double length, double width, double height);
    bool intersects_with_arm_base(double x, double y, double z, double length, double width, double height);
    bool intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height);
    double random_double(double min, double max);
};

std::string PlannerTester::PLANNING_GROUP; 

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
  
  rng.seed(42);
  
  fixed_goal_pose.position.x = 0.43;
  fixed_goal_pose.position.y = 0.0;
  fixed_goal_pose.position.z = 0.38;
  fixed_goal_pose.orientation.w = 0.0;
  fixed_goal_pose.orientation.x = 0.0;
  fixed_goal_pose.orientation.y = 1.0;
  fixed_goal_pose.orientation.z = 0.0;
  
  ros::NodeHandle pnh("~");
  std::string config_relative_path = "config/planner_tester_ompl.yaml";

  if (!pnh.getParam("yaml_config_path", yaml_config_path)) {
    // Try xarm_planner first, then fallback to vader_planner
    yaml_config_path = ros::package::getPath("xarm_planner") + "/" + config_relative_path;
    
    std::ifstream test_file(yaml_config_path);
    if (!test_file.good()) {
      yaml_config_path = ros::package::getPath("vader_planner") + "/" + config_relative_path;
    }
  }

  ROS_INFO("Attempting to load YAML config from: %s", yaml_config_path.c_str());
  
  if (!loadYAMLConfig()) {
    ROS_WARN("Failed to load YAML config, using default parameters");
  }
  
  create_ground_plane();
  
  ros::Duration(1.0).sleep();
  
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

  
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;  
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.01; 

  
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
    

    if(block_id.empty())
    {
        static int block_counter = 0;
        block.id = "block_" + std::to_string(block_counter++);
    }
    else
    {
        block.id = block_id;
    }

    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = length;
    primitive.dimensions[1] = width;
    primitive.dimensions[2] = height;

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
    // Previous random block generation code commented out
    /*
    double goal_x = fixed_goal_pose.position.x;
    double goal_y = fixed_goal_pose.position.y;
    double goal_z = fixed_goal_pose.position.z;
    double arm_x = 0.0;
    double arm_y = 0.0;
    double arm_z = 0.0;
    double center_x = 0.5 * (arm_x + goal_x);
    double center_y = 0.0;
    double center_z = 0.5 * (arm_z + goal_z);
    double cube_size = 0.12;
    double square_size = 0.6;
    double gap = (square_size - 3 * cube_size) / 2.0;
    double plane_offset = center_x + cube_size * 1.2;
    // Arrange cubes in a 4x4 grid, skip the center 2x2 for a larger hole, so cubes are connected
    int idx = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row >= 1 && row <= 2 && col >= 1 && col <= 2) continue; // leave center 2x2 empty
            double y = center_y + (col - 1.5) * (cube_size + gap/2.0);
            double z = center_z + (row - 1.5) * (cube_size + gap/2.0);
            double x = plane_offset;
            BlockInfo block_info;
            block_info.x = x;
            block_info.y = y;
            block_info.z = z;
            block_info.length = cube_size;
            block_info.width = cube_size;
            block_info.height = cube_size;
            block_info.id = "vertical_square_cube_" + std::to_string(idx);
            generated_blocks.push_back(block_info);
            add_block(cube_size, cube_size, cube_size, x, y, z, block_info.id);
            idx++;
        }
    }
    ROS_INFO("Generated %lu cubes in vertical square with central hole (connected)", generated_blocks.size());
    */

    // Create vertical wall with hole in center
    double wall_x = 0.4;  // Wall position in front of arm
    double wall_thickness = 0.05;
    double wall_height = 0.8;
    double wall_width = 0.8;
    double hole_width = 0.3;
    double hole_height = 0.3;
    double wall_center_y = 0.0;
    double wall_center_z = 0.4;

    // Create wall segments around the hole
    double side_width = (wall_width - hole_width) / 2;
    
    // Bottom segment
    add_block(wall_thickness, wall_width, (wall_center_z - hole_height/2), 
              wall_x, wall_center_y, (wall_center_z - hole_height/2)/2, "wall_bottom");
    
    // Top segment  
    add_block(wall_thickness, wall_width, (wall_height - wall_center_z - hole_height/2),
              wall_x, wall_center_y, wall_center_z + hole_height/2 + (wall_height - wall_center_z - hole_height/2)/2, "wall_top");
    
    // Left segment - positioned at left edge
    add_block(wall_thickness, side_width, hole_height,
              wall_x, -wall_width/2 + side_width/2, wall_center_z, "wall_left");
    
    // Right segment - positioned at right edge
    add_block(wall_thickness, side_width, hole_height,
              wall_x, wall_width/2 - side_width/2, wall_center_z, "wall_right");

    ROS_INFO("Generated vertical wall with hole at x=%.2f for arm passage", wall_x);
}

bool PlannerTester::is_position_valid(double x, double y, double z, double length, double width, double height)
{
    if(z - height/2 <= 0.01) // Ground plane thickness + small margin
    {
        return false;
    }
    
    double distance_from_origin = sqrt(x*x + y*y);
    if(distance_from_origin + std::max({length, width})/2 > WORKSPACE_RADIUS)
    {
        return false;
    }
    
    if(intersects_with_arm_base(x, y, z, length, width, height))
    {
        return false;
    }
    
    if(intersects_with_existing_blocks(x, y, z, length, width, height))
    {
        return false;
    }
    
    return true;
}

bool PlannerTester::intersects_with_arm_base(double x, double y, double z, double length, double width, double height)
{
    double distance_from_origin = sqrt(x*x + y*y);
    
    double block_min_distance = distance_from_origin - std::max({length, width})/2;
    
    return block_min_distance < ARM_SAFETY_RADIUS;
}

bool PlannerTester::intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height)
{
    for(const auto& existing_block : generated_blocks)
    {
        double dx = fabs(x - existing_block.x);
        double dy = fabs(y - existing_block.y);
        double dz = fabs(z - existing_block.z);
        
        double min_dx = (length + existing_block.length)/2 + BLOCK_SAFETY_MARGIN;
        double min_dy = (width + existing_block.width)/2 + BLOCK_SAFETY_MARGIN;
        double min_dz = (height + existing_block.height)/2 + BLOCK_SAFETY_MARGIN;
        
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

bool PlannerTester::loadYAMLConfig()
{
    try {
        std::ifstream test_file(yaml_config_path);
        if (!test_file.good()) {
            ROS_ERROR("YAML config file does not exist or is not readable: %s", yaml_config_path.c_str());
            return false;
        }
        test_file.close();
        
        YAML::Node config = YAML::LoadFile(yaml_config_path);
        if (config["RRTstar"]) {
            ROS_INFO("Successfully loaded YAML configuration from: %s", yaml_config_path.c_str());
            return true;
        } else {
            ROS_ERROR("YAML config does not contain RRTstar section");
            return false;
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load YAML config: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load YAML config: %s", e.what());
    }
    return false;
}

void PlannerTester::updateYAMLConfig(const RRTStarParams& params)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_config_path);
        
        if (config["RRTstar"]) {
            config["RRTstar"]["range"] = params.range;
            config["RRTstar"]["goal_bias"] = params.goal_bias;
            config["RRTstar"]["delay_collision_checking"] = params.delay_collision_checking;
            config["RRTstar"]["use_k_nearest"] = params.use_k_nearest;
            config["RRTstar"]["rewire_factor"] = params.rewire_factor;
            config["RRTstar"]["max_nearest_neighbors"] = params.max_nearest_neighbors;
            config["RRTstar"]["max_states"] = params.max_states;
            config["RRTstar"]["use_informed_sampling"] = params.use_informed_sampling;
            config["RRTstar"]["sample_rejection_attempts"] = params.sample_rejection_attempts;
            config["RRTstar"]["use_rejection_sampling"] = params.use_rejection_sampling;
            
            std::ofstream fout(yaml_config_path);
            if (fout.is_open()) {
                fout << config;
                fout.close();
                ROS_INFO("Updated YAML config with new RRT* parameters");
            } else {
                ROS_ERROR("Failed to open YAML file for writing: %s", yaml_config_path.c_str());
            }
        } else {
            ROS_ERROR("YAML config does not contain RRTstar section");
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to update YAML config: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to update YAML config: %s", e.what());
    }
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



void PlannerTester::setRRTStarParameters(const RRTStarParams& params)
{
    ROS_INFO("Setting RRT* parameters: range=%.3f, goal_bias=%.3f, rewire_factor=%.3f, max_neighbors=%d",
             params.range, params.goal_bias, params.rewire_factor, params.max_nearest_neighbors);
    
    updateYAMLConfig(params);
    
    ros::Duration(0.1).sleep();
}

std::vector<RRTStarParams> PlannerTester::generateParameterCombinations(int num_trials)
{
    std::vector<RRTStarParams> combinations;
    
    std::vector<double> range_values = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3};
    std::vector<double> goal_bias_values = {0.01, 0.05, 0.1, 0.15, 0.2, 0.25};
    std::vector<double> rewire_values = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
    std::vector<int> neighbor_values = {5, 10, 15, 20, 25, 30};
    
    std::vector<int> range_indices(num_trials);
    std::vector<int> goal_bias_indices(num_trials);
    std::vector<int> rewire_indices(num_trials);
    std::vector<int> neighbor_indices(num_trials);
    
    for (int i = 0; i < num_trials; ++i) {
        range_indices[i] = i % range_values.size();
        goal_bias_indices[i] = i % goal_bias_values.size();
        rewire_indices[i] = i % rewire_values.size();
        neighbor_indices[i] = i % neighbor_values.size();
    }

    std::mt19937 rng(42); 
    std::shuffle(range_indices.begin(), range_indices.end(), rng);
    std::shuffle(goal_bias_indices.begin(), goal_bias_indices.end(), rng);
    std::shuffle(rewire_indices.begin(), rewire_indices.end(), rng);
    std::shuffle(neighbor_indices.begin(), neighbor_indices.end(), rng);
    
    for (int i = 0; i < num_trials; ++i)
    {
        RRTStarParams params;
        
      
        params.range = range_values[range_indices[i]];
        params.goal_bias = goal_bias_values[goal_bias_indices[i]];
        params.rewire_factor = rewire_values[rewire_indices[i]];
        params.max_nearest_neighbors = neighbor_values[neighbor_indices[i]];
       
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

PlanningMetrics PlannerTester::runSingleTest(const RRTStarParams& params)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    setRRTStarParameters(params);
    
    group.setPoseTarget(fixed_goal_pose);
    
    bool success = (group.plan(my_xarm_plan) == 
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    PlanningMetrics metrics;
    metrics.planning_time = duration.count() / 1000.0;
    metrics.success = success;
    
    if (success)
    {
        metrics.path_length = calculatePathLength(my_xarm_plan.trajectory_);
    }
    
    return metrics;
}

void PlannerTester::logResults(const RRTStarParams& params, const PlanningMetrics& metrics, int trial_id, std::ofstream& csv_file)
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
    
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
             << (metrics.success ? "true" : "false") << "\n";
    
    csv_file.flush();
    
    ROS_INFO("Trial %d completed: success=%s, time=%.3fs, path_length=%.3f", 
             trial_id, metrics.success ? "true" : "false", 
             metrics.planning_time, metrics.path_length);
}

void PlannerTester::runParameterTests(int num_trials)
{

    auto param_combinations = generateParameterCombinations(num_trials);
    
    
    std::string csv_path = "/home/docker_ws/wall_planner_testing_results.csv";
    std::ofstream csv_file(csv_path);
    
    if (!csv_file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
        return;
    }
    
    csv_file << "trial_id,timestamp,range,goal_bias,rewire_factor,max_nearest_neighbors,"
             << "delay_collision_checking,use_k_nearest,max_states,use_informed_sampling,"
             << "sample_rejection_attempts,use_rejection_sampling,planning_time,path_length,success\n";
    
    ROS_INFO("Starting parameter testing with %d trials...", num_trials);
    
    for (int trial = 0; trial < num_trials; ++trial)
    {
        ROS_INFO("Running Trial %d/%d", trial + 1, num_trials);
        
   
        auto metrics = runSingleTest(param_combinations[trial]);
        
       
        logResults(param_combinations[trial], metrics, trial + 1, csv_file);
        
       
        ros::Duration(0.5).sleep();
    }
    
    csv_file.close();
    ROS_INFO("Completed %d trials. Results saved to %s", num_trials, csv_path.c_str());

    int success_count = 0, total_lines = 0;
    std::ifstream summary_file(csv_path);
    std::string line;
    if (summary_file.is_open()) {
        std::getline(summary_file, line);
        while (std::getline(summary_file, line)) {
            total_lines++;
            size_t last_comma = line.rfind(',');
            if (last_comma != std::string::npos) {
                std::string result = line.substr(last_comma + 1);
                if (result == "true")
                    success_count++;
            }
        }
        summary_file.close();
    }
    int fail_count = total_lines - success_count;
    ROS_INFO("Results summary:");
    ROS_INFO("Total trials: %d", total_lines);
    ROS_INFO("Successful trials: %d", success_count);
    ROS_INFO("Failed trials: %d", fail_count);
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

 
  int num_trials = 50; 
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


  ros::waitForShutdown();
  return 0;
}