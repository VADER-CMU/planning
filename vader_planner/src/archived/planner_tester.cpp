#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>

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

#include <Eigen/Dense>

#include "utils/utils.h"

// #include "utils/vader_cost_objective.h"

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
    double joint_space_path_length;
    double task_space_path_length;
    bool success;
    
    PlanningMetrics() : planning_time(0.0), joint_space_path_length(0.0), task_space_path_length(0.0), success(false) {}
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

    std::shared_ptr<XArmForwardKinematics> xarm_fk_ptr;


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
    
    double calculateTaskPathLength(const moveit_msgs::RobotTrajectory& trajectory);
    double calculateJointPathLength(const moveit_msgs::RobotTrajectory& trajectory);

    void init();
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

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "Planner Tester Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  
  rng.seed(42);
  
  fixed_goal_pose.position.x = 0.5;
  fixed_goal_pose.position.y = 0.0;
  fixed_goal_pose.position.z = 0.4;
  fixed_goal_pose.orientation.w = 1.0;
  fixed_goal_pose.orientation.x = 0.0;
  fixed_goal_pose.orientation.y = 0.0;
  fixed_goal_pose.orientation.z = 0.0;

  yaml_config_path = "/home/docker_ws/src/planning/vader_planner/config/planner_tester_ompl.yaml";
  
  if (!loadYAMLConfig()) {
    ROS_WARN("Failed to load YAML config from all paths, using default parameters");
  }

  xarm_fk_ptr = std::make_shared<XArmForwardKinematics>();
  
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
    ROS_INFO("Generating %d strategic blocks between arm and goal...", NUM_RANDOM_BLOCKS);
    
    double goal_x = fixed_goal_pose.position.x;
    double goal_y = fixed_goal_pose.position.y;
    double goal_z = fixed_goal_pose.position.z;
    
    double path_length = sqrt(goal_x*goal_x + goal_y*goal_y + goal_z*goal_z);
    double min_clearance = 0.12;
    
    for(int i = 0; i < NUM_RANDOM_BLOCKS; i++)
    {
        double length = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        double width = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        double height = random_double(MIN_BLOCK_SIZE, MAX_BLOCK_SIZE);
        
        double t = 0.25 + (0.5 * random_double(0.0, 1.0));
        
        double path_x = t * goal_x;
        double path_y = t * goal_y;
        double path_z = t * goal_z;
        
        double max_offset = min_clearance - std::max({length, width, height})/2;
        if(max_offset < 0.02) max_offset = 0.02;
        
        double side = (random_double(0.0, 1.0) < 0.5) ? -1.0 : 1.0;
        double offset_distance = random_double(max_offset, max_offset + 0.05);
        
        double perpendicular_x = -goal_y / path_length;
        double perpendicular_y = goal_x / path_length;
        
        double x = path_x + side * offset_distance * perpendicular_x;
        double y = path_y + side * offset_distance * perpendicular_y;
        double z = random_double(0.05 + height/2, std::min(goal_z - 0.1, 0.3));
        
        if(sqrt(x*x + y*y) < ARM_SAFETY_RADIUS + std::max({length, width})/2)
        {
            continue;
        }
        
        BlockInfo block_info;
        block_info.x = x;
        block_info.y = y;
        block_info.z = z;
        block_info.length = length;
        block_info.width = width;
        block_info.height = height;
        block_info.id = "strategic_block_" + std::to_string(generated_blocks.size());
        
        generated_blocks.push_back(block_info);
        
        add_block(length, width, height, x, y, z, block_info.id);
        
        ROS_INFO("Generated block %lu at (%.2f, %.2f, %.2f) with size (%.2f, %.2f, %.2f)", 
                 generated_blocks.size(), x, y, z, length, width, height);
    }
    
    ROS_INFO("Successfully generated %lu strategic blocks", generated_blocks.size());
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

double PlannerTester::calculateJointPathLength(const moveit_msgs::RobotTrajectory& trajectory)
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

double PlannerTester::calculateTaskPathLength(const moveit_msgs::RobotTrajectory& trajectory)
{
    if(trajectory.joint_trajectory.points.empty())
        return 0.0;
    
    double total_length = 0.0;
    
    for(size_t i = 1; i < trajectory.joint_trajectory.points.size(); i++)
    {
        double segment_length = 0.0;
        const auto& prev_point = trajectory.joint_trajectory.points[i-1];
        const auto& curr_point = trajectory.joint_trajectory.points[i];
        std::array<double, XArmForwardKinematics::N_JOINTS> prev_arr, curr_arr;
        for (size_t i = 0; i < 7; ++i){
            prev_arr[i] = prev_point.positions[i];
            curr_arr[i] = curr_point.positions[i];
        }    
        // Compute FK for both points as Eigen::Matrix4d
        Eigen::Matrix4d prev_tf = xarm_fk_ptr->forward_kinematics(prev_arr);
        Eigen::Matrix4d curr_tf = xarm_fk_ptr->forward_kinematics(curr_arr);

        // Extract translation components
        Eigen::Vector3d prev_pos = prev_tf.block<3,1>(0,3);
        Eigen::Vector3d curr_pos = curr_tf.block<3,1>(0,3);

        // Compute L2 norm between the two positions
        segment_length = (curr_pos - prev_pos).norm();

        total_length += segment_length;
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
        
      
        // params.range = range_values[range_indices[i]];
        // params.goal_bias = goal_bias_values[goal_bias_indices[i]];
        // params.rewire_factor = rewire_values[rewire_indices[i]];
        // params.max_nearest_neighbors = neighbor_values[neighbor_indices[i]];
       
        // params.delay_collision_checking = (i % 2 == 0);
        // params.use_k_nearest = (i % 2 == 0);
        // params.max_states = 1000 + (i % 5) * 200;
        // params.use_informed_sampling = (i % 3 == 0);
        // params.sample_rejection_attempts = 50 + (i % 10) * 10;
        // params.use_rejection_sampling = (i % 2 == 0);
        
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
        metrics.joint_space_path_length = calculateJointPathLength(my_xarm_plan.trajectory_);
        metrics.task_space_path_length = calculateTaskPathLength(my_xarm_plan.trajectory_);
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
             << metrics.joint_space_path_length << ","
             << metrics.task_space_path_length << ","
             << (metrics.success ? "true" : "false") << "\n";
    
    csv_file.flush();
    
    ROS_INFO("Trial %d completed: success=%s, time=%.3fs, joint_space_path_length=%.3f, task_space_path_length=%.3f",
             trial_id, metrics.success ? "true" : "false", 
             metrics.planning_time, metrics.joint_space_path_length, metrics.task_space_path_length);
}

void PlannerTester::runParameterTests(int num_trials)
{

    auto param_combinations = generateParameterCombinations(num_trials);
    
    
    char cwd[1024];
    std::string csv_path;
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        csv_path = std::string(cwd) + "/ompl_testing_results.csv";
    } else {
        csv_path = std::string(getenv("HOME")) + "/ompl_ws/ompl_testing_results.csv";
    }
    std::ofstream csv_file(csv_path);
    
    if (!csv_file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
        return;
    }
    
    csv_file << "trial_id,timestamp,range,goal_bias,rewire_factor,max_nearest_neighbors,"
             << "delay_collision_checking,use_k_nearest,max_states,use_informed_sampling,"
             << "sample_rejection_attempts,use_rejection_sampling,planning_time,joint_space_path_length,task_space_path_length,success\n";
    
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

    // Accurate summary counting
    int success_count = 0, fail_count = 0, total_lines = 0;
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
                else if (result == "false")
                    fail_count++;
            }
        }
        summary_file.close();
    }
    ROS_INFO("Results summary:");
    ROS_INFO("Total lines (excluding header): %d", total_lines);
    ROS_INFO("Successful trials: %d", success_count);
    ROS_INFO("Failed trials: %d", fail_count);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_tester");
  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_name", robot_name);
  PlannerTester::PLANNING_GROUP = "L_xarm7";

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
      ROS_INFO("Run with 'test' argument to start parameter testing");
  }


  ros::waitForShutdown();
  return 0;
}