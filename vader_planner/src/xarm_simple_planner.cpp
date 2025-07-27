/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
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

class XArmSimplePlanner
{
  public:
    XArmSimplePlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
    XArmSimplePlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~XArmSimplePlanner(){ delete visual_tools;};
    void start();
    void stop();

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
    
    // New functions for random block generation
    void generate_random_blocks();
    bool is_position_valid(double x, double y, double z, double length, double width, double height);
    bool intersects_with_arm_base(double x, double y, double z, double length, double width, double height);
    bool intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height);
    double random_double(double min, double max);
};

void XArmSimplePlanner::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("xarm_pose_plan", &XArmSimplePlanner::do_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &XArmSimplePlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &XArmSimplePlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &XArmSimplePlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &XArmSimplePlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  
  // Initialize random number generator
  rng.seed(std::chrono::steady_clock::now().time_since_epoch().count());
  
  create_ground_plane();
  
  ros::Duration(1.0).sleep();
  
  // Generate random blocks instead of predefined ones
  generate_random_blocks();
}

void XArmSimplePlanner::generate_random_blocks()
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

bool XArmSimplePlanner::is_position_valid(double x, double y, double z, double length, double width, double height)
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

bool XArmSimplePlanner::intersects_with_arm_base(double x, double y, double z, double length, double width, double height)
{
    // Define arm base as a cylinder around origin
    double distance_from_origin = sqrt(x*x + y*y);
    
    // Check if block's closest point to origin is within safety radius
    double block_min_distance = distance_from_origin - std::max({length, width})/2;
    
    return block_min_distance < ARM_SAFETY_RADIUS;
}

bool XArmSimplePlanner::intersects_with_existing_blocks(double x, double y, double z, double length, double width, double height)
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

double XArmSimplePlanner::random_double(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

void XArmSimplePlanner::start()
{
  ROS_INFO("Spinning");
  spinner.start();
}

void XArmSimplePlanner::stop()
{
  spinner.stop();
}

void XArmSimplePlanner::show_trail(bool plan_result)
{
  if(plan_result)
  {
    ROS_INFO_NAMED("xarm_planner", "Visualizing plan as trajectory line");
    
    visual_tools->deleteAllMarkers();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
  }
}

void XArmSimplePlanner::analyze_trajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& plan_type)
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

void XArmSimplePlanner::create_ground_plane()
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

void XArmSimplePlanner::add_block(double length, double width, double height, double x, double y, double z, const std::string& block_id)
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

bool XArmSimplePlanner::do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res)
{
  group.setPoseTarget(req.target);
  
  ROS_INFO("xarm_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
    req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
    req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);

  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "POSE_PLAN");

  return success;
}

bool XArmSimplePlanner::do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res)
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
  fprintf(stderr, "[XArmSimplePlanner::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

  res.success = success;
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "CARTESIAN_PLAN");
  return success;

}

bool XArmSimplePlanner::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
{
  ROS_INFO("move_group_planner received new plan Request");
  if(!group.setJointValueTarget(req.target))
  {
    ROS_ERROR("setJointValueTarget() Failed! Please check the dimension and range of given joint target.");
    return false;
  }
  
  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
  show_trail(success);
  if(success) analyze_trajectory(my_xarm_plan.trajectory_, "JOINT_PLAN");
  return success;
}

bool XArmSimplePlanner::exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res)
{
  if(req.exec)
  {
    ROS_INFO("Received Execution Service Request");
    bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
    res.success = finish_ok;
    return finish_ok;
  }

  res.success = false;
  return false;
}

/* execution subscriber call-back function */
void XArmSimplePlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("Received Execution Command !!!!!");
    group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
  }
}


std::string XArmSimplePlanner::PLANNING_GROUP; // Definition of static class member

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_move_group_planner");
  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_name", robot_name);
  XArmSimplePlanner::PLANNING_GROUP = robot_name;

  XArmSimplePlanner planner;

  planner.start();

  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
}






// /* Copyright 2018 UFACTORY Inc. All Rights Reserved.
//  *
//  * Software License Agreement (BSD License)
//  *
//  * Author: Jason Peng <jason@ufactory.cc>
//  ============================================================================*/
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <std_msgs/Bool.h>
// #include <xarm_planner/pose_plan.h>
// #include <xarm_planner/joint_plan.h>
// #include <xarm_planner/exec_plan.h>
// #include <xarm_planner/single_straight_plan.h>

// #define SPINNER_THREAD_NUM 2

// /* Used for Cartesian path computation, please modify as needed: */
// const double jump_threshold = 0.0;
// const double eef_step = 0.005;
// const double maxV_scale_factor = 0.3; // check!!


// namespace rvt = rviz_visual_tools;

// class XArmSimplePlanner
// {
//   public:
//     XArmSimplePlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
//     XArmSimplePlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
//     ~XArmSimplePlanner(){ delete visual_tools;};
//     void start();
//     void stop();

//     static std::string PLANNING_GROUP; // declaration of static class member

//   private:
//     ros::NodeHandle node_handle;
//     ros::AsyncSpinner spinner;
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     std::vector<std::string> joint_names;
//     moveit::planning_interface::MoveGroupInterface group;
//     moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;
//     moveit_visual_tools::MoveItVisualTools *visual_tools;

//     ros::Publisher display_path;
//     ros::ServiceServer plan_pose_srv;
//     ros::ServiceServer plan_joint_srv;
//     ros::ServiceServer sing_cart_srv;
//     ros::Subscriber exec_plan_sub; /* non-blocking*/
//     ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

//     void init();
//     bool do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res);
//     bool do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res);
//     bool do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res);
//     bool exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res);
//     void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
//     void analyze_trajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& plan_type);
//     void show_trail(bool plan_result);
//     void create_ground_plane();
//     void add_block(double length, double width, double height, double x, double y, double z, const std::string& block_id = "");
    
// };

// void XArmSimplePlanner::init()
// {
//   joint_names = group.getJointNames();

//   display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

//   ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

//   ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

//   /* Notice: the correct way to specify member function as callbacks */
//   plan_pose_srv = node_handle.advertiseService("xarm_pose_plan", &XArmSimplePlanner::do_pose_plan, this);
//   plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &XArmSimplePlanner::do_joint_plan, this);
//   sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &XArmSimplePlanner::do_single_cartesian_plan, this);

//   exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &XArmSimplePlanner::execute_plan_topic, this);
//   exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &XArmSimplePlanner::exec_plan_cb, this);

//   visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 0.8;
//   visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
//   visual_tools->trigger();
//   create_ground_plane();
//   // Add labrynth here based on blocks
//   ros::Duration(1.0).sleep();
//   int LENGTH = 1.5;
//   int WIDTH = 0.2;
//   int HEIGHT = 0.4;
//   //Left block
//   add_block(LENGTH, WIDTH, HEIGHT, 0.5 + WIDTH/2, 0, HEIGHT/2, "Left Block");
//   //Top block
//   //Right block
//   //Front obstacle

// }

// void XArmSimplePlanner::start()
// {
//   ROS_INFO("Spinning");
//   spinner.start();
// }

// void XArmSimplePlanner::stop()
// {
//   spinner.stop();
// }

// void XArmSimplePlanner::show_trail(bool plan_result)
// {
//   if(plan_result)
//   {
//     ROS_INFO_NAMED("xarm_planner", "Visualizing plan as trajectory line");
    
//     visual_tools->deleteAllMarkers();
//     const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//     visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
//     visual_tools->trigger();
//   }
// }
// void XArmSimplePlanner::analyze_trajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& plan_type)
// {
//   if(trajectory.joint_trajectory.points.empty())
//   {
//     ROS_WARN("Empty trajectory for analysis");
//     return;
//   }

//   double total_length = 0.0;
//   double total_time = 0.0;
//   int num_points = trajectory.joint_trajectory.points.size();
  
//   // Calculate total joint space distance
//   for(size_t i = 1; i < trajectory.joint_trajectory.points.size(); i++)
//   {
//     double segment_length = 0.0;
//     const auto& prev_point = trajectory.joint_trajectory.points[i-1];
//     const auto& curr_point = trajectory.joint_trajectory.points[i];
    
//     for(size_t j = 0; j < prev_point.positions.size(); j++)
//     {
//       double joint_diff = curr_point.positions[j] - prev_point.positions[j];
//       segment_length += joint_diff * joint_diff;
//     }
//     total_length += sqrt(segment_length);
//   }
  
//   total_time = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  
//   ROS_INFO("=== TRAJECTORY ANALYSIS (%s) ===", plan_type.c_str());
//   ROS_INFO("Total joint space length: %.4f radians", total_length);
//   ROS_INFO("Total execution time: %.4f seconds", total_time);
//   ROS_INFO("Number of waypoints: %d", num_points);
//   ROS_INFO("Average speed: %.4f rad/s", total_length / total_time);
//   ROS_INFO("=== END ANALYSIS ===");
// }

// // Add these two functions to the private section of XArmSimplePlanner class:

// // Add these two functions to the private section of XArmSimplePlanner class:

// void XArmSimplePlanner::create_ground_plane()
// {
//     moveit_msgs::CollisionObject ground_plane;
//     ground_plane.header.frame_id = group.getPlanningFrame();
//     ground_plane.id = "ground_plane";

//     // Define ground plane as a box
//     shape_msgs::SolidPrimitive primitive;
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[0] = 2.0;  // 2m x 2m ground plane
//     primitive.dimensions[1] = 2.0;
//     primitive.dimensions[2] = 0.01; // 1cm thick

//     // Position ground plane at z = -0.005 (half thickness below z=0)
//     geometry_msgs::Pose ground_pose;
//     ground_pose.orientation.w = 1.0;
//     ground_pose.position.x = 0.0;
//     ground_pose.position.y = 0.0;
//     ground_pose.position.z = -0.005;

//     ground_plane.primitives.push_back(primitive);
//     ground_plane.primitive_poses.push_back(ground_pose);
//     ground_plane.operation = ground_plane.ADD;

//     std::vector<moveit_msgs::CollisionObject> collision_objects;
//     collision_objects.push_back(ground_plane);
//     planning_scene_interface.addCollisionObjects(collision_objects);

//     ROS_INFO("Ground plane added to planning scene");
// }

// void XArmSimplePlanner::add_block(double length, double width, double height, double x, double y, double z, const std::string& block_id)
// {
//     moveit_msgs::CollisionObject block;
//     block.header.frame_id = group.getPlanningFrame();
    
//     // Generate unique ID if not provided
//     if(block_id.empty())
//     {
//         static int block_counter = 0;
//         block.id = "block_" + std::to_string(block_counter++);
//     }
//     else
//     {
//         block.id = block_id;
//     }

//     // Define block as a box
//     shape_msgs::SolidPrimitive primitive;
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[0] = length;
//     primitive.dimensions[1] = width;
//     primitive.dimensions[2] = height;

//     // Position block at specified location
//     geometry_msgs::Pose block_pose;
//     block_pose.orientation.w = 1.0;
//     block_pose.position.x = x;
//     block_pose.position.y = y;
//     block_pose.position.z = z;

//     block.primitives.push_back(primitive);
//     block.primitive_poses.push_back(block_pose);
//     block.operation = block.ADD;

//     std::vector<moveit_msgs::CollisionObject> collision_objects;
//     collision_objects.push_back(block);
//     planning_scene_interface.addCollisionObjects(collision_objects);

//     ROS_INFO("Block '%s' (%.2fx%.2fx%.2f) added at position (%.2f, %.2f, %.2f)", 
//              block.id.c_str(), length, width, height, x, y, z);
// }

// bool XArmSimplePlanner::do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res)
// {
//   group.setPoseTarget(req.target);
  
//   ROS_INFO("xarm_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
//     req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
//     req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);

//   bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   res.success = success;
//   ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
//   show_trail(success);
//   if(success) analyze_trajectory(my_xarm_plan.trajectory_, "POSE_PLAN");

//   return success;
// }

// bool XArmSimplePlanner::do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res)
// {
//   std::vector<geometry_msgs::Pose> waypoints;
//   waypoints.push_back(req.target);
//   group.setMaxVelocityScalingFactor(maxV_scale_factor);
//   moveit_msgs::RobotTrajectory trajectory;
  
//   double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   bool success = true;
//   if(fraction<0.9)
//     success = false;
//   else
//   {
//     my_xarm_plan.trajectory_ = trajectory;
//   }
//   fprintf(stderr, "[XArmSimplePlanner::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

//   res.success = success;
//   show_trail(success);
//   if(success) analyze_trajectory(my_xarm_plan.trajectory_, "CARTESIAN_PLAN");
//   return success;

// }

// bool XArmSimplePlanner::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
// {
//   ROS_INFO("move_group_planner received new plan Request");
//   if(!group.setJointValueTarget(req.target))
//   {
//     ROS_ERROR("setJointValueTarget() Failed! Please check the dimension and range of given joint target.");
//     return false;
//   }
  
//   bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   res.success = success;
//   ROS_INFO_NAMED("move_group_planner", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
//   show_trail(success);
//   if(success) analyze_trajectory(my_xarm_plan.trajectory_, "JOINT_PLAN");
//   return success;
// }

// bool XArmSimplePlanner::exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res)
// {
//   if(req.exec)
//   {
//     ROS_INFO("Received Execution Service Request");
//     bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
//     res.success = finish_ok;
//     return finish_ok;
//   }

//   res.success = false;
//   return false;
// }

// /* execution subscriber call-back function */
// void XArmSimplePlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
// {
//   if(exec->data)
//   { 
//     ROS_INFO("Received Execution Command !!!!!");
//     group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
//   }
// }


// std::string XArmSimplePlanner::PLANNING_GROUP; // Definition of static class member

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "xarm_move_group_planner");
//   ros::NodeHandle nh;
//   std::string robot_name = "";
//   nh.getParam("robot_name", robot_name);
//   XArmSimplePlanner::PLANNING_GROUP = robot_name;

//   XArmSimplePlanner planner;

//   planner.start();

//   ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");

//   /* necessary: because AsyncSpinner is not operating in the same thread */
//   ros::waitForShutdown();
//   return 0;
// }

