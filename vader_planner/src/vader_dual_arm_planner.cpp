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

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

const std::string PLANNING_GROUP_L = "L_xarm7";
const std::string PLANNING_GROUP_R = "R_xarm7";

namespace rvt = rviz_visual_tools;

class VADERPlanner
{
  public:
    VADERPlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name), PLANNING_GROUP(plan_group_name){init();};
    // VADERPlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~VADERPlanner(){ delete visual_tools;};
    void start();
    void stop();


  private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_names;
    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    std::string left_or_right_arm;
    std::string PLANNING_GROUP;

    ros::Publisher display_path;
    ros::ServiceServer plan_pose_srv;
    ros::ServiceServer plan_joint_srv;
    ros::ServiceServer sing_cart_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    bool do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res);
    bool do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res);
    bool do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
    void show_trail(bool plan_result);
};

void VADERPlanner::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService(PLANNING_GROUP + "_xarm_pose_plan", &VADERPlanner::do_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService(PLANNING_GROUP + "_xarm_joint_plan", &VADERPlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService(PLANNING_GROUP + "_xarm_straight_plan", &VADERPlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe(PLANNING_GROUP + "_xarm_planner_exec", 10, &VADERPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService(PLANNING_GROUP + "_xarm_exec_plan", &VADERPlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, PLANNING_GROUP + "_xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

}

void VADERPlanner::start()
{
  ROS_INFO("Spinning");
  spinner.start();
}

void VADERPlanner::stop()
{
  spinner.stop();
}

void VADERPlanner::show_trail(bool plan_result)
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

bool VADERPlanner::do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res)
{
  group.setPoseTarget(req.target);
  
  ROS_INFO("xarm_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
    req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
    req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);

  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  show_trail(success);

  return success;
}

bool VADERPlanner::do_single_cartesian_plan(xarm_planner::single_straight_plan::Request &req, xarm_planner::single_straight_plan::Response &res)
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
  fprintf(stderr, "[VADERPlanner::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

  res.success = success;
  show_trail(success);
  
  return success;

}

bool VADERPlanner::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
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
  return success;
}

bool VADERPlanner::exec_plan_cb(xarm_planner::exec_plan::Request &req, xarm_planner::exec_plan::Response &res)
{
  if(req.exec)
  {
    ROS_INFO("%s Received Execution Service Request", PLANNING_GROUP.c_str());
    bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
    res.success = finish_ok;
    return finish_ok;
  }

  res.success = false;
  return false;
}

/* execution subscriber call-back function */
void VADERPlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("%s Received Execution Command !!!!!", PLANNING_GROUP.c_str());
    group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
  }
}


// std::string VADERPlanner::PLANNING_GROUP; // Definition of static class member

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vader_dual_arm_planner");
  ros::NodeHandle nh;
  // std::string robot_name1 = "";
  // nh.getParam("robot_name", robot_name1);
  // VADERPlanner::PLANNING_GROUP = robot_name1;

  VADERPlanner plannerL(PLANNING_GROUP_L);
  VADERPlanner plannerR(PLANNING_GROUP_R);

  plannerL.start();
  plannerR.start();
  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");
  //rosservice call L_xarm7_xarm_pose_plan 'target: [[0.28, 0.2, 0.2], [1.0, 0.0, 0.0, 0.0]]'
  //rosservice call R_xarm7_xarm_pose_plan 'target: [[0.28, 0.8, 0.2], [1.0, 0.0, 0.0, 0.0]]' //right arm is set at [0,1,0] origin, so move target accordingly
  //rosservice call L_xarm7_xarm_exec_plan 'true'; rosservice call R_xarm7_xarm_exec_plan 'true' //this is sequential.

  ros::waitForShutdown();
  return 0;
}

