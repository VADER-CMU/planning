// /* Copyright 2018 UFACTORY Inc. All Rights Reserved.
//  *
//  * Software License Agreement (BSD License)
//  *
//  * Author: Jason Peng <jason@ufactory.cc>
//  ============================================================================*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <vader_planner/pose_plan.h>
#include <vader_planner/joint_plan.h>
#include <vader_planner/exec_plan.h>
#include <vader_planner/single_straight_plan.h>

#include <tf/transform_broadcaster.h>//include for quaternion rotation 
#include <tf/transform_listener.h>//include for quaternion rotation 
#include <tf/transform_datatypes.h>//include for quaternion rotation 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//include <xarm_moveit_servo/kinematic_constraints/utils.h>


#include <geometric_shapes/shapes.h> // Include for cylinder shape
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;


namespace rvt = rviz_visual_tools;

class VADERPlanner
{
  public:
    VADERPlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
    VADERPlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~VADERPlanner(){ delete visual_tools;};
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

    void init();
    bool do_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res);
    bool do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res);
    bool do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res);
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
  plan_pose_srv = node_handle.advertiseService("xarm_pose_plan", &VADERPlanner::do_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &VADERPlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &VADERPlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &VADERPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &VADERPlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
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
    ROS_INFO_NAMED("vader_planner", "Visualizing plan as trajectory line");
   
    visual_tools->deleteAllMarkers();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
  }
}

// bool VADERPlanner::do_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
// { 
//   //snippet for quaternion transformation
//   tf::Quaternion original_quat;
//   tf::quaternionMsgToTF(req.target.orientation, original_quat);
  
//   geometry_msgs::Pose msg_quat;
//   msg_quat.orientation.x = req.target.orientation.x;    
//   msg_quat.orientation.y = req.target.orientation.y;
//   msg_quat.orientation.z = req.target.orientation.z;
//   msg_quat.orientation.w = req.target.orientation.w;
//   msg_quat.position.x = req.target.position.x;
//   msg_quat.position.y = req.target.position.y;
//   msg_quat.position.z = req.target.position.z;

//   tf::Quaternion rotation_quat;
//   rotation_quat.setRPY(0, M_PI/2, 0);

//   tf::Quaternion rotated_quat = rotation_quat * original_quat;
  
//   rotated_quat.normalize();
//   tf::quaternionTFToMsg(rotated_quat, req.target.orientation);

//   ROS_INFO("Quaternion: old=%f, new=%f,rot_quat=%f", original_quat, rotated_quat, M_PI);
// //end

//   // Convert quaternion to rotation matrix
//   tf::Matrix3x3 rotation_matrix(rotated_quat);
  
//   // Define offset in local Z direction (0.5m along local frame Z-axis)
//   tf::Vector3 offset(0.0, 0.0, 0.15);
  
//   // Transform the offset using the rotation matrix
//   tf::Vector3 transformed_offset = rotation_matrix * offset;

//   // Apply transformed offset to position
//   req.target.position.x += transformed_offset.x();
//   req.target.position.y += transformed_offset.y();
//   req.target.position.z += transformed_offset.z();

//   group.setPoseTarget(req.target);

//   tf::Vector3 x_axis(1, 0, 0);
//   tf::Vector3 rotated_x_axis = tf::quatRotate(original_quat, x_axis);
  
//   moveit_msgs::CollisionObject cylinder_object;
//   cylinder_object.header.frame_id = group.getPlanningFrame();
//   cylinder_object.id = "cylinder_1";
//   cylinder_object.header.frame_id = "link_base";

//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.CYLINDER;
//   primitive.dimensions.resize(2);
//   primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1; // Height - to be changed based on input
//   primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.075; // Radius - to be changed based on input
//   tf::Vector3 displacement = 1.1*rotated_x_axis.normalized()*primitive.dimensions[primitive.CYLINDER_RADIUS];

//   geometry_msgs::Pose cylinder_pose;
//   cylinder_pose.orientation.x = msg_quat.orientation.x;
//   cylinder_pose.orientation.y = msg_quat.orientation.y;
//   cylinder_pose.orientation.z = msg_quat.orientation.z;
//   cylinder_pose.orientation.w = msg_quat.orientation.w;
//   cylinder_pose.position.x = msg_quat.position.x;//
//   cylinder_pose.position.y = msg_quat.position.y;
//   cylinder_pose.position.z = msg_quat.position.z;

//   cylinder_object.primitives.push_back(primitive);
//   cylinder_object.primitive_poses.push_back(cylinder_pose);
//   cylinder_object.operation = moveit_msgs::CollisionObject::ADD;

//   planning_scene_interface.applyCollisionObject(cylinder_object);

//   ROS_INFO("Added cylinder: x=%f, y=%f, z=%f", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
//   ROS_INFO("vader_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
//     req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
//     req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);
    


//   moveit::core::RobotStatePtr current_state = group.getCurrentState();
//   const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

//   bool found_ik = current_state->setFromIK(joint_model_group, req.target, 10, 0.1);

//   bool success = false;

//   if (found_ik) {
//     std::vector <double> joint_values;
//     current_state->copyJointGroupPositions(joint_model_group, joint_values);

//     ROS_INFO("Found IK solution:");

//     group.setJointValueTarget(joint_values);
//     success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     res.success = success;
//     ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
//     show_trail(success);

//   }
//   else{
//     ROS_ERROR("Did not find IK solution");
//     success = false;
//   }
  

//   // cylinder_object.operation = moveit_msgs::CollisionObject::REMOVE; // Remove the cylinder
//   // planning_scene_interface.applyCollisionObject(cylinder_object); 

//   return success;
// }

bool VADERPlanner::do_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
{ 
  //snippet for quaternion transformation
  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(req.target.orientation, original_quat);
  
  geometry_msgs::Pose msg_quat;
  msg_quat.orientation.x = req.target.orientation.x;    
  msg_quat.orientation.y = req.target.orientation.y;
  msg_quat.orientation.z = req.target.orientation.z;
  msg_quat.orientation.w = req.target.orientation.w;
  msg_quat.position.x = req.target.position.x;
  msg_quat.position.y = req.target.position.y;
  msg_quat.position.z = req.target.position.z;

  tf::Quaternion rotation_quat;
  rotation_quat.setRPY(0, M_PI/2, 0);

  tf::Quaternion rotated_quat = rotation_quat * original_quat;
  
  rotated_quat.normalize();
  tf::quaternionTFToMsg(rotated_quat, req.target.orientation);

  ROS_INFO("Quaternion: old=%f, new=%f,rot_quat=%f", original_quat, rotated_quat, M_PI);

  // Convert quaternion to rotation matrix
  tf::Matrix3x3 rotation_matrix(rotated_quat);
  
  // Define offset in local Z direction (0.15m along local frame Z-axis)
  tf::Vector3 offset(0.0, 0.0, 0.15);
  
  // Transform the offset using the rotation matrix
  tf::Vector3 transformed_offset = rotation_matrix * offset;

  // Apply transformed offset to position
  geometry_msgs::Pose final_grasp_pose = req.target;
  final_grasp_pose.position.x += transformed_offset.x();
  final_grasp_pose.position.y += transformed_offset.y();
  final_grasp_pose.position.z += transformed_offset.z();

  // Create and add the cylinder to the planning scene
  tf::Vector3 x_axis(1, 0, 0);
  tf::Vector3 rotated_x_axis = tf::quatRotate(original_quat, x_axis);
  
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.header.frame_id = group.getPlanningFrame();
  cylinder_object.id = "cylinder_1";
  cylinder_object.header.frame_id = "link_base";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1; // Height
  primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.075; // Radius

  // Define cylinder position 
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.x = msg_quat.orientation.x;
  cylinder_pose.orientation.y = msg_quat.orientation.y;
  cylinder_pose.orientation.z = msg_quat.orientation.z;
  cylinder_pose.orientation.w = msg_quat.orientation.w;
  
  // Place cylinder at original position
  cylinder_pose.position.x = msg_quat.position.x;
  cylinder_pose.position.y = msg_quat.position.y;
  cylinder_pose.position.z = msg_quat.position.z;

  cylinder_object.primitives.push_back(primitive);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = moveit_msgs::CollisionObject::ADD;

  // Spawn the cylinder in the planning scene
  planning_scene_interface.applyCollisionObject(cylinder_object);

  ROS_INFO("Added cylinder: x=%f, y=%f, z=%f", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
  
  // Calculate pre-grasp position
  // Move back from the cylinder position along the rotated x-axis
  double pregrasp_distance = 0.3; // Larger distance to avoid collision planning issues
  
  geometry_msgs::Pose pregrasp_pose;
  pregrasp_pose.orientation = final_grasp_pose.orientation; // Same orientation as final grasp
  pregrasp_pose.position.x = cylinder_pose.position.x - pregrasp_distance * rotated_x_axis.x();
  pregrasp_pose.position.y = cylinder_pose.position.y - pregrasp_distance * rotated_x_axis.y();
  pregrasp_pose.position.z = cylinder_pose.position.z - pregrasp_distance * rotated_x_axis.z();
  
  ROS_INFO("Planning to pre-grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z, 
    pregrasp_pose.orientation.x, pregrasp_pose.orientation.y, pregrasp_pose.orientation.z, pregrasp_pose.orientation.w);
  
  // Set the planning pipeline to CHOMP (if it's not already the default)
  group.setPlannerId("CHOMP");

  // First plan: Go to pre-grasp position
  group.setPoseTarget(pregrasp_pose);
  
  // Find IK solution for pre-grasp
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  
  bool found_ik = current_state->setFromIK(joint_model_group, pregrasp_pose, 10, 0.1);
  bool success = false;
  
  if (found_ik) {
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    ROS_INFO("Found IK solution for pre-grasp pose");
    
    group.setJointValueTarget(joint_values);
    success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
      ROS_INFO("Pre-grasp plan SUCCEEDED");
      show_trail(success);
      
      // Execute the plan to pre-grasp
      bool exec_result = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (exec_result) {
        ROS_INFO("Successfully moved to pre-grasp position");
        
        // Second plan: Go to final grasp position
        // Use regular path planning (CHOMP) for final grasp position
        group.setPoseTarget(final_grasp_pose);
        
        // Check IK solution for final grasp
        found_ik = current_state->setFromIK(joint_model_group, final_grasp_pose, 10, 0.1);
        
        if (found_ik) {
          current_state->copyJointGroupPositions(joint_model_group, joint_values);
          group.setJointValueTarget(joint_values);
          
          success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          
          if (success) {
            ROS_INFO("Grasp approach plan SUCCEEDED");
            show_trail(success);
            
            // Execute the plan to grasp
            exec_result = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (exec_result) {
              ROS_INFO("Successfully moved to grasp position");
              res.success = true;
            } else {
              ROS_ERROR("Failed to execute grasp approach");
              res.success = false;
            }
          } else {
            ROS_ERROR("Failed to plan grasp approach");
            res.success = false;
          }
        } else {
          ROS_ERROR("Did not find IK solution for grasp pose");
          res.success = false;
        }
      } else {
        ROS_ERROR("Failed to execute pre-grasp plan");
        res.success = false;
      }
    } else {
      ROS_ERROR("Failed to plan to pre-grasp position");
      res.success = false;
    }
  } else {
    ROS_ERROR("Did not find IK solution for pre-grasp pose");
    res.success = false;
  }
  
  return res.success;
}

bool VADERPlanner::do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res)
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

bool VADERPlanner::do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res)
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

bool VADERPlanner::exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res)
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
void VADERPlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("Received Execution Command !!!!!");
    group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
  }
}


std::string VADERPlanner::PLANNING_GROUP; // Definition of static class member

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vader_dual_arm_planner");
  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_name", robot_name);
  VADERPlanner::PLANNING_GROUP = robot_name;

  VADERPlanner planner(VADERPlanner::PLANNING_GROUP);

  planner.start();

  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
}

