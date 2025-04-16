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

#include <vader_msgs/SingleArmPlanRequest.h>
#include <vader_msgs/SingleArmExecutionRequest.h>

#include <tf/transform_broadcaster.h>//include for quaternion rotation 
#include <tf/transform_listener.h>//include for quaternion rotation 
#include <tf/transform_datatypes.h>//include for quaternion rotation 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    tf::TransformListener tf_listener;

    geometry_msgs::Pose pregrasp_pose;

    ros::Publisher display_path;
    ros::ServiceServer plan_pose_srv;

    ros::ServiceServer plan_pose_exec_srv;

    ros::ServiceServer plan_final_pose_srv;

    ros::ServiceServer plan_final_pose_exec_srv;
    
    ros::ServiceServer plan_joint_srv;
    ros::ServiceServer plan_grasp_srv; // New service for grasp pose planning
    ros::ServiceServer plan_grasp_exec_srv; // New service for grasp pose planning

    ros::ServiceServer sing_cart_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    bool do_pregrasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool do_pregrasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);
    bool do_final_grasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool do_final_grasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);

    bool do_grasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res);
    bool do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res);
    bool do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
    void show_trail(bool plan_result);
    tf::Vector3 calculatePerpendicularVector(const tf::Vector3& cylinder_axis);
    
    // Store cylinder data for access between pregrasp and grasp operations
    geometry_msgs::Pose stored_cylinder_pose;
    tf::Vector3 stored_cylinder_axis;
};

void VADERPlanner::init()
{
  joint_names = group.getVariableNames(); // Changed from deprecated getJointNames()

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("singleArmPlan", &VADERPlanner::do_pregrasp_pose_plan, this);
  plan_pose_exec_srv = node_handle.advertiseService("singleArmExec", &VADERPlanner::do_pregrasp_exec_plan, this);
  plan_final_pose_srv = node_handle.advertiseService("singleArmFinalPlan", &VADERPlanner::do_final_grasp_pose_plan, this);
  plan_final_pose_exec_srv = node_handle.advertiseService("singleArmFinalExec", &VADERPlanner::do_final_grasp_exec_plan, this);
  plan_grasp_srv = node_handle.advertiseService("xarm_grasp_plan", &VADERPlanner::do_grasp_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &VADERPlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &VADERPlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &VADERPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &VADERPlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

// Add a cuboid as a ground plane to the scene
moveit_msgs::CollisionObject ground_plane;
ground_plane.header.frame_id = group.getPlanningFrame();
ground_plane.id = "ground_plane";

// Define the cuboid dimensions
shape_msgs::SolidPrimitive ground_primitive;
ground_primitive.type = ground_primitive.BOX;
ground_primitive.dimensions.resize(3);
ground_primitive.dimensions[0] = 2.0; // Length in x-direction
ground_primitive.dimensions[1] = 2.0; // Width in y-direction
ground_primitive.dimensions[2] = 0.01; // Height in z-direction

// Define the pose of the cuboid
geometry_msgs::Pose ground_pose;
ground_pose.position.x = 0.0;
ground_pose.position.y = 0.0;
ground_pose.position.z = - 0.01 - (ground_primitive.dimensions[2] / 2.0);
ground_pose.orientation.w = 1.0; // No rotation

ground_plane.primitives.push_back(ground_primitive);
ground_plane.primitive_poses.push_back(ground_pose);
ground_plane.operation = moveit_msgs::CollisionObject::ADD;

// Add the ground plane to the planning scene
planning_scene_interface.applyCollisionObject(ground_plane);

}



void V

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

tf::Vector3 VADERPlanner::calculatePerpendicularVector(const tf::Vector3& cylinder_axis) {
  
  tf::Vector3 ref_vector(0, 0, 1);
  if (fabs(cylinder_axis.dot(ref_vector)) > 0.9) {
    ref_vector = tf::Vector3(1, 0, 0);
  }
  
  tf::Vector3 perpendicular = cylinder_axis.cross(ref_vector);
  perpendicular.normalize();
  return perpendicular;
}

bool VADERPlanner::do_pregrasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{ 
  // Store original request data for reference and later use in grasp planning
  geometry_msgs::Pose original_target;// = req.target;
  original_target.orientation.x = req.pepper.fruit_data.pose.orientation.x;    
  original_target.orientation.y = req.pepper.fruit_data.pose.orientation.y;
  original_target.orientation.z = req.pepper.fruit_data.pose.orientation.z;
  original_target.orientation.w = req.pepper.fruit_data.pose.orientation.w;
  original_target.position.x = req.pepper.fruit_data.pose.position.x;
  original_target.position.y = req.pepper.fruit_data.pose.position.y;
  original_target.position.z = req.pepper.fruit_data.pose.position.z; 
  stored_cylinder_pose = original_target;
  
  // Extract quaternion from the target
  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(original_target.orientation, original_quat);
  
  // Assume cylinder axis is aligned with original quaternion's z-axis
  tf::Matrix3x3 rotation_matrix(original_quat);
  tf::Vector3 cylinder_axis = tf::Vector3(rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2]);
  cylinder_axis.normalize();
  stored_cylinder_axis = cylinder_axis;
  
  // Apply rotation of pi/2 around Y axis to the quaternion
  tf::Quaternion rotation_quat;
  rotation_quat.setRPY(0, M_PI/2, 0);
  tf::Quaternion rotated_quat = rotation_quat * original_quat;
  rotated_quat.normalize();
  
  // Create a cylinder object for collision avoidance
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.header.frame_id = group.getPlanningFrame();
  cylinder_object.id = "cylinder_1";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.08; // Height
  primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.035; // Radius
  
  // Set cylinder position and orientation
  geometry_msgs::Pose cylinder_pose = original_target;
//   cylinder_pose.position.z -= 0.2;
  
  cylinder_object.primitives.push_back(primitive);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
  
  planning_scene_interface.applyCollisionObject(cylinder_object);
  
  ROS_INFO("Added cylinder at position (%f, %f, %f)", 
    cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
  
  // Convert quaternion to rotation matrix for offset calculation
  tf::Matrix3x3 rotation_matrix_rotated(rotated_quat);
  
  // Define offset in local Z direction (0.15m along local frame Z-axis)
  tf::Vector3 offset(0.0, 0.0, 0.0); // 0.15);
  
  // Transform the offset using the rotation matrix
  tf::Vector3 transformed_offset = rotation_matrix_rotated * offset;
  
  // Create the pre-grasp target pose
  pregrasp_pose.orientation.x = rotated_quat.x();
  pregrasp_pose.orientation.y = rotated_quat.y();
  pregrasp_pose.orientation.z = rotated_quat.z();
  pregrasp_pose.orientation.w = rotated_quat.w();
  
  // Apply transformed offset to position
  pregrasp_pose.position.x = original_target.position.x + transformed_offset.x();
  pregrasp_pose.position.y = original_target.position.y + transformed_offset.y();
  pregrasp_pose.position.z = original_target.position.z + transformed_offset.z();
  
  ROS_INFO("Planning to pre-grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z, 
    pregrasp_pose.orientation.x, pregrasp_pose.orientation.y, pregrasp_pose.orientation.z, pregrasp_pose.orientation.w);
  
  // Use IK to find joint values for the target pose
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  
  bool found_ik = current_state->setFromIK(joint_model_group, pregrasp_pose, 10, 0.1);
  bool success = false;
  
  if (found_ik) {
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    ROS_INFO("Found IK solution for pre-grasp pose");
    
    // Use joint values target instead of pose target
    group.setJointValueTarget(joint_values);
    
    // Plan using joint space goal
    success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.result = success;
    
    ROS_INFO_NAMED("move_group_planner", "This plan (joint-space goal) %s", success ? "SUCCEEDED" : "FAILED");
    
    show_trail(success);
    
  } else {
    ROS_ERROR("Did not find IK solution for pre-grasp pose");
    success = false;
  }
  
  res.result = success;
  return success;
}

bool VADERPlanner::do_pregrasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res){
  // Execute the plan
  bool exec_result = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (exec_result) {
    ROS_INFO("Successfully moved to pre-grasp position");
  } else {
    ROS_ERROR("Failed to execute pre-grasp plan");
  }
  res.result = exec_result;
  return exec_result;
}


bool VADERPlanner::do_final_grasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{ 
  //request parameters are ignored :sunglasses:
  geometry_msgs::Pose current_pose = pregrasp_pose;//group.getCurrentPose("vader_gripper_base_link").pose;
  ROS_INFO("Planning to final grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    current_pose.position.x, current_pose.position.y, current_pose.position.z, 
    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  
  tf::Vector3 approach(0.0, 0.0, 0.08); // 0.15);

  tf::Quaternion curr_quat;
  tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
  tf::Matrix3x3 curr_rot(curr_quat);

  tf::Vector3 transformed_approach = curr_rot * approach;

  current_pose.position.x += transformed_approach.x();
  current_pose.position.y += transformed_approach.y();
  current_pose.position.z += transformed_approach.z();
  ROS_INFO("Planning to final grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    current_pose.position.x, current_pose.position.y, current_pose.position.z, 
    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  // Use IK to find joint values for the target pose
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  
  bool found_ik = current_state->setFromIK(joint_model_group, current_pose, 10, 0.1);
  bool success = false;
  
  if (found_ik) {
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    ROS_INFO("Found IK solution for pre-grasp pose");
    
    // Use joint values target instead of pose target
    group.setJointValueTarget(joint_values);
    
    // Plan using joint space goal
    success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.result = success;
    
    ROS_INFO_NAMED("move_group_planner", "This plan (joint-space goal) %s", success ? "SUCCEEDED" : "FAILED");
    
    show_trail(success);
    
  } else {
    ROS_ERROR("Did not find IK solution for pre-grasp pose");
    success = false;
  }
  
  res.result = success;
  return success;
}


bool VADERPlanner::do_final_grasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res){
  // Execute the plan
  bool exec_result = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (exec_result) {
    ROS_INFO("Successfully moved to final grasp position");
  } else {
    ROS_ERROR("Failed to execute final grasp plan");
  }
  res.result = exec_result;
  return exec_result;
}

bool VADERPlanner::do_grasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
{
  // Use the stored cylinder pose and axis from pregrasp planning
  geometry_msgs::Pose cylinder_pose = stored_cylinder_pose;
  tf::Vector3 cylinder_axis = stored_cylinder_axis;
  
  // Get current robot state and transform for vader_gripper_base_link
  tf::StampedTransform gripper_transform;
  try {
    tf_listener.waitForTransform(group.getPlanningFrame(), "vader_gripper_base_link", 
                               ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform(group.getPlanningFrame(), "vader_gripper_base_link", 
                              ros::Time(0), gripper_transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("TF Exception: %s", ex.what());
    res.success = false;
    return false;
  }
  
  // Calculate the perpendicular vector to cylinder axis
  tf::Vector3 perpendicular_vector = calculatePerpendicularVector(cylinder_axis);
  
  // Calculate quaternion that aligns gripper's x-axis with perpendicular vector
  // and the approach direction (z-axis) with the vector from cylinder to gripper
  tf::Vector3 cylinder_position(cylinder_pose.position.x, 
                              cylinder_pose.position.y, 
                              cylinder_pose.position.z);
  tf::Vector3 gripper_position = gripper_transform.getOrigin();
  
  // Calculate vector from cylinder to gripper (this will be our approach vector)
  tf::Vector3 approach_vector = gripper_position - cylinder_position;
  approach_vector.normalize();
  
  // The x-axis of the gripper frame should be aligned with perpendicular_vector
  tf::Vector3 x_axis = perpendicular_vector;
  
  // The z-axis of the gripper frame should be aligned with approach_vector
  tf::Vector3 z_axis = approach_vector;
  
  // Calculate y-axis to complete the right-handed coordinate system
  tf::Vector3 y_axis = z_axis.cross(x_axis);
  y_axis.normalize();
  
  // Re-normalize x-axis to ensure orthogonality
  x_axis = y_axis.cross(z_axis);
  x_axis.normalize();
  
  // Create rotation matrix from three axis vectors
  tf::Matrix3x3 rotation_matrix(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z()
  );
  
  // Convert rotation matrix to quaternion
  tf::Quaternion grasp_quat;
  rotation_matrix.getRotation(grasp_quat);
  grasp_quat.normalize();
  
  // Calculate position offset from cylinder for grasp (slightly away from surface)
  double cylinder_radius = 0.075; // Same as in collision object
  double offset_distance = cylinder_radius + 0.02; // Offset for grasping
  
  // Calculate grasp position by moving offset_distance along approach_vector from cylinder surface
  tf::Vector3 offset_position = cylinder_position + (approach_vector * offset_distance);
  
  // Create the grasp pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.orientation.x = grasp_quat.x();
  grasp_pose.orientation.y = grasp_quat.y();
  grasp_pose.orientation.z = grasp_quat.z();
  grasp_pose.orientation.w = grasp_quat.w();
  grasp_pose.position.x = offset_position.x();
  grasp_pose.position.y = offset_position.y();
  grasp_pose.position.z = offset_position.z();
  
  ROS_INFO("Planning to grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, 
    grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
  
  // Calculate Cartesian path for approach
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(grasp_pose);
  
  // Approach along the vector joining cylinder centroid and gripper frame
  geometry_msgs::Pose final_grasp_pose = grasp_pose;
  final_grasp_pose.position.x = cylinder_position.x() + (approach_vector.x() * 0.01); // Closer to cylinder
  final_grasp_pose.position.y = cylinder_position.y() + (approach_vector.y() * 0.01);
  final_grasp_pose.position.z = cylinder_position.z() + (approach_vector.z() * 0.01);
  waypoints.push_back(final_grasp_pose);
  
  // Compute Cartesian path
  group.setMaxVelocityScalingFactor(maxV_scale_factor * 0.5); // Slower for grasp
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  bool success = true;
  if (fraction < 0.9) {
    // If Cartesian planning fails, try joint-space planning to the first waypoint
    ROS_WARN("Cartesian planning failed with coverage: %lf, trying joint-space planning", fraction);
    
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
    
    bool found_ik = current_state->setFromIK(joint_model_group, grasp_pose, 10, 0.1);
    
    if (found_ik) {
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(joint_model_group, joint_values);
      
      group.setJointValueTarget(joint_values);
      success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
      ROS_ERROR("Could not find IK solution for grasp pose");
      success = false;
    }
  } else {
    // Use the Cartesian trajectory directly
    my_xarm_plan.trajectory_ = trajectory;
    success = true;
  }
  
  ROS_INFO_NAMED("move_group_planner", "This grasp plan %s", success ? "SUCCEEDED" : "FAILED");
  show_trail(success);
  
  if (success) {
    // Execute the plan
    bool exec_result = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (exec_result) {
      ROS_INFO("Successfully moved to grasp position");
    } else {
      ROS_ERROR("Failed to execute grasp plan");
      success = false;
    }
  }
  
  res.success = success;
  return success;
}

bool VADERPlanner::do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res)
{
  // For Cartesian path, we'll compute the path but then convert to joint space
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(req.target);
  group.setMaxVelocityScalingFactor(maxV_scale_factor);
  moveit_msgs::RobotTrajectory trajectory;
 
  double fraction = group.computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
  bool success = true;
  
  if(fraction < 0.9) {
    success = false;
  } else {
    // Get the first waypoint and compute IK
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
    
    bool found_ik = current_state->setFromIK(joint_model_group, req.target, 10, 0.1);
    
    if (found_ik) {
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(joint_model_group, joint_values);
      
      // Use joint-space planning instead
      group.setJointValueTarget(joint_values);
      success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
      ROS_ERROR("Could not find IK solution for Cartesian target");
      success = false;
    }
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