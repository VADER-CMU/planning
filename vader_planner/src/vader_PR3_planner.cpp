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

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>

#include <vader_msgs/SingleArmPlanRequest.h>
#include <vader_msgs/SingleArmExecutionRequest.h>

#include <tf/transform_broadcaster.h>//include for quaternion rotation 
#include <tf/transform_listener.h>//include for quaternion rotation 
#include <tf/transform_datatypes.h>//include for quaternion rotation 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


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
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    bool do_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool exec_plan_cb(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);
    void show_trail(bool plan_result);
};

void VADERPlanner::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("singleArmPlan", &VADERPlanner::do_pose_plan, this);
  exec_plan_srv = node_handle.advertiseService("singleArmExec", &VADERPlanner::exec_plan_cb, this);

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

bool VADERPlanner::do_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{ 
  //snippet for quaternion transformation
  tf::Quaternion orig_quat;
  tf::quaternionMsgToTF(req.pepper.fruit_data.pose.orientation, orig_quat);

  // tf2_ros::Buffer tf_buffer;
  // tf2_ros::TransformListener tf_listener(tf_buffer); 
  
  geometry_msgs::Pose orig_pose;
  orig_pose.orientation.x = req.pepper.fruit_data.pose.orientation.x;    
  orig_pose.orientation.y = req.pepper.fruit_data.pose.orientation.y;
  orig_pose.orientation.z = req.pepper.fruit_data.pose.orientation.z;
  orig_pose.orientation.w = req.pepper.fruit_data.pose.orientation.w;
  orig_pose.position.x = req.pepper.fruit_data.pose.position.x;
  orig_pose.position.y = req.pepper.fruit_data.pose.position.y;
  orig_pose.position.z = req.pepper.fruit_data.pose.position.z;

  tf::Quaternion rotation_quat;
  rotation_quat.setRPY(0, M_PI/2, 0);

  tf::Quaternion rotated_quat = rotation_quat * orig_quat;
  
  rotated_quat.normalize();
  tf::quaternionTFToMsg(rotated_quat, req.pepper.fruit_data.pose.orientation);

  ROS_INFO("Quaternion: old=%f, new=%f,rot_quat=%f", orig_quat, rotated_quat, M_PI);
// end

  // Convert quaternion to rotation matrix
  tf::Matrix3x3 rotation_matrix(rotated_quat);
  
  // Define offset in local Z direction (0.5m along local frame Z-axis)
  tf::Vector3 offset(0.0, 0.0, -0.1);
  
  // Transform the offset using the rotation matrix
  tf::Vector3 transformed_offset = rotation_matrix * offset;

  // Apply transformed offset to position
  req.pepper.fruit_data.pose.position.x += transformed_offset.x();
  req.pepper.fruit_data.pose.position.y += transformed_offset.y();
  req.pepper.fruit_data.pose.position.z += transformed_offset.z();

  group.setPoseTarget(req.pepper.fruit_data.pose);

  tf::Vector3 x_axis(1, 0, 0);
  tf::Vector3 rotated_x_axis = tf::quatRotate(orig_quat, x_axis);
  
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "cylinder_1";
  cylinder_object.header.frame_id = "link_base";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = req.pepper.fruit_data.shape.dimensions[0]; // Height - to be changed based on input
  primitive.dimensions[primitive.CYLINDER_RADIUS] = req.pepper.fruit_data.shape.dimensions[1]; // Radius - to be changed based on input

  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.x = orig_pose.orientation.x;
  cylinder_pose.orientation.y = orig_pose.orientation.y;
  cylinder_pose.orientation.z = orig_pose.orientation.z;
  cylinder_pose.orientation.w = orig_pose.orientation.w;
  cylinder_pose.position.x = orig_pose.position.x;
  cylinder_pose.position.y = orig_pose.position.y;
  cylinder_pose.position.z = orig_pose.position.z;

  cylinder_object.primitives.push_back(primitive);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = moveit_msgs::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObject(cylinder_object);

  moveit_msgs::CollisionObject box_object;
  box_object.id = "box_1";
  box_object.header.frame_id = "link_base";

  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[box_primitive.BOX_X] = 0.3; // Length
  box_primitive.dimensions[box_primitive.BOX_Y] = 0.3; // Width
  box_primitive.dimensions[box_primitive.BOX_Z] = 0.56; // Height

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = cylinder_pose.position.x;
  box_pose.position.y = cylinder_pose.position.y;
  box_pose.position.z = cylinder_pose.position.z - (req.pepper.fruit_data.shape.dimensions[0] / 2) - 0.28; // Half the height of the box
  box_object.primitives.push_back(box_primitive);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = moveit_msgs::CollisionObject::ADD;

  // Add the box to the planning scene
  planning_scene_interface.applyCollisionObject(box_object);

  ROS_INFO("Added box: x=%f, y=%f, z=%f", box_pose.position.x, box_pose.position.y, box_pose.position.z);

  ROS_INFO("Added cylinder: x=%f, y=%f, z=%f", cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
  ROS_INFO("vader_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
    req.pepper.fruit_data.pose.position.x, req.pepper.fruit_data.pose.position.y, req.pepper.fruit_data.pose.position.z, req.pepper.fruit_data.pose.orientation.x, \
    req.pepper.fruit_data.pose.orientation.y, req.pepper.fruit_data.pose.orientation.z, req.pepper.fruit_data.pose.orientation.w);

  bool success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.result = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
 
  show_trail(success);

  //cylinder_object.operation = moveit_msgs::CollisionObject::REMOVE; // Remove the cylinder
  //planning_scene_interface.applyCollisionObject(cylinder_object); 

  return success;
}

bool VADERPlanner::exec_plan_cb(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res)
{
  ROS_INFO("Received Execution Service Request");
  bool finish_ok = (group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
  res.result = finish_ok;
  return finish_ok;
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


