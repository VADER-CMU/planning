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

#include <tf/transform_broadcaster.h>//include for quaternion rotation 
#include <tf/transform_listener.h>//include for quaternion rotation 
#include <tf/transform_datatypes.h>//include for quaternion rotation 
#include <tf/tf.h>

#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <iostream>
 
 #define SPINNER_THREAD_NUM 2
 
 /* Used for Cartesian path computation, please modify as needed: */
 const double jump_threshold = 0.0;
 const double eef_step = 0.005;
 const double maxV_scale_factor = 0.3;
 
 const std::string PLAN_GROUP_L_PREFIX = "L_xarm7";
 const std::string PLAN_GROUP_R_PREFIX = "R_xarm7";

 const std::string PEPPER_SUBSCRIBE_TOPIC = "target_pepper_pose";
 
 namespace rvt = rviz_visual_tools;
 
 class VADERPR2Planner
 {
   public:
     VADERPR2Planner():
        spinner(SPINNER_THREAD_NUM),
        PLANNING_GROUP_L(PLAN_GROUP_L_PREFIX),
        PLANNING_GROUP_R(PLAN_GROUP_R_PREFIX),
        group_L(PLAN_GROUP_L_PREFIX),
        group_R(PLAN_GROUP_R_PREFIX)
        {
          init();
        };
     ~VADERPR2Planner(){ delete visual_tools;};
     void do_pepper_plan(const vader_msgs::Pepper::ConstPtr& msg);
     bool gripper_plan_and_execute(const vader_msgs::Fruit& fruit_pose);
     bool cutter_plan_and_execute(const vader_msgs::Peduncle& peduncle_pose);
     void start();
     void stop();
 
   private:
     ros::NodeHandle node_handle;
     ros::AsyncSpinner spinner;
     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //  std::vector<std::string> joint_names;
     moveit::planning_interface::MoveGroupInterface group_L, group_R;
     moveit::planning_interface::MoveGroupInterface::Plan plan_L, plan_R;
     moveit_visual_tools::MoveItVisualTools *visual_tools;

    std::string PLANNING_GROUP_L;
    std::string PLANNING_GROUP_R;
 
     ros::Publisher display_path;
     ros::Subscriber pepper_sub_;

     void init();

     void show_trail(bool plan_result);
 };
 
 void VADERPR2Planner::init()
 {
    display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/
  
    visual_tools = new moveit_visual_tools::MoveItVisualTools("L_link_base");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.8;

    visual_tools->publishText(text_pose, "VADER PR2 Planner Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools->trigger();
    // std::cout << "VaderPlanner waiting for messages" << std::endl;
    // Initialize subscriber and publisher
    pepper_sub_ = node_handle.subscribe(PEPPER_SUBSCRIBE_TOPIC, 10, &VADERPR2Planner::do_pepper_plan, this);
    ROS_INFO("Planner initialized with left planning group: %s and right planning group: %s", \
      PLANNING_GROUP_L.c_str(), PLANNING_GROUP_R.c_str());
  }
 
 void VADERPR2Planner::start()
 {
   ROS_INFO("Spinning");
   spinner.start();
 }
 
 void VADERPR2Planner::stop()
 {
   spinner.stop();
 }
 
 void VADERPR2Planner::show_trail(bool plan_result)
 {
   if(plan_result)
   {
     ROS_INFO_NAMED("xarm_planner", "Visualizing plan as trajectory line");
     
     visual_tools->deleteAllMarkers();
     const robot_state::JointModelGroup* joint_model_group_L = group_L.getCurrentState()->getJointModelGroup(PLANNING_GROUP_L);
     visual_tools->publishTrajectoryLine(plan_L.trajectory_, joint_model_group_L);
     const robot_state::JointModelGroup* joint_model_group_R = group_R.getCurrentState()->getJointModelGroup(PLANNING_GROUP_R);
     visual_tools->publishTrajectoryLine(plan_R.trajectory_, joint_model_group_R);
     visual_tools->trigger();
   }
 }

 //Rotates given pose by 90 deg around the y axis
geometry_msgs::Pose calculateFruitPregraspPose(const geometry_msgs::Pose& target_pose) {
  // Assuming target_pose contains the necessary information to calculate the pregrasp pose
  tf::Quaternion pregrasp_pose;

  tf::Quaternion rotation_quat;
  rotation_quat.setRPY( -M_PI/2,0, 0);

  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(target_pose.orientation, original_quat);

  pregrasp_pose = rotation_quat * original_quat;
  pregrasp_pose.normalize();

  geometry_msgs::Pose pregrasp_pose_result;
  tf::quaternionTFToMsg(pregrasp_pose, pregrasp_pose_result.orientation);
  
  pregrasp_pose_result.position.x = target_pose.position.x;// + 0.1; // Adjust as needed
  pregrasp_pose_result.position.y = target_pose.position.y;// + 0.1; // Adjust as needed
  pregrasp_pose_result.position.z = target_pose.position.z;// + 0.1; // Adjust as needed
  return pregrasp_pose_result;
 }
 
geometry_msgs::Pose calculatePedunclePregraspPose(const geometry_msgs::Pose& target_pose) {
  // Assuming target_pose contains the necessary information to calculate the pregrasp pose
  tf::Quaternion pregrasp_pose;

  tf::Quaternion rotation_quat;
  rotation_quat.setRPY( M_PI/2,0, 0);

  tf::Quaternion original_quat;
  tf::quaternionMsgToTF(target_pose.orientation, original_quat);

  pregrasp_pose = rotation_quat * original_quat;
  pregrasp_pose.normalize();

  geometry_msgs::Pose pregrasp_pose_result;
  tf::quaternionTFToMsg(pregrasp_pose, pregrasp_pose_result.orientation);
  
  pregrasp_pose_result.position.x = target_pose.position.x;// + 0.1; // Adjust as needed
  pregrasp_pose_result.position.y = target_pose.position.y;// + 0.1; // Adjust as needed
  pregrasp_pose_result.position.z = target_pose.position.z;// + 0.1; // Adjust as needed
  return pregrasp_pose_result;
 }

 void VADERPR2Planner::do_pepper_plan(const vader_msgs::Pepper::ConstPtr& msg) {
    // std::cout << "Hello" << std::endl;  
    ROS_INFO_NAMED("do_pepper_plan", "Received new pepper planning request");

    vader_msgs::Fruit fruit = msg->fruit_data;
    vader_msgs::Peduncle peduncle = msg->peduncle_data;
    //Make it a collision object
    moveit_msgs::CollisionObject fruit_collision_obj;
    fruit_collision_obj.id = "fruit_1";
    fruit_collision_obj.header.frame_id = "L_link_base";
    fruit_collision_obj.primitives.push_back(fruit.shape);
    fruit_collision_obj.primitive_poses.push_back(fruit.pose);
    fruit_collision_obj.operation = fruit_collision_obj.ADD;
    planning_scene_interface.applyCollisionObject(fruit_collision_obj);
    //Make it a collision object
    moveit_msgs::CollisionObject peduncle_collision_obj;
    peduncle_collision_obj.id = "peduncle_1";
    peduncle_collision_obj.header.frame_id = "L_link_base";
    peduncle_collision_obj.primitives.push_back(peduncle.shape);
    peduncle_collision_obj.primitive_poses.push_back(peduncle.pose);
    peduncle_collision_obj.operation = peduncle_collision_obj.ADD;
    planning_scene_interface.applyCollisionObject(peduncle_collision_obj);

    // Do gripper movement
    bool gripper_success = VADERPR2Planner::gripper_plan_and_execute(fruit);
    // bool gripper_success = true;
    ROS_INFO_NAMED("do_pepper_plan", "Gripper planning and execution completed");

    // Do cutter movement
    
    bool cutter_success = VADERPR2Planner::cutter_plan_and_execute(peduncle);

    // Conclude
    ROS_INFO_NAMED("do_pepper_plan", "Dual arm execution completed");
    ROS_INFO_NAMED("do_pepper_plan", "Gripper success: %s, Cutter success: %s", \
      gripper_success ? "true" : "false", cutter_success ? "true" : "false");
    // std::cout << "Done" << std::endl;
 }

 bool VADERPR2Planner::gripper_plan_and_execute(const vader_msgs::Fruit& fruit) {
    ROS_INFO_NAMED("gripper_plan_and_execute", "Received new gripper planning request");
    geometry_msgs::Pose target_pose = calculateFruitPregraspPose(fruit.pose);
    target_pose.position.x = target_pose.position.x;// fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];
    target_pose.position.y = target_pose.position.y - fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];
    target_pose.position.z = target_pose.position.z;// + fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];

    group_L.setPoseTarget(target_pose);
    
    bool success = (group_L.plan(plan_L) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ROS_ERROR_NAMED("gripper_plan_and_execute", "Failed to plan for gripper");
      return false;
    }
    show_trail(success);
    bool exec_ok = (group_L.execute(plan_L) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return exec_ok;
 }

 bool VADERPR2Planner::cutter_plan_and_execute(const vader_msgs::Peduncle& peduncle) {
    ROS_INFO("Received new cutter planning request");
    geometry_msgs::Pose target_pose = calculatePedunclePregraspPose(peduncle.pose);
    target_pose.position.x = target_pose.position.x;//peduncle.shape.dimensions[peduncle.shape.CYLINDER_RADIUS];
    target_pose.position.y = target_pose.position.y + peduncle.shape.dimensions[peduncle.shape.CYLINDER_RADIUS];// + 0.05;
    target_pose.position.z = target_pose.position.z + peduncle.shape.dimensions[peduncle.shape.CYLINDER_HEIGHT]/ 3;

    // TODO: Abhi: Add collision object for CUTTER ARM

    group_R.setPoseTarget(target_pose);
    
    bool success = (group_R.plan(plan_R) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ROS_ERROR_NAMED("gripper_plan_and_execute", "Failed to plan for gripper");
      return false;
    }
    show_trail(success);
    bool exec_ok = (group_R.execute(plan_R) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return exec_ok;
 }

 
 
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "vader_PR2_planner");
  //  ros::NodeHandle nh;
 
   VADERPR2Planner planner;
 
   planner.start();
  //  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");
 
   ros::waitForShutdown();
   return 0;
 }
 
 