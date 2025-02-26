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
     VADERPR2Planner():spinner(SPINNER_THREAD_NUM){init();};
     ~VADERPR2Planner(){ delete visual_tools;};
     void do_pepper_plan(const vader_msgs::Pepper::ConstPtr& msg);
     bool gripper_plan_and_execute(const vader_msgs::Fruit& fruit_pose);
     bool cutter_plan_and_execute(const vader_msgs::Peduncle& peduncle_pose);
     void start();
     void stop();
 
   private:
     ros::NodeHandle node_handle;
     ros::AsyncSpinner spinner;
    //  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //  std::vector<std::string> joint_names;
    //  moveit::planning_interface::MoveGroupInterface group;
    //  moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan;
     moveit_visual_tools::MoveItVisualTools *visual_tools;
    //  std::string left_or_right_arm;
    //  std::string PLANNING_GROUP;
 
     ros::Publisher display_path;
     ros::Subscriber pepper_sub_;

     void init();

     void show_trail(bool plan_result);
 };
 
 void VADERPR2Planner::init()
 {
    display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/
  
    // visual_tools->publishText(text_pose, PLANNING_GROUP + "_xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
    // visual_tools->trigger();
    // std::cout << "VaderPlanner waiting for messages" << std::endl;
    // Initialize subscriber and publisher
    pepper_sub_ = node_handle.subscribe(PEPPER_SUBSCRIBE_TOPIC, 10, &VADERPR2Planner::do_pepper_plan, this);

    visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.8;
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
    //  ROS_INFO_NAMED("xarm_planner", "Visualizing plan as trajectory line");
     
    //  visual_tools->deleteAllMarkers();
    //  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //  visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
    //  visual_tools->trigger();
   }
 }


 geometry_msgs::Pose calculateFruitPOI(const vader_msgs::Pepper::ConstPtr& msg) {
  // Assuming msg contains the necessary information to calculate the POI
  geometry_msgs::Pose pose_msg;
  // Fill in pose_msg based on msg TODO Rohit
  return pose_msg;
}

 geometry_msgs::Pose calculatePedunclePOI(const vader_msgs::Pepper::ConstPtr& msg) {
  // Assuming msg contains the necessary information to calculate the POI
  geometry_msgs::Pose pose_msg;
  // Fill in pose_msg based on msg TODO Rohit
  return pose_msg;
}

 void VADERPR2Planner::do_pepper_plan(const vader_msgs::Pepper::ConstPtr& msg) {
  // std::cout << "Hello" << std::endl;  
  ROS_INFO_NAMED("do_pepper_plan", "Received new pepper planning request");

    // Do gripper movement
    vader_msgs::Fruit fruit = msg->fruit_data;
    bool gripper_success = VADERPR2Planner::gripper_plan_and_execute(fruit);

    ROS_INFO_NAMED("do_pepper_plan", "Gripper planning and execution completed");

    // Do cutter movement
    vader_msgs::Peduncle peduncle = msg->peduncle_data;
    bool cutter_success = VADERPR2Planner::cutter_plan_and_execute(peduncle);

    // Conclude
    ROS_INFO_NAMED("do_pepper_plan", "Dual arm execution completed");
    ROS_INFO_NAMED("do_pepper_plan", "Gripper success: %s, Cutter success: %s", \
      gripper_success ? "true" : "false", cutter_success ? "true" : "false");
    // std::cout << "Done" << std::endl;
 }

 bool VADERPR2Planner::gripper_plan_and_execute(const vader_msgs::Fruit& fruit_pose) {
    ROS_INFO_NAMED("gripper_plan_and_execute", "Received new gripper planning request");
    // TODO: Abhi: Add collision object for CUTTER ARM
    //plan path to pose
    //execute path
    return true; //success
 }

 bool VADERPR2Planner::cutter_plan_and_execute(const vader_msgs::Peduncle& peduncle_pose) {
    ROS_INFO("Received new cutter planning request");
    // TODO: Abhi: Add collision object for GRIPPER ARM
    //plan path to pose
    //execute path
    return true; //success
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
 
 