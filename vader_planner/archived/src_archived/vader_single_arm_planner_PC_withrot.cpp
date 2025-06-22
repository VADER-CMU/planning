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

#include <tf/transform_broadcaster.h> //include for quaternion rotation
#include <tf/transform_listener.h>    //include for quaternion rotation
#include <tf/transform_datatypes.h>   //include for quaternion rotation
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// include <xarm_moveit_servo/kinematic_constraints/utils.h>

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
  VADERPlanner(const std::string plan_group_name) : spinner(SPINNER_THREAD_NUM), group(plan_group_name) { init(); };
  VADERPlanner() : spinner(SPINNER_THREAD_NUM), group(PLANNING_GROUP) { init(); };
  ~VADERPlanner() { delete visual_tools; };
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
  ros::Subscriber exec_plan_sub;    /* non-blocking*/
  ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

  void init();
  bool do_pregrasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res);
  bool do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res);
  bool do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res);
  bool exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res);
  void execute_plan_topic(const std_msgs::Bool::ConstPtr &exec);
  void show_trail(bool plan_result);
};

void VADERPlanner::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("xarm_pose_plan", &VADERPlanner::do_pregrasp_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &VADERPlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &VADERPlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &VADERPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &VADERPlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  moveit_msgs::CollisionObject ground_plane;
  ground_plane.header.frame_id = group.getPlanningFrame();
  ground_plane.id = "ground_plane";

  // Define the cuboid dimensions
  shape_msgs::SolidPrimitive ground_primitive;
  ground_primitive.type = ground_primitive.BOX;
  ground_primitive.dimensions.resize(3);
  ground_primitive.dimensions[0] = 2.0;  // Length in x-direction
  ground_primitive.dimensions[1] = 2.0;  // Width in y-direction
  ground_primitive.dimensions[2] = 0.01; // Height in z-direction

  // Define the pose of the cuboid
  geometry_msgs::Pose ground_pose;
  ground_pose.position.x = -1.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = -0.01 - (ground_primitive.dimensions[2] / 2.0);
  ground_pose.orientation.w = 1.0; // No rotation

  ground_plane.primitives.push_back(ground_primitive);
  ground_plane.primitive_poses.push_back(ground_pose);
  ground_plane.operation = moveit_msgs::CollisionObject::ADD;

  // Add the ground plane to the planning scene
  planning_scene_interface.applyCollisionObject(ground_plane);
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
  if (plan_result)
  {
    ROS_INFO_NAMED("vader_planner", "Visualizing plan as trajectory line");

    visual_tools->deleteAllMarkers();
    const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools->publishTrajectoryLine(my_xarm_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
  }
}

// bool VADERPlanner::do_pregrasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
// {
//   // Save original target pose (cylinder pose)
//   geometry_msgs::Pose cylinder_pose;
//   cylinder_pose.position = req.target.position;
//   cylinder_pose.orientation = req.target.orientation;

//   // Convert quaternion to tf format
//   tf::Quaternion cylinder_quat;
//   tf::quaternionMsgToTF(cylinder_pose.orientation, cylinder_quat);

//   // Get cylinder centroid position
//   tf::Vector3 cylinder_centroid(
//     cylinder_pose.position.x,
//     cylinder_pose.position.y,
//     cylinder_pose.position.z
//   );

//   // Calculate cylinder axis (z-axis of the cylinder)
//   tf::Vector3 cylinder_axis = tf::quatRotate(cylinder_quat, tf::Vector3(0, 0, 1)).normalized();

//   // Define radius for the parametric circle
//   double radius = 0.25; // Offset distance from cylinder centroid

//   // Generate two orthonormal vectors u,v that are perpendicular to cylinder_axis
//   tf::Vector3 u, v;

//   // Find first perpendicular vector u
//   tf::Vector3 ref(0, 0, 1);
//   if (std::abs(cylinder_axis.dot(ref)) > 0.9) {
//     // If cylinder axis is nearly vertical, use a different reference
//     ref = tf::Vector3(1, 0, 0);
//   }

//   u = cylinder_axis.cross(ref).normalized();
//   v = cylinder_axis.cross(u).normalized();

//   // Calculate the values A and B
//   double A = -cylinder_centroid.dot(u);  // Note the negative signs as per the formula
//   double B = -cylinder_centroid.dot(v);

//   // Calculate theta_min to find the closest point to origin
//   double theta_min = atan2(B, A);

//   // Calculate the closest point on the parametric circle
//   tf::Vector3 closest_point = cylinder_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);

//   // Log calculated values for debugging
//   ROS_INFO("Cylinder centroid: (%f, %f, %f)",
//            cylinder_centroid.x(), cylinder_centroid.y(), cylinder_centroid.z());
//   ROS_INFO("Parametric circle calculation: A=%f, B=%f, theta_min=%f",
//            A, B, theta_min);
//   ROS_INFO("Closest point on circle: (%f, %f, %f)",
//            closest_point.x(), closest_point.y(), closest_point.z());

//   // Create cylinder collision object for visualization
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.CYLINDER;
//   primitive.dimensions.resize(2);
//   primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
//   primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.075;

//   moveit_msgs::CollisionObject cylinder_object;
//   cylinder_object.header.frame_id = group.getPlanningFrame();
//   cylinder_object.id = "cylinder_1";
//   cylinder_object.header.frame_id = "link_base";
//   cylinder_object.primitives.push_back(primitive);
//   cylinder_object.primitive_poses.push_back(cylinder_pose);
//   cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
//   planning_scene_interface.applyCollisionObject(cylinder_object);

//   // Calculate end effector orientation
//   // The gripper z-axis should point from the goal point to the cylinder centroid
//   tf::Vector3 ee_z = (cylinder_centroid - closest_point).normalized();

//   // Ensure end effector y-axis is perpendicular to cylinder axis
//   tf::Vector3 ee_y = cylinder_axis.cross(ee_z).normalized();

//   // Calculate end effector x-axis to complete right-handed coordinate system
//   tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

//   // Create rotation matrix for end effector orientation
//   tf::Matrix3x3 ee_rotation;
//   ee_rotation.setValue(
//     ee_x.x(), ee_y.x(), ee_z.x(),
//     ee_x.y(), ee_y.y(), ee_z.y(),
//     ee_x.z(), ee_y.z(), ee_z.z()
//   );

//   // Convert rotation matrix to quaternion
//   tf::Quaternion ee_quat;
//   ee_rotation.getRotation(ee_quat);
//   ee_quat.normalize();

//   // Set end effector pose
//   geometry_msgs::Pose end_effector_pose;
//   end_effector_pose.position.x = closest_point.x();
//   end_effector_pose.position.y = closest_point.y();
//   end_effector_pose.position.z = closest_point.z();
//   tf::quaternionTFToMsg(ee_quat, end_effector_pose.orientation);

//   // Log end effector pose
//   ROS_INFO("End effector position: (%f, %f, %f)",
//            end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
//   ROS_INFO("End effector orientation: (%f, %f, %f, %f)",
//            end_effector_pose.orientation.x, end_effector_pose.orientation.y,
//            end_effector_pose.orientation.z, end_effector_pose.orientation.w);

//   // Set target for planning
//   group.setPoseTarget(end_effector_pose);

//   // Try to find IK solution with robust error handling
//   moveit::core::RobotStatePtr current_state = group.getCurrentState();
//   const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

//   // First try with the optimal position calculated
//   bool found_ik = current_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
//   bool success = false;

//   // If optimal position fails, try variations on the parametric circle
//   if (!found_ik) {
//     ROS_WARN("Initial IK solution failed, trying alternative points on the parametric circle");

//     // Try different angles around the parametric circle in 30-degree increments
//     for (int i = 1; i <= 11 && !found_ik; i++) {
//       // Try i*30 degrees away from the optimal angle
//       double test_angle = theta_min + i * (M_PI/6);

//       // Calculate test point
//       tf::Vector3 test_point = cylinder_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

//       // Calculate orientation (gripper z-axis still points at cylinder centroid)
//       tf::Vector3 test_ee_z = (cylinder_centroid - test_point).normalized();
//       tf::Vector3 test_ee_y = cylinder_axis.cross(test_ee_z).normalized();

//       // Check if the cross product result is valid (non-zero length)
//       if (test_ee_y.length() < 0.1) {
//         // If ee_z is nearly parallel to cylinder_axis, use different approach
//         tf::Vector3 world_up(0, 0, 1);
//         test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ?
//           tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() :
//           world_up.cross(test_ee_z).normalized();
//       }

//       tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

//       // Create rotation matrix and convert to quaternion
//       tf::Matrix3x3 test_rotation;
//       test_rotation.setValue(
//         test_ee_x.x(), test_ee_y.x(), test_ee_z.x(),
//         test_ee_x.y(), test_ee_y.y(), test_ee_z.y(),
//         test_ee_x.z(), test_ee_y.z(), test_ee_z.z()
//       );

//       tf::Quaternion test_quat;
//       test_rotation.getRotation(test_quat);
//       test_quat.normalize();

//       // Set up test pose
//       geometry_msgs::Pose test_pose;
//       test_pose.position.x = test_point.x();
//       test_pose.position.y = test_point.y();
//       test_pose.position.z = test_point.z();
//       tf::quaternionTFToMsg(test_quat, test_pose.orientation);

//       // Try IK for this pose
//       found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

//       if (found_ik) {
//         ROS_INFO("Found IK solution at alternative angle: theta = %.2f radians (%.2f degrees from optimal)",
//           test_angle, i * 30.0);
//         end_effector_pose = test_pose;
//         group.setPoseTarget(end_effector_pose);
//         break;
//       }
//     }

//     // If circle positions fail, try with adjusted radius
//     if (!found_ik) {
//       ROS_WARN("Circle position IK solutions failed, trying with adjusted radius");

//       // Try different radii
//       std::vector<double> test_radii = {0.3, 0.2, 0.35, 0.15, 0.4};

//       for (const auto& test_radius : test_radii) {
//         // Try the optimal angle first with new radius
//         tf::Vector3 test_point = cylinder_centroid + test_radius * (cos(theta_min) * u + sin(theta_min) * v);

//         // Calculate orientation
//         tf::Vector3 test_ee_z = (cylinder_centroid - test_point).normalized();
//         tf::Vector3 test_ee_y = cylinder_axis.cross(test_ee_z).normalized();
//         tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

//         // Create rotation matrix
//         tf::Matrix3x3 test_rotation;
//         test_rotation.setValue(
//           test_ee_x.x(), test_ee_y.x(), test_ee_z.x(),
//           test_ee_x.y(), test_ee_y.y(), test_ee_z.y(),
//           test_ee_x.z(), test_ee_y.z(), test_ee_z.z()
//         );

//         tf::Quaternion test_quat;
//         test_rotation.getRotation(test_quat);
//         test_quat.normalize();

//         // Set up test pose
//         geometry_msgs::Pose test_pose;
//         test_pose.position.x = test_point.x();
//         test_pose.position.y = test_point.y();
//         test_pose.position.z = test_point.z();
//         tf::quaternionTFToMsg(test_quat, test_pose.orientation);

//         // Try IK for this pose
//         found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

//         if (found_ik) {
//           ROS_INFO("Found IK solution with adjusted radius: %.2f m", test_radius);
//           end_effector_pose = test_pose;
//           group.setPoseTarget(end_effector_pose);
//           break;
//         }
//       }
//     }
//   }

//   // Plan motion if IK was found
//   if (found_ik) {
//     std::vector<double> joint_values;
//     current_state->copyJointGroupPositions(joint_model_group, joint_values);
//     ROS_INFO("Found valid IK solution");

//     group.setJointValueTarget(joint_values);
//     success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     res.success = success;
//     ROS_INFO_NAMED("move_group_planner", "Plan %s", success ? "SUCCEEDED" : "FAILED");

//     show_trail(success);
//   } else {
//     ROS_ERROR("Could not find IK solution for any tested pose");
//     success = false;
//   }

//   return success;
// }

bool VADERPlanner::do_pregrasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
{
  // Save original target pose (cylinder pose)
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.position = req.target.position;
  cylinder_pose.orientation = req.target.orientation;

  // Convert quaternion to tf format
  tf::Quaternion cylinder_quat;
  tf::quaternionMsgToTF(cylinder_pose.orientation, cylinder_quat);

  // Get cylinder centroid position
  tf::Vector3 cylinder_centroid(
      cylinder_pose.position.x,
      cylinder_pose.position.y,
      cylinder_pose.position.z);

  // Calculate cylinder axis (z-axis of the cylinder)
  tf::Vector3 cylinder_axis = tf::quatRotate(cylinder_quat, tf::Vector3(0, 0, 1)).normalized();

  // Define radius for the parametric circle
  double radius = 0.25; // Offset distance from cylinder centroid

  // Generate two orthonormal vectors u,v that are perpendicular to cylinder_axis
  tf::Vector3 u, v;

  // Find first perpendicular vector u
  tf::Vector3 ref(0, 0, 1);
  if (std::abs(cylinder_axis.dot(ref)) > 0.9)
  {
    // If cylinder axis is nearly vertical, use a different reference
    ref = tf::Vector3(1, 0, 0);
  }

  u = cylinder_axis.cross(ref).normalized();
  v = cylinder_axis.cross(u).normalized();

  // Calculate the values A and B
  double A = -cylinder_centroid.dot(u); // Note the negative signs as per the formula
  double B = -cylinder_centroid.dot(v);

  // Calculate theta_min to find the closest point to origin
  double theta_min = atan2(B, A);

  // Calculate the closest point on the parametric circle
  tf::Vector3 closest_point = cylinder_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);

  // Log calculated values for debugging
  ROS_INFO("Cylinder centroid: (%f, %f, %f)",
           cylinder_centroid.x(), cylinder_centroid.y(), cylinder_centroid.z());
  ROS_INFO("Parametric circle calculation: A=%f, B=%f, theta_min=%f",
           A, B, theta_min);
  ROS_INFO("Closest point on circle: (%f, %f, %f)",
           closest_point.x(), closest_point.y(), closest_point.z());

  // Create cylinder collision object for visualization
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
  primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.075;

  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.header.frame_id = group.getPlanningFrame();
  cylinder_object.id = "cylinder_1";
  cylinder_object.header.frame_id = "link_base";
  cylinder_object.primitives.push_back(primitive);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface.applyCollisionObject(cylinder_object);

  // Calculate end effector orientation
  // The gripper z-axis should point from the goal point to the cylinder centroid
  tf::Vector3 ee_z = (cylinder_centroid - closest_point).normalized();

  // Ensure end effector y-axis is perpendicular to cylinder axis
  tf::Vector3 ee_y = cylinder_axis.cross(ee_z).normalized();

  // Calculate end effector x-axis to complete right-handed coordinate system
  tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

  // Create rotation matrix for end effector orientation
  tf::Matrix3x3 ee_rotation;
  ee_rotation.setValue(
      ee_x.x(), ee_y.x(), ee_z.x(),
      ee_x.y(), ee_y.y(), ee_z.y(),
      ee_x.z(), ee_y.z(), ee_z.z());

  // Convert rotation matrix to quaternion
  tf::Quaternion ee_quat;
  ee_rotation.getRotation(ee_quat);
  ee_quat.normalize();

  // Set end effector pose
  geometry_msgs::Pose end_effector_pose;
  end_effector_pose.position.x = closest_point.x();
  end_effector_pose.position.y = closest_point.y();
  end_effector_pose.position.z = closest_point.z();
  tf::quaternionTFToMsg(ee_quat, end_effector_pose.orientation);

  // Log end effector pose
  ROS_INFO("End effector position: (%f, %f, %f)",
           end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
  ROS_INFO("End effector orientation: (%f, %f, %f, %f)",
           end_effector_pose.orientation.x, end_effector_pose.orientation.y,
           end_effector_pose.orientation.z, end_effector_pose.orientation.w);

  // Set target for planning
  group.setPoseTarget(end_effector_pose);

  // Try to find IK solution with robust error handling
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

  // First try with the optimal position calculated
  bool found_ik = current_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
  bool success = false;

  // If optimal position fails, try variations on the parametric circle
  if (!found_ik)
  {
    ROS_WARN("Initial IK solution failed, trying alternative points on the parametric circle");

    // Try different angles around the parametric circle in 30-degree increments
    for (int i = 1; i <= 11 && !found_ik; i++)
    {
      // Try i*30 degrees away from the optimal angle
      double test_angle = theta_min + i * (M_PI / 6);

      // Calculate test point
      tf::Vector3 test_point = cylinder_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

      // Calculate orientation (gripper z-axis still points at cylinder centroid)
      tf::Vector3 test_ee_z = (cylinder_centroid - test_point).normalized();
      tf::Vector3 test_ee_y = cylinder_axis.cross(test_ee_z).normalized();

      // Check if the cross product result is valid (non-zero length)
      if (test_ee_y.length() < 0.1)
      {
        // If ee_z is nearly parallel to cylinder_axis, use different approach
        tf::Vector3 world_up(0, 0, 1);
        test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : world_up.cross(test_ee_z).normalized();
      }

      tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

      // Create rotation matrix and convert to quaternion
      tf::Matrix3x3 test_rotation;
      test_rotation.setValue(
          test_ee_x.x(), test_ee_y.x(), test_ee_z.x(),
          test_ee_x.y(), test_ee_y.y(), test_ee_z.y(),
          test_ee_x.z(), test_ee_y.z(), test_ee_z.z());

      tf::Quaternion test_quat;
      test_rotation.getRotation(test_quat);
      test_quat.normalize();

      // Set up test pose
      geometry_msgs::Pose test_pose;
      test_pose.position.x = test_point.x();
      test_pose.position.y = test_point.y();
      test_pose.position.z = test_point.z();
      tf::quaternionTFToMsg(test_quat, test_pose.orientation);

      // Try IK for this pose
      found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

      if (found_ik)
      {
        ROS_INFO("Found IK solution at alternative angle: theta = %.2f radians (%.2f degrees from optimal)",
                 test_angle, i * 30.0);
        end_effector_pose = test_pose;
        group.setPoseTarget(end_effector_pose);
        break;
      }
    }

    // If circle positions fail, try with adjusted radius
    if (!found_ik)
    {
      ROS_WARN("Circle position IK solutions failed, trying with adjusted radius");

      // Try different radii
      std::vector<double> test_radii = {0.3, 0.2, 0.35, 0.15, 0.4};

      for (const auto &test_radius : test_radii)
      {
        // Try the optimal angle first with new radius
        tf::Vector3 test_point = cylinder_centroid + test_radius * (cos(theta_min) * u + sin(theta_min) * v);

        // Calculate orientation
        tf::Vector3 test_ee_z = (cylinder_centroid - test_point).normalized();
        tf::Vector3 test_ee_y = cylinder_axis.cross(test_ee_z).normalized();
        tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

        // Create rotation matrix
        tf::Matrix3x3 test_rotation;
        test_rotation.setValue(
            test_ee_x.x(), test_ee_y.x(), test_ee_z.x(),
            test_ee_x.y(), test_ee_y.y(), test_ee_z.y(),
            test_ee_x.z(), test_ee_y.z(), test_ee_z.z());

        tf::Quaternion test_quat;
        test_rotation.getRotation(test_quat);
        test_quat.normalize();

        // Set up test pose
        geometry_msgs::Pose test_pose;
        test_pose.position.x = test_point.x();
        test_pose.position.y = test_point.y();
        test_pose.position.z = test_point.z();
        tf::quaternionTFToMsg(test_quat, test_pose.orientation);

        // Try IK for this pose
        found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

        if (found_ik)
        {
          ROS_INFO("Found IK solution with adjusted radius: %.2f m", test_radius);
          end_effector_pose = test_pose;
          group.setPoseTarget(end_effector_pose);
          break;
        }
      }
    }
  }

  // Plan motion if IK was found
  if (found_ik)
  {
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO("Found valid IK solution");

    group.setJointValueTarget(joint_values);
    success = (group.plan(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      ROS_INFO_NAMED("move_group_planner", "Pregrasp pose plan SUCCEEDED");

      // Execute the planned motion to reach the pregrasp pose
      // This assumes your planner class has a function to execute the planned trajectory
      // If not, you'll need to add code to execute the plan before proceeding
      bool execution_success = group.execute(my_xarm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

      if (execution_success)
      {
        ROS_INFO("Successfully reached pregrasp pose, now planning 90-degree rotation of final joint");

        // Get the current robot state after reaching the pregrasp pose
        moveit::core::RobotStatePtr new_state = group.getCurrentState();

        // Get the current joint values
        std::vector<double> final_joint_values;
        new_state->copyJointGroupPositions(joint_model_group, final_joint_values);

        // Get the number of joints in the group
        int num_joints = final_joint_values.size();

        if (num_joints > 0)
        {
          // Rotate only the final joint by 90 degrees
          final_joint_values[num_joints - 1] += M_PI / 2.0;

          // Normalize the joint angle to the range (-PI, PI) if needed
          while (final_joint_values[num_joints - 1] > M_PI)
          {
            final_joint_values[num_joints - 1] -= 2.0 * M_PI;
          }
          while (final_joint_values[num_joints - 1] < -M_PI)
          {
            final_joint_values[num_joints - 1] += 2.0 * M_PI;
          }

          ROS_INFO("Final joint before rotation: %f, after rotation: %f radians",
                   final_joint_values[num_joints - 1] - M_PI / 2.0, final_joint_values[num_joints - 1]);

          // Plan the joint rotation motion
          group.setJointValueTarget(final_joint_values);

          // Create a new plan for the rotation
          moveit::planning_interface::MoveGroupInterface::Plan rotation_plan;
          bool rotation_success = (group.plan(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

          if (rotation_success)
          {
            ROS_INFO("Successfully planned final joint rotation");

            // Execute the rotation
            bool rotation_execution = group.execute(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

            if (rotation_execution)
            {
              ROS_INFO("Successfully executed final joint rotation");
              // Final success depends on both motions succeeding
              success = true;
            }
            else
            {
              ROS_ERROR("Failed to execute final joint rotation");
              success = false;
            }
          }
          else
          {
            ROS_ERROR("Failed to plan final joint rotation");
            success = false;
          }
        }
        else
        {
          ROS_WARN("No joints found in group, cannot rotate final joint");
          // Still consider the pregrasp pose successful even if we couldn't rotate
          success = true;
        }
      }
      else
      {
        ROS_ERROR("Failed to execute pregrasp pose plan");
        success = false;
      }
    }
    else
    {
      ROS_INFO_NAMED("move_group_planner", "Pregrasp pose plan FAILED");
      success = false;
    }

    res.success = success;
    show_trail(success);
  }
  else
  {
    ROS_ERROR("Could not find IK solution for any tested pose");
    success = false;
    res.success = success;
  }

  return success;
}

bool VADERPlanner::do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res)
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(req.target);
  group.setMaxVelocityScalingFactor(maxV_scale_factor);
  moveit_msgs::RobotTrajectory trajectory;

  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  bool success = true;
  if (fraction < 0.9)
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
  if (!group.setJointValueTarget(req.target))
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
  if (req.exec)
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
void VADERPlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr &exec)
{
  if (exec->data)
  {
    ROS_INFO("Received Execution Command !!!!!");
    group.asyncExecute(my_xarm_plan); /* return without waiting for finish */
  }
}

std::string VADERPlanner::PLANNING_GROUP; // Definition of static class member

int main(int argc, char **argv)
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
