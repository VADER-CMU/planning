/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h> //include for quaternion rotation
#include <tf/transform_listener.h>    //include for quaternion rotation
#include <tf/transform_datatypes.h>   //include for quaternion rotation
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// include <xarm_moveit_servo/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/robot_state/conversions.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometric_shapes/shapes.h> // Include for cylinder shape
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <vader_msgs/PlanningRequest.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <geometric_shapes/shape_operations.h>
#include "utils/utils.h"

#include <thread>
#include <future>
#include <iostream>
#include <optional>
#include <vector>
#include <queue>

#define SPINNER_THREAD_NUM 2

const double eef_step = 0.01;
const double cartesian_threshold = 0.9;

const double DEG2RAD = (M_PI / 180.0);

const double INV_KINEMATICS_SOLUTION_TIME_LIMIT_SEC = 0.1;
const double GRIPPER_GRASP_DIRECTION_OFFSET_RADIANS = M_PI/4;
const double CUTTER_GRASP_DIRECTION_OFFSET_RADIANS = -M_PI/4;

const double STORAGE_LOWER_Z_METER = 0.15;

const std::string GRIPPER_MOVE_GROUP = "L_xarm7";
const std::string CUTTER_MOVE_GROUP = "R_xarm7";
const std::vector<double> gripper_arm_home_joint_positions = {58.3 * DEG2RAD, -107.9 * DEG2RAD, -0.6 * DEG2RAD, 54 * DEG2RAD, 25.3 * DEG2RAD, 40.6 * DEG2RAD, 94.4* DEG2RAD};
const std::vector<double> cutter_arm_home_joint_positions = {-58.3 * DEG2RAD, -107.9 * DEG2RAD, 0.6 * DEG2RAD, 54 * DEG2RAD, -25.3 * DEG2RAD, 40.6 * DEG2RAD, 178.2 * DEG2RAD};

const std::vector<double> gripper_arm_storage_joint_positions = {80 * DEG2RAD, -39.9 * DEG2RAD, 11.2 * DEG2RAD, 31.4 * DEG2RAD, 0 * DEG2RAD, 68.3 * DEG2RAD, 84.7 * DEG2RAD};

namespace rvt = rviz_visual_tools;


static tf::Quaternion _get_norm_quat_from_axes(tf::Vector3 &ax_x, tf::Vector3 &ax_y, tf::Vector3 &ax_z)
{
    tf::Matrix3x3 rot_matrix;
    rot_matrix.setValue(
        ax_x.x(), ax_y.x(), ax_z.x(),
        ax_x.y(), ax_y.y(), ax_z.y(),
        ax_x.z(), ax_y.z(), ax_z.z());

    tf::Quaternion quat;
    rot_matrix.getRotation(quat);
    quat.normalize();
    return quat;
}

static geometry_msgs::Pose _get_pose_from_pos_and_quat(tf::Vector3 &pos, tf::Quaternion &quat)
{
    geometry_msgs::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();
    tf::quaternionTFToMsg(quat, pose.orientation);
    return pose;
}

geometry_msgs::Pose translateByLocalZ(const geometry_msgs::Pose& original_pose, double distance) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(original_pose.orientation, quat);
    tf::Vector3 local_z(0.0, 0.0, 1.0);
    tf::Vector3 world_offset = tf::quatRotate(quat, local_z) * distance;

    geometry_msgs::Pose new_pose = original_pose;
    new_pose.position.x += world_offset.x();
    new_pose.position.y += world_offset.y();
    new_pose.position.z += world_offset.z();

    return new_pose;
}
class VADERGripperPlanner {
public:
    VADERGripperPlanner(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_in)
        : move_group_(GRIPPER_MOVE_GROUP), visual_tools(visual_tools_in) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Gripper Planner");
        }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planGuidedCartesian(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();
        auto current_pose = move_group_.getCurrentPose().pose;

        double dist = std::sqrt(
            std::pow(target_pose.position.x - current_pose.position.x, 2) +
            std::pow(target_pose.position.y - current_pose.position.y, 2) +
            std::pow(target_pose.position.z - current_pose.position.z, 2)
        );

        int num_waypoints = static_cast<int>(dist / eef_step);

        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 2; i <= num_waypoints; ++i) {
            double ratio = static_cast<double>(i) / num_waypoints;
            geometry_msgs::Pose waypoint;
            waypoint.position.x = current_pose.position.x + ratio * (target_pose.position.x - current_pose.position.x);
            waypoint.position.y = current_pose.position.y + ratio * (target_pose.position.y - current_pose.position.y);
            waypoint.position.z = current_pose.position.z + ratio * (target_pose.position.z - current_pose.position.z);

            // Calculate intermediate quat
            tf::Quaternion current_quat;
            tf::quaternionMsgToTF(current_pose.orientation, current_quat);
            tf::Quaternion target_quat;
            tf::quaternionMsgToTF(target_pose.orientation, target_quat);
            tf::Quaternion intermediate_quat = current_quat.slerp(target_quat, ratio);
            tf::quaternionTFToMsg(intermediate_quat, waypoint.orientation);

            waypoints.push_back(waypoint);
        }

        move_group_.setMaxVelocityScalingFactor(0.2);
        move_group_.setMaxAccelerationScalingFactor(0.1);
        moveit_msgs::RobotTrajectory trajectory;

        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, trajectory);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        if (fraction >= cartesian_threshold) {
            // ROS_INFO_NAMED("vader_planner", "Gripper guided cartesian path computed successfully.");
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Gripper guided cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }

    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_WARN_NAMED("vader_planner", "Gripper plan computed.");
            return plan;
        }
        ROS_ERROR_NAMED("vader_planner", "Gripper planning failed.");
        return std::nullopt;
    }

    bool alreadyAtJointPositions(const std::vector<double>& joint_positions) {
        std::vector<double> current_joints = move_group_.getCurrentJointValues();

        // std::cout << "curr: ";
        // for(auto joint_val : current_joints) {
        //     std::cout << joint_val << ", ";
        // }
        // std::cout << "\ntgt: ";
        // for (auto joint_val : joint_positions) {
        //     std::cout << joint_val << ", ";
        // }
        // std::cout << "\n";  

        // Calculate norm between current and target joint positions
        double norm = 0.0;
        for (size_t i = 0; i < current_joints.size(); i++) {
            norm += std::pow(current_joints[i] - joint_positions[i], 2);
        }
        norm = std::sqrt(norm);

        // If norm is small, robot is already at target
        const double epsilon = 0.01; // radians
        if (norm < epsilon) {
            ROS_WARN_NAMED("vader_planner", "Robot already at target joint positions (norm: %f)", norm);
            return true;
        }
        // ROS_INFO_NAMED("vader_planner", "Robot not at target joint positions (norm: %f)", norm);
        return false;
    }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planToJointPositions(const std::vector<double>& joint_positions) {
        move_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_WARN_NAMED("vader_planner", "Gripper joint position plan computed.");
            return plan;
        }
        ROS_ERROR_NAMED("vader_planner", "Gripper joint position planning failed.");
        return std::nullopt;
    }

    // Execute synchronously
    bool execSync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // Execute asynchronously
    void execAsync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_.asyncExecute(plan);
    }

    bool testPoseIK(const geometry_msgs::Pose& target_pose) {
        moveit::core::RobotState state_copy = *move_group_.getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = state_copy.getJointModelGroup(GRIPPER_MOVE_GROUP);

        // Check if the pose is reachable
        bool found_ik = state_copy.setFromIK(joint_model_group, target_pose, INV_KINEMATICS_SOLUTION_TIME_LIMIT_SEC);
        return found_ik;
    }

    geometry_msgs::Pose getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    std::queue<geometry_msgs::Pose> generate_parametric_circle_poses(geometry_msgs::Pose &fruit_pose, double approach_dist, double angle_offset_manipulator, bool debug)
    {
        std::queue<geometry_msgs::Pose> pose_queue;

        tf::Quaternion fruit_quat;
        tf::quaternionMsgToTF(fruit_pose.orientation, fruit_quat);

        tf::Vector3 fruit_axis = tf::quatRotate(fruit_quat, tf::Vector3(0, 0, 1)).normalized();

        tf::Vector3 fruit_centroid(
            fruit_pose.position.x,
            fruit_pose.position.y,
            fruit_pose.position.z);

        tf::Vector3 u, v;

        tf::Vector3 ref(0, 0, 1);
        if (std::abs(fruit_axis.dot(ref)) > 0.9)
        {
            ref = tf::Vector3(1, 0, 0);
        }

        u = fruit_axis.cross(ref).normalized();
        v = fruit_axis.cross(u).normalized();

        double A = -fruit_centroid.dot(u);
        double B = -fruit_centroid.dot(v);

        double theta_min = atan2(B, A);
        if(debug){
            ROS_INFO("=== Parametric Circle Pose Queue ===");
        }

        std::vector<double> test_radii = {approach_dist}; //, 0.3, 0.2, 0.35, 0.15, 0.4};
        
        for (size_t r_idx = 0; r_idx < test_radii.size(); r_idx++)
        {
            double radius = test_radii[r_idx];
            
            std::vector<double> angle_offsets;
            if (r_idx == 0)
            {
                angle_offsets = {angle_offset_manipulator, angle_offset_manipulator + M_PI/6,  angle_offset_manipulator - M_PI/6,  angle_offset_manipulator + 2*M_PI/6,  angle_offset_manipulator - 2*M_PI/6}; //3*M_PI/6, 4*M_PI/6, 5*M_PI/6, 6*M_PI/6, 7*M_PI/6, 8*M_PI/6, 9*M_PI/6,;
            }
            else
            {
                angle_offsets = {angle_offset_manipulator};
            }
            
            for (double offset : angle_offsets)
            {
                double test_angle = theta_min + offset;
                
                tf::Vector3 test_point = fruit_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

                tf::Vector3 test_ee_z = (fruit_centroid - test_point).normalized();
                tf::Vector3 test_ee_y = fruit_axis.cross(test_ee_z).normalized();

                if (test_ee_y.length() < 0.1)
                {
                    tf::Vector3 world_up(0, 0, 1);
                    test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : world_up.cross(test_ee_z).normalized();
                }

                tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

                tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
                geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(test_point, test_quat);

                pose_queue.push(test_pose);
                if(debug){
                    ROS_INFO("Pose %zu: radius=%.3f, angle_offset=%.3f rad (%.1f deg)", 
                            pose_queue.size(), radius, offset, offset * 180.0 / M_PI);
                    ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                            test_pose.position.x, test_pose.position.y, test_pose.position.z);
                    ROS_INFO("  Orientation: x=%.3f,y=%.3f, z=%.3f, w=%.3f", 
                            test_pose.orientation.x, test_pose.orientation.y, test_pose.orientation.z, test_pose.orientation.w);
                    
                    // Publish labeled coordinate axes
                    visual_tools->publishAxisLabeled(test_pose, std::to_string(pose_queue.size()), rvt::SMALL);

                }
            }
        }
        visual_tools->trigger();
        if(debug){
            ROS_INFO("=== Total poses in queue: %zu ===", pose_queue.size());
        }
        return pose_queue;
    }


    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
};

class VADERCutterPlanner {
public:
    VADERCutterPlanner(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_in)
        : move_group_(CUTTER_MOVE_GROUP), visual_tools(visual_tools_in) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Cutter Planner");
        }

    bool alreadyAtJointPositions(const std::vector<double>& joint_positions) {
        std::vector<double> current_joints = move_group_.getCurrentJointValues();

        // Calculate norm between current and target joint positions
        double norm = 0.0;
        for (size_t i = 0; i < current_joints.size(); i++) {
            norm += std::pow(current_joints[i] - joint_positions[i], 2);
        }
        norm = std::sqrt(norm);

        // If norm is small, robot is already at target
        const double epsilon = 0.01; // radians
        if (norm < epsilon) {
            ROS_WARN_NAMED("vader_planner", "Robot already at target joint positions (norm: %f)", norm);
            return true;
        }
        // ROS_INFO_NAMED("vader_planner", "Robot not at target joint positions (norm: %f)", norm);
        return false;
    }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planGuidedCartesian(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();
        auto current_pose = move_group_.getCurrentPose().pose;

        double dist = std::sqrt(
            std::pow(target_pose.position.x - current_pose.position.x, 2) +
            std::pow(target_pose.position.y - current_pose.position.y, 2) +
            std::pow(target_pose.position.z - current_pose.position.z, 2)
        );

        int num_waypoints = static_cast<int>(dist / eef_step);

        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 2; i <= num_waypoints; ++i) {
            double ratio = static_cast<double>(i) / num_waypoints;
            geometry_msgs::Pose waypoint;
            waypoint.position.x = current_pose.position.x + ratio * (target_pose.position.x - current_pose.position.x);
            waypoint.position.y = current_pose.position.y + ratio * (target_pose.position.y - current_pose.position.y);
            waypoint.position.z = current_pose.position.z + ratio * (target_pose.position.z - current_pose.position.z);

            // Calculate intermediate quat
            tf::Quaternion current_quat;
            tf::quaternionMsgToTF(current_pose.orientation, current_quat);
            tf::Quaternion target_quat;
            tf::quaternionMsgToTF(target_pose.orientation, target_quat);
            tf::Quaternion intermediate_quat = current_quat.slerp(target_quat, ratio);
            tf::quaternionTFToMsg(intermediate_quat, waypoint.orientation);

            waypoints.push_back(waypoint);
        }

        move_group_.setMaxVelocityScalingFactor(0.2);
        move_group_.setMaxAccelerationScalingFactor(0.1);
        moveit_msgs::RobotTrajectory trajectory;
        
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, trajectory);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        if (fraction >= cartesian_threshold) {
            // ROS_INFO_NAMED("vader_planner", "Cutter guided cartesian path computed successfully.");
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Cutter guided cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planToJointPositions(const std::vector<double>& joint_positions) {
        move_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_WARN_NAMED("vader_planner", "Gripper joint position plan computed.");
            return plan;
        }
        ROS_ERROR_NAMED("vader_planner", "Gripper joint position planning failed.");
        return std::nullopt;
    }


    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_WARN_NAMED("vader_planner", "Cutter plan computed.");
            return plan;
        }
        ROS_ERROR_NAMED("vader_planner", "Cutter planning failed.");
        return std::nullopt;
    }

    // Execute synchronously
    bool execSync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // Execute asynchronously
    void execAsync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_.asyncExecute(plan);
    }
    
    bool testPoseIK(const geometry_msgs::Pose& target_pose) {
        moveit::core::RobotState state_copy = *move_group_.getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = state_copy.getJointModelGroup(CUTTER_MOVE_GROUP);

        // Check if the pose is reachable
        bool found_ik = state_copy.setFromIK(joint_model_group, target_pose, INV_KINEMATICS_SOLUTION_TIME_LIMIT_SEC);
        return found_ik;
    }

    geometry_msgs::Pose getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    std::queue<geometry_msgs::Pose> generate_parametric_circle_poses(geometry_msgs::Pose &fruit_pose, double approach_dist, double angle_offset_manipulator, bool debug)
    {
        std::queue<geometry_msgs::Pose> pose_queue;

        tf::Quaternion fruit_quat;
        tf::quaternionMsgToTF(fruit_pose.orientation, fruit_quat);

        tf::Vector3 fruit_axis = tf::quatRotate(fruit_quat, tf::Vector3(0, 0, 1)).normalized();

        tf::Vector3 fruit_centroid(
            fruit_pose.position.x,
            fruit_pose.position.y,
            fruit_pose.position.z);

        tf::Vector3 u, v;

        tf::Vector3 ref(0, 0, 1);
        if (std::abs(fruit_axis.dot(ref)) > 0.9)
        {
            ref = tf::Vector3(1, 0, 0);
        }

        u = fruit_axis.cross(ref).normalized();
        v = fruit_axis.cross(u).normalized();

        double A = -fruit_centroid.dot(u);
        double B = -fruit_centroid.dot(v);

        double theta_min = atan2(B, A);
        if(debug){
            ROS_INFO("=== Parametric Circle Pose Queue ===");
        }

        std::vector<double> test_radii = {approach_dist};//, 0.3, 0.2, 0.35, 0.15, 0.4};
        
        for (size_t r_idx = 0; r_idx < test_radii.size(); r_idx++)
        {
            double radius = test_radii[r_idx];
            
            std::vector<double> angle_offsets;
            if (r_idx == 0)
            {
                angle_offsets = {angle_offset_manipulator, angle_offset_manipulator + M_PI/6,  angle_offset_manipulator - M_PI/6,  angle_offset_manipulator + 2*M_PI/6,  angle_offset_manipulator- 2*M_PI/6}; //3*M_PI/6, 4*M_PI/6, 5*M_PI/6, 6*M_PI/6, 7*M_PI/6, 8*M_PI/6, 9*M_PI/6,};
            }
            else
            {
                angle_offsets = {angle_offset_manipulator};
            }
            
            for (double offset : angle_offsets)
            {
                double test_angle = theta_min + offset;
                
                tf::Vector3 test_point = fruit_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

                tf::Vector3 test_ee_z = (fruit_centroid - test_point).normalized();
                tf::Vector3 test_ee_y = fruit_axis.cross(test_ee_z).normalized();

                if (test_ee_y.length() < 0.1)
                {
                    tf::Vector3 world_up(0, 0, 1);
                    test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : world_up.cross(test_ee_z).normalized();
                }

                tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

                tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);

                // Create a rotation matrix for a 90-degree clockwise rotation around the z-axis
                tf::Matrix3x3 rotation_matrix(
                    0, 1, 0,  // First column
                    -1, 0, 0, // Second column
                    0, 0, 1   // Third column
                );

                // Apply the rotation matrix to the quaternion
                tf::Quaternion rotation_quat;
                rotation_matrix.getRotation(rotation_quat);
                test_quat = test_quat * rotation_quat;
                test_quat.normalize();

                geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(test_point, test_quat);

                pose_queue.push(test_pose);
                if(debug){
                    ROS_INFO("Pose %zu: radius=%.3f, angle_offset=%.3f rad (%.1f deg)", 
                            pose_queue.size(), radius, offset, offset * 180.0 / M_PI);
                    ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                            test_pose.position.x, test_pose.position.y, test_pose.position.z);
                    ROS_INFO("  Orientation: x=%.3f,y=%.3f, z=%.3f, w=%.3f", 
                            test_pose.orientation.x, test_pose.orientation.y, test_pose.orientation.z, test_pose.orientation.w);
                    
                // Publish labeled coordinate axes
                visual_tools->publishAxisLabeled(test_pose, std::to_string(pose_queue.size()), rvt::SMALL);
                }
            }
        }
        visual_tools->trigger();
        if(debug){
            ROS_INFO("=== Total poses in queue: %zu ===", pose_queue.size());
        }
        return pose_queue;
    }


    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools; //shared from planner server main instance
};

class VADERPlannerServer {
public:
    VADERPlannerServer()
        : spinner(SPINNER_THREAD_NUM),
          gripper_planner_(visual_tools),
          cutter_planner_(visual_tools) {

        // display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        ros::Duration(1.0).sleep(); // give publisher time to connect
        ROS_INFO_NAMED("vader_planner", "VADER Dual Planner Server Initialized");

        setupWorkspaceCollision();
        planning_service = node_handle.advertiseService("vader_planning_service", &VADERPlannerServer::planningServiceHandler, this);
    }

    void visualizePepper(const vader_msgs::Pepper pepper_estimate) {
        //Visualize as a visual tools marker
        std_msgs::ColorRGBA pepper_color;
        // green
        pepper_color.r = 0.0f;
        pepper_color.g = 1.0f;
        pepper_color.b = 0.0f;
        pepper_color.a = 0.8f; // semi-transparent
        visual_tools->publishCylinder(pepper_estimate.fruit_data.pose, pepper_color, pepper_estimate.fruit_data.shape.dimensions[0], pepper_estimate.fruit_data.shape.dimensions[1]*2);
        
        std_msgs::ColorRGBA peduncle_color;
        // brown
        peduncle_color.r = 0.6f;
        peduncle_color.g = 0.3f;
        peduncle_color.b = 0.0f;
        peduncle_color.a = 0.8f; // semi-transparent
        visual_tools->publishCylinder(pepper_estimate.peduncle_data.pose, peduncle_color, pepper_estimate.peduncle_data.shape.dimensions[0], pepper_estimate.peduncle_data.shape.dimensions[1]*2);
        visual_tools->trigger();
    }

    void setupWorkspaceCollision(){
        //Warthog body
        moveit_msgs::CollisionObject warthog_body;
        warthog_body.id = "warthog_body";
        warthog_body.header.frame_id = "world";

        shape_msgs::SolidPrimitive box_primitive;
        box_primitive.type = shape_msgs::SolidPrimitive::BOX;
        box_primitive.dimensions.resize(3);
        box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.8;
        box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
        box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.6;

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = -0.50;
        box_pose.position.y = 0.25;
        box_pose.position.z = 0.5;
        box_pose.orientation.w = 1.0;

        warthog_body.primitives.push_back(box_primitive);
        warthog_body.primitive_poses.push_back(box_pose);
        warthog_body.operation = warthog_body.ADD;

        // Define an orange bright color as std_msgs::ColorRGBA
        std_msgs::ColorRGBA orange_bright;
        orange_bright.r = 1.0f;
        orange_bright.g = 0.5f;
        orange_bright.b = 0.0f;
        orange_bright.a = 1.0f;

        planning_scene_interface.applyCollisionObject(warthog_body, orange_bright);

        // ----------------------------------------- Ground Plane -----------------------------------------
        moveit_msgs::CollisionObject ground_plane;
        ground_plane.header.frame_id = "world";
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
        ground_pose.position.x = 0.0;
        ground_pose.position.y = 0.0;
        ground_pose.position.z = 0.10 - (ground_primitive.dimensions[2] / 2.0);
        ground_pose.orientation.w = 1.0; // No rotation

        ground_plane.primitives.push_back(ground_primitive);
        ground_plane.primitive_poses.push_back(ground_pose);
        ground_plane.operation = moveit_msgs::CollisionObject::ADD;

        // Add the ground plane to the planning scene
        planning_scene_interface.applyCollisionObject(ground_plane);
    }

    void show_trails(const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& plan_gripper,
                     const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& plan_cutter)
    {

        if (plan_gripper.has_value()) {
            const robot_state::JointModelGroup *joint_model_group_gripper = gripper_planner_.move_group_.getCurrentState()->getJointModelGroup(GRIPPER_MOVE_GROUP);
            visual_tools->publishTrajectoryLine(plan_gripper->trajectory_, joint_model_group_gripper);
        }
        if (plan_cutter.has_value()) {
            const robot_state::JointModelGroup *joint_model_group_cutter = cutter_planner_.move_group_.getCurrentState()->getJointModelGroup(CUTTER_MOVE_GROUP);
            visual_tools->publishTrajectoryLine(plan_cutter->trajectory_, joint_model_group_cutter);
        }

        visual_tools->trigger();
    }


    void start() {
        spinner.start();
    }

    void stop() {
        spinner.stop();
    }

    int homeGripper(){
        bool alreadyThere = gripper_planner_.alreadyAtJointPositions(gripper_arm_home_joint_positions);
        if(alreadyThere) {
            ROS_WARN_NAMED("vader_planner", "Gripper already at home joint positions.");
            return vader_msgs::PlanningRequest::Response::SUCCESS;
        }

        auto plan = gripper_planner_.planToJointPositions(gripper_arm_home_joint_positions);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper home movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_RRT_TIMEOUT;
        }
        show_trails(plan, std::nullopt);
        bool success = gripper_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;
    }

    int moveGripperToStorage(){
        auto plan = gripper_planner_.planToJointPositions(gripper_arm_storage_joint_positions);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper home movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_RRT_TIMEOUT;
        }
        show_trails(plan, std::nullopt);
        bool success = gripper_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;
    }

    int homeCutter(){
        bool alreadyThere = cutter_planner_.alreadyAtJointPositions(cutter_arm_home_joint_positions);
        if(alreadyThere) {
            ROS_WARN_NAMED("vader_planner", "Cutter already at home joint positions.");
            return vader_msgs::PlanningRequest::Response::SUCCESS;
        }

        auto plan = cutter_planner_.planToJointPositions(cutter_arm_home_joint_positions);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter home movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_RRT_TIMEOUT;
        }
        show_trails(std::nullopt, plan);
        bool success = cutter_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_EXECUTE_FAILED;
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;
    }

    int gripperGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        // Approach pose is moved backward
        geometry_msgs::Pose approach_pose = translateByLocalZ(target_pose, -final_approach_dist);

        auto plan = gripper_planner_.planGuidedCartesian(approach_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper grasp movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(plan, std::nullopt);
        bool success = gripper_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }

        plan = gripper_planner_.planGuidedCartesian(target_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper final approach movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(plan, std::nullopt);
        success = gripper_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;

        // Move cartesian to actual target pose...
    }

    int cutterGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        auto gripper_current_pose = gripper_planner_.getCurrentPose();

        //TODO use this for debugging pose based on end effector pose
        // ROS_WARN_STREAM("Current gripper end effector pose: ("
        //                 << gripper_current_pose.position.x << ", "
        //                 << gripper_current_pose.position.y << ", "
        //                 << gripper_current_pose.position.z << "), quat (" 
        //                 << gripper_current_pose.orientation.x << ", "
        //                 << gripper_current_pose.orientation.y << ", "
        //                 << gripper_current_pose.orientation.z << ", "
        //                 << gripper_current_pose.orientation.w << ")");

        // ROS_WARN_STREAM("Cutter target pose: ("
        //                 << target_pose.position.x << ", "
        //                 << target_pose.position.y << ", "
        //                 << target_pose.position.z << "), quat (" 
        //                 << target_pose.orientation.x << ", "
        //                 << target_pose.orientation.y << ", "
        //                 << target_pose.orientation.z << ", "
        //                 << target_pose.orientation.w << ")");

        geometry_msgs::Pose approach_pose = translateByLocalZ(target_pose, -final_approach_dist);
        visual_tools->publishAxisLabeled(approach_pose, "Cutter Approach Pose", rvt::SMALL);
        visual_tools->trigger();

        auto plan = cutter_planner_.planGuidedCartesian(approach_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter grasp movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(std::nullopt, plan);
        bool success = cutter_planner_.execSync(plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_EXECUTE_FAILED;
        }

        // Move cartesian to actual target pose
        if(success) {
            auto cartesian_plan = cutter_planner_.planGuidedCartesian(target_pose);
            if(cartesian_plan == std::nullopt) {
                ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter final approach movement.");
                return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_CARTESIAN_FAILED;
            }
            show_trails(std::nullopt, cartesian_plan);
            success = cutter_planner_.execSync(cartesian_plan.value());
            if(!success) {
                return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_EXECUTE_FAILED;
            }
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;
    }

    int parallelMovePregrasp(geometry_msgs::Pose& gripper_target_pose, geometry_msgs::Pose& cutter_target_pose){
        bool success = true;
        // auto cutter_plan = cutter_planner_.planRRT(cutter_target_pose);
        // if(cutter_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter movement to pregrasp.");
        //     return false;
        // }
        // show_trails(std::nullopt, cutter_plan);
        // auto gripper_plan_future = std::async(&VADERGripperPlanner::planRRT, &gripper_planner_, gripper_target_pose);
        // std::thread cutter_exec_thread([&]() {
        //     cutter_planner_.execSync(cutter_plan.value());
        //     ROS_WARN_NAMED("vader_planner", "Cutter movement to pregrasp finished.");
        // });
        // auto gripper_plan = gripper_plan_future.get();
        // if(gripper_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to pregrasp.");
        //     success = false;
        // }
        // cutter_exec_thread.join();
        // show_trails(gripper_plan, std::nullopt);
        // success &= gripper_planner_.execSync(gripper_plan.value());

        // auto gripper_plan = gripper_planner_.planRRT(gripper_target_pose);
        // if(gripper_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to pregrasp.");
        //     return false;
        // }
        // show_trails(gripper_plan, std::nullopt);
        // auto cutter_plan_future = std::async(&VADERCutterPlanner::planRRT, &cutter_planner_, cutter_target_pose);
        // std::thread gripper_exec_thread([&]() {
        //     gripper_planner_.execSync(gripper_plan.value());
        //     ROS_WARN_NAMED("vader_planner", "Gripper movement to pregrasp finished.");
        // });
        // auto cutter_plan = cutter_plan_future.get();
        // if(cutter_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter movement to pregrasp.");
        //     success = false;
        // }
        // gripper_exec_thread.join();
        // show_trails(std::nullopt, cutter_plan);
        // success &= cutter_planner_.execSync(cutter_plan.value());

        // Rotate gripper_target_pose by 20 degrees around its own Y axis
        // tf::Quaternion quat;
        // tf::quaternionMsgToTF(gripper_target_pose.orientation, quat);

        // Create a quaternion representing a 20 degree rotation about Y axis
        // double angle_rad = 30.0 * M_PI / 180.0;
        // tf::Quaternion rot_y(tf::Vector3(0, 1, 0), angle_rad);

        // // Apply the rotation: rot_y * quat (local Y axis)
        // quat = rot_y * quat;
        // quat.normalize();
        // tf::quaternionTFToMsg(quat, gripper_target_pose.orientation);

        // gripper_target_pose.position.z += 0.03;

        auto gripper_plan = gripper_planner_.planGuidedCartesian(gripper_target_pose);

        if(gripper_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to pregrasp.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(gripper_plan, std::nullopt);
        success = gripper_planner_.execSync(gripper_plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }

        auto cutter_plan = cutter_planner_.planGuidedCartesian(cutter_target_pose);

        if(cutter_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter movement to pregrasp.");
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(std::nullopt, cutter_plan);
        success = cutter_planner_.execSync(cutter_plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_EXECUTE_FAILED;
        }
        return vader_msgs::PlanningRequest::Response::SUCCESS;
    }

    int parallelMoveStorage(){

        int returnCode = 0;
        // auto gripper_plan = gripper_planner_.planRRT(storageBinPose);
        // if(gripper_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to storage bin.");
        //     return false;
        // }
        // visual_tools->deleteAllMarkers();
        // show_trails(gripper_plan, std::nullopt);
        // auto cutter_plan_future = std::async(&VADERCutterPlanner::planRRT, &cutter_planner_, cutterHomePose);
        // std::thread gripper_exec_thread([&]() {
        //     gripper_planner_.execSync(gripper_plan.value());
        //     ROS_WARN_NAMED("vader_planner", "Gripper movement to storage bin finished.");
        // });
        // auto cutter_plan = cutter_plan_future.get();
        // if(cutter_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter home movement.");
        //     success = false;
        // }
        // gripper_exec_thread.join();
        // show_trails(std::nullopt, cutter_plan);
        // success &= cutter_planner_.execSync(cutter_plan.value());


        // auto gripper_plan = gripper_planner_.planRRT(storageBinPose);

        // gripper_plan = gripper_planner_.planRRT(storageBinPose);
        // auto gripper_plan = gripper_planner_.planToJointPositions(gripper_arm_home_joint_positions);
        // gripper_plan = gripper_planner_.planToJointPositions(gripper_arm_storage_joint_positions);
        // if(gripper_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to storage.");
        //     return false;
        // }
        // show_trails(gripper_plan, std::nullopt);
        // success &= gripper_planner_.execSync(gripper_plan.value());

        // auto gripper_plan = gripper_planner_.planGuidedCartesian(...); //TODO make it retract by same amount
        
        geometry_msgs::Pose cutter_retract_pose = translateByLocalZ(cutter_planner_.getCurrentPose(), -0.15);
        cutter_retract_pose.position.z += 0.10; // extra up for cutter
        geometry_msgs::Pose gripper_retract_pose = translateByLocalZ(gripper_planner_.getCurrentPose(), -0.15);

        auto cutter_retract_plan = cutter_planner_.planGuidedCartesian(cutter_retract_pose);
        if(cutter_retract_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter retract movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(std::nullopt, cutter_retract_plan);
        bool success = cutter_planner_.execSync(cutter_retract_plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_CUTTER_EXECUTE_FAILED;
        }
        removeSharedWorkspaceCollision();

        auto gripper_retract_plan = gripper_planner_.planGuidedCartesian(gripper_retract_pose);
        if(gripper_retract_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper retract movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(gripper_retract_plan, std::nullopt);
        success = gripper_planner_.execSync(gripper_retract_plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }

        returnCode = homeCutter();
        if (returnCode != vader_msgs::PlanningRequest::Response::SUCCESS) {
            return returnCode;
        }
        
        returnCode = moveGripperToStorage();
        if (returnCode != vader_msgs::PlanningRequest::Response::SUCCESS) {
            return returnCode;
        }

        // lower gripper to storage position
        geometry_msgs::Pose gripper_lowered_pose = gripper_planner_.getCurrentPose();
        gripper_lowered_pose.position.z -= STORAGE_LOWER_Z_METER;
        auto gripper_lower_plan = gripper_planner_.planGuidedCartesian(gripper_lowered_pose);
        if(gripper_lower_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper lowering movement.");
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_CARTESIAN_FAILED;
        }
        show_trails(gripper_lower_plan, std::nullopt);
        success = gripper_planner_.execSync(gripper_lower_plan.value());
        if(!success) {
            return vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_EXECUTE_FAILED;
        }

        return returnCode;
    }

    bool planningServiceHandler(vader_msgs::PlanningRequest::Request &req,
                                vader_msgs::PlanningRequest::Response &res) {
        switch(req.mode) {
            case vader_msgs::PlanningRequest::Request::HOME_CUTTER:{

                visual_tools->deleteAllMarkers();
                res.result = homeCutter();
                break;
            }
            case vader_msgs::PlanningRequest::Request::HOME_GRIPPER:{

                visual_tools->deleteAllMarkers();
                res.result = homeGripper();
                break;
            }
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_PREGRASP:
            {
                vader_msgs::Pepper pepper_estimate = req.pepper;
                visual_tools->deleteAllMarkers();
                visualizePepper(pepper_estimate);

                // ROS_INFO("Pepper estimate Position: x=%.3f, y=%.3f, z=%.3f", 
                //         pepper_estimate.fruit_data.pose.position.x, pepper_estimate.fruit_data.pose.position.y, pepper_estimate.fruit_data.pose.position.z);

                
                pepper_estimate.fruit_data.pose.position.z += 0.10; //lift pregrasp to help camera see peduncle
                pepper_estimate.peduncle_data.pose.position.z += 0.10; //Same for cutter cam

                bool debug_PC_output = false;
                auto gripper_target_poses = gripper_planner_.generate_parametric_circle_poses(pepper_estimate.fruit_data.pose, 0.25, GRIPPER_GRASP_DIRECTION_OFFSET_RADIANS, debug_PC_output);
                auto cutter_target_poses = cutter_planner_.generate_parametric_circle_poses(pepper_estimate.peduncle_data.pose, 0.25, CUTTER_GRASP_DIRECTION_OFFSET_RADIANS, debug_PC_output);
                visual_tools->trigger();

                double dynamic_y = 0;
                if (pepper_estimate.fruit_data.pose.position.y - 0.07 < 0.1) {
                    dynamic_y = 0.1;
                }
                else if (pepper_estimate.fruit_data.pose.position.y - 0.07 > 0.4) {
                    dynamic_y = 0.4;
                }
                else {
                    dynamic_y = pepper_estimate.fruit_data.pose.position.y - 0.07;
                }
                setUpSharedWorkspaceCollision(dynamic_y, 0.55);

                // Test IK for each pose in the queue until we find one that is valid
                bool found_valid_poses = false;
                geometry_msgs::Pose gripper_pregrasp_pose;
                while(!gripper_target_poses.empty())
                {
                    geometry_msgs::Pose gripper_test_pose = gripper_target_poses.front();
                    gripper_target_poses.pop();

                    bool gripper_ik_valid = gripper_planner_.testPoseIK(gripper_test_pose);

                    if (gripper_ik_valid)
                    {
                        // ROS_INFO("Found valid IK pose for gripper pregrasp.");
                        gripper_pregrasp_pose = gripper_test_pose;
                        // visual_tools->deleteAllMarkers();
                        visual_tools->publishAxisLabeled(gripper_test_pose, "Gripper Pregrasp", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for gripper pregrasp.");
                    res.result = vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_NO_IK_SOLUTION_FOUND;
                    res.success = false;
                    ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
                    return res.success;
                }
                found_valid_poses = false;
                geometry_msgs::Pose cutter_pregrasp_pose;
                while(!cutter_target_poses.empty())
                {
                    geometry_msgs::Pose cutter_test_pose = cutter_target_poses.front();
                    cutter_target_poses.pop();

                    bool cutter_ik_valid = cutter_planner_.testPoseIK(cutter_test_pose);

                    if (cutter_ik_valid)
                    {
                        // ROS_INFO("Found valid IK pose for cutter pregrasp.");
                        cutter_pregrasp_pose = cutter_test_pose;
                        visual_tools->publishAxisLabeled(cutter_test_pose, "Cutter Pregrasp", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for cutter pregrasp.");
                    res.result = vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_NO_IK_SOLUTION_FOUND;
                    res.success = false;
                    ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
                    return res.success;
                }
                visual_tools->trigger();
                
                res.result = parallelMovePregrasp(gripper_pregrasp_pose, cutter_pregrasp_pose);
                break;
            }
            case vader_msgs::PlanningRequest::Request::GRIPPER_GRASP:{
                // TODO here, calculate desired pose based off of pepper estimate
                vader_msgs::Pepper pepper_estimate = req.pepper;
                visual_tools->deleteAllMarkers();
                visualizePepper(pepper_estimate);
                visual_tools->trigger();
                double final_approach_dist = 0.10;

                // ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                //         pepper_estimate.fruit_data.pose.position.x, pepper_estimate.fruit_data.pose.position.y, pepper_estimate.fruit_data.pose.position.z);
                bool debug_PC_output = false;
                auto gripper_target_poses = gripper_planner_.generate_parametric_circle_poses(pepper_estimate.fruit_data.pose, final_approach_dist, GRIPPER_GRASP_DIRECTION_OFFSET_RADIANS, debug_PC_output);
                // Test IK for each pose in the queue until we find one that is valid
                bool found_valid_poses = false;
                geometry_msgs::Pose gripper_pregrasp_pose;
                while(!gripper_target_poses.empty())
                {
                    geometry_msgs::Pose gripper_test_pose = gripper_target_poses.front();
                    gripper_target_poses.pop();

                    bool gripper_ik_valid = gripper_planner_.testPoseIK(gripper_test_pose);

                    //See if we can reach the final grasp pose at all (IK)

                    geometry_msgs::Pose gripper_final_pose = translateByLocalZ(gripper_test_pose, 0.17);
                    gripper_final_pose.position.z += 0.01;

                    bool reachable = gripper_planner_.testPoseIK(gripper_final_pose);
                    if (gripper_ik_valid && !reachable) {
                        ROS_WARN_NAMED("vader_planner", "waypoint valid, but final grasp unreachable!");
                        continue;
                    }
                    if (gripper_ik_valid)
                    {
                        gripper_pregrasp_pose = gripper_test_pose;
                        visual_tools->publishAxisLabeled(gripper_test_pose, "Gripper Grasp Pose", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for gripper pregrasp.");
                    res.result = vader_msgs::PlanningRequest::Response::FAIL_GRIPPER_PLAN_NO_IK_SOLUTION_FOUND;
                    res.success = false;
                    ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
                    return res.success;
                }
                visual_tools->trigger();

                //Cutter reachable?
                auto cutter_target_poses = cutter_planner_.generate_parametric_circle_poses(pepper_estimate.peduncle_data.pose, final_approach_dist, CUTTER_GRASP_DIRECTION_OFFSET_RADIANS, debug_PC_output);
                // Test IK for each pose in the queue until we find one that is valid
                found_valid_poses = false;
                geometry_msgs::Pose cutter_pregrasp_pose;
                while(!cutter_target_poses.empty())
                {
                    geometry_msgs::Pose cutter_test_pose = cutter_target_poses.front();
                    cutter_target_poses.pop();

                    bool cutter_ik_valid = cutter_planner_.testPoseIK(cutter_test_pose);

                    //See if we can reach the final grasp pose at all (IK)
                    // Translate the target pose along its local -Z axis by final_approach_dist
                    geometry_msgs::Pose cutter_final_pose = translateByLocalZ(cutter_test_pose, 0.08);
                    cutter_final_pose.position.z += 0.07; // extra offset to account for cutter length
                    
                    bool reachable = gripper_planner_.testPoseIK(cutter_final_pose);
                    if (cutter_ik_valid && !reachable) {
                        ROS_WARN_NAMED("vader_planner", "waypoint valid, but final grasp unreachable!");
                        continue;
                    }
                    if (cutter_ik_valid)
                    {
                        // ROS_INFO("Found valid IK pose for cutter pregrasp.");
                        cutter_pregrasp_pose = cutter_test_pose;
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for cutter pregrasp.");
                    res.result = vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_NO_IK_SOLUTION_FOUND;
                    res.success = false;
                    ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
                    return res.success;
                }
                visual_tools->trigger();

                geometry_msgs::Pose gripper_final_pose = translateByLocalZ(gripper_pregrasp_pose, 0.17);
                gripper_final_pose.position.z += 0.01;

                res.result = gripperGrasp(gripper_final_pose, final_approach_dist);
                break;
            }
            case vader_msgs::PlanningRequest::Request::CUTTER_GRASP:{
                // TODO here, calculate desired pose based off of pepper estimate
                vader_msgs::Pepper pepper_estimate = req.pepper;
                visualizePepper(pepper_estimate);
                visual_tools->trigger();
                double final_approach_dist = 0.1;

                // ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f",
                //         pepper_estimate.peduncle_data.pose.position.x, pepper_estimate.peduncle_data.pose.position.y, pepper_estimate.peduncle_data.pose.position.z);
                bool debug_PC_output = false;
                auto cutter_target_poses = cutter_planner_.generate_parametric_circle_poses(pepper_estimate.peduncle_data.pose, final_approach_dist, CUTTER_GRASP_DIRECTION_OFFSET_RADIANS, debug_PC_output);
                // Test IK for each pose in the queue until we find one that is valid
                bool found_valid_poses = false;
                geometry_msgs::Pose cutter_pregrasp_pose;
                while(!cutter_target_poses.empty())
                {
                    geometry_msgs::Pose cutter_test_pose = cutter_target_poses.front();
                    cutter_target_poses.pop();

                    bool cutter_ik_valid = cutter_planner_.testPoseIK(cutter_test_pose);

                    //See if we can reach the final grasp pose at all (IK)
                    // Translate the target pose along its local -Z axis by final_approach_dist
                    geometry_msgs::Pose cutter_final_pose = translateByLocalZ(cutter_test_pose, 0.08);
                    cutter_final_pose.position.z += 0.07; // extra offset to account for cutter length
                    
                    bool reachable = gripper_planner_.testPoseIK(cutter_final_pose);
                    if (cutter_ik_valid && !reachable) {
                        ROS_WARN_NAMED("vader_planner", "waypoint valid, but final grasp unreachable!");
                        continue;
                    }
                    if (cutter_ik_valid)
                    {
                        // ROS_INFO("Found valid IK pose for cutter pregrasp.");
                        cutter_pregrasp_pose = cutter_test_pose;
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for cutter pregrasp.");
                    res.result = vader_msgs::PlanningRequest::Response::FAIL_CUTTER_PLAN_NO_IK_SOLUTION_FOUND;
                    res.success = false;
                    ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
                    return res.success;
                }
                visual_tools->trigger();

                // Translate the target pose along its local -Z axis by final_approach_dist
                cutter_pregrasp_pose = translateByLocalZ(cutter_pregrasp_pose, 0.08);
                cutter_pregrasp_pose.position.z += 0.07; // extra offset to account for cutter length
                visual_tools->publishAxisLabeled(cutter_pregrasp_pose, "Cutter Grasp Pose", rvt::MEDIUM);

                // Plan a Cartesian approach to the target pose
                res.result = cutterGrasp(cutter_pregrasp_pose, final_approach_dist);
                break;
            }
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_STORAGE:{
                visual_tools->deleteAllMarkers();
                res.result = parallelMoveStorage();
                break;
            }
            default:{
                ROS_ERROR_NAMED("vader_planner", "Unknown planning mode requested: %d", req.mode);
                res.result = vader_msgs::PlanningRequest::Response::FAIL_UNKNOWN;
                break;
            }
        }
        res.success = (res.result == vader_msgs::PlanningRequest::Response::SUCCESS);
        ROS_WARN_NAMED("vader_planner", "Planning service completed with result code: %d", res.result);
        return res.success;
    }

    void setUpSharedWorkspaceCollision(double divide_workspace_y, double x_depth) {
        moveit_msgs::CollisionObject workspace_divide_wall;
        workspace_divide_wall.header.frame_id = "world";
        workspace_divide_wall.id = "workspace_divide_wall";

        shape_msgs::SolidPrimitive wall_primitive = makeBoxPrimitive(x_depth, 0.01, 1.0);
        geometry_msgs::Pose wall_pose = makePose(x_depth / 2.0, divide_workspace_y, 0.5, QUAT_IDENTITY());

        workspace_divide_wall.primitives.push_back(wall_primitive);
        workspace_divide_wall.primitive_poses.push_back(wall_pose);
        workspace_divide_wall.operation = workspace_divide_wall.ADD;

        // Add the single thin wall to the planning scene (no ACM / allowed entries changes)
        planning_scene_interface.applyCollisionObject(workspace_divide_wall, COLOR_BLUE_TRANSLUCENT());

        // ROS_WARN_NAMED("vader_planner", "Shared workspace collision set up.");
    }

    void removeSharedWorkspaceCollision(){
        planning_scene_interface.removeCollisionObjects({"workspace_divide_wall"});
        // ROS_WARN_NAMED("vader_planner", "Shared workspace collision removed.");
    }

private: 
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"); //shared from planner server main instance
    // ros::Publisher display_path;
    
    ros::NodeHandle node_handle;
    ros::ServiceServer planning_service;

    planning_scene_monitor::PlanningSceneMonitorPtr psm;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    VADERGripperPlanner gripper_planner_;
    VADERCutterPlanner cutter_planner_;
    ros::AsyncSpinner spinner;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_dual_planner");

    VADERPlannerServer plannerServer;

    plannerServer.start();

    ros::waitForShutdown();
    return 0;
}
