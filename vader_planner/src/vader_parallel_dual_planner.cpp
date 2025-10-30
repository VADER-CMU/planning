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

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.01;//0.005;
const double maxV_scale_factor = 0.3;
const double cartesian_threshold = 0.9;

const std::string GRIPPER_MOVE_GROUP = "L_xarm7";
const std::string CUTTER_MOVE_GROUP = "R_xarm7";

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

class VADERGripperPlanner {
public:
    VADERGripperPlanner(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_in)
        : move_group_(GRIPPER_MOVE_GROUP), visual_tools(visual_tools_in) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Gripper Planner");
        }

    // Plan a Cartesian path
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planCartesian(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();
        
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);
        move_group_.setMaxVelocityScalingFactor(maxV_scale_factor);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction >= cartesian_threshold) {
            ROS_INFO_NAMED("vader_planner", "Gripper cartesian path computed successfully.");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Gripper cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planGuidedCartesian(const geometry_msgs::Pose& target_pose) {
        auto current_pose = move_group_.getCurrentPose().pose;

        double dist = std::sqrt(
            std::pow(target_pose.position.x - current_pose.position.x, 2) +
            std::pow(target_pose.position.y - current_pose.position.y, 2) +
            std::pow(target_pose.position.z - current_pose.position.z, 2)
        );

        int num_waypoints = static_cast<int>(dist / eef_step);

        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 0; i <= num_waypoints; ++i) {
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
        double fraction = 0.0;
        
        while(fraction < cartesian_threshold) {
            ROS_INFO_NAMED("vader_planner", "Retry with fraction: %f", fraction);
            fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        }
        for(size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i){
            trajectory_msgs::JointTrajectoryPoint& point = trajectory.joint_trajectory.points[i];
            if(i % 5 == 0){
                std::cout << point.time_from_start << ", " ;
            }
            point.time_from_start = ros::Duration(i*0.05);
        }

        // robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), GRIPPER_MOVE_GROUP);
        // rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory); // computed_trajectory from computeCartesianPath()
        // trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // bool iptp_success = iptp.computeTimeStamps(rt);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        // if(iptp_success) {
        //     rt.getRobotTrajectoryMsg(trajectory);
        //     plan.trajectory_ = trajectory;
        // } else {
        //     // Handle error
        //     ROS_WARN_NAMED("vader_planner", "Gripper: IPTP procedure error");
        // }

        // if (trajectory.points.size() >= 2)
        // {
        //     if ((trajectory.points.end() - 1)->time_from_start == (trajectory.points.end() - 2)->time_from_start)
        //     {
        //     ROS_WARN("last 2 waypoints time from start are equal");

        //     auto& point = trajectory.points[trajectory.points.size() - 1];
        //     point.time_from_start = point.time_from_start + ros::Duration(1e-3);
        //     }
        // }


        if (fraction >= cartesian_threshold) {
            ROS_INFO_NAMED("vader_planner", "Gripper guided cartesian path computed successfully.");
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Gripper guided cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }

    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();

        auto curr_pose = move_group_.getCurrentPose().pose;

        ROS_WARN_NAMED("vader_planner", "current pose: position=(%f, %f, %f) quat=(%f, %f, %f, %f)",
                        curr_pose.position.x, curr_pose.position.y, curr_pose.position.z,
                        curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w);
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ROS_WARN_NAMED("vader_planner", "Planning for gripper...");

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_WARN_NAMED("vader_planner", "Gripper plan computed.");
            return plan;
        }
        ROS_ERROR_NAMED("vader_planner", "Gripper planning failed.");
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
        bool found_ik = state_copy.setFromIK(joint_model_group, target_pose, 10, 0.1);
        return found_ik;
    }

    std::queue<geometry_msgs::Pose> generate_parametric_circle_poses(geometry_msgs::Pose &fruit_pose, double approach_dist, double angle_offset_manipulator)
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

        ROS_INFO("=== Parametric Circle Pose Queue ===");

        std::vector<double> test_radii = {approach_dist, 0.3, 0.2, 0.35, 0.15, 0.4};
        
        for (size_t r_idx = 0; r_idx < test_radii.size(); r_idx++)
        {
            double radius = test_radii[r_idx];
            
            std::vector<double> angle_offsets;
            if (r_idx == 0)
            {
                angle_offsets = {angle_offset_manipulator, angle_offset_manipulator + M_PI/6,  angle_offset_manipulator + 11*M_PI/6,  angle_offset_manipulator + 2*M_PI/6,  angle_offset_manipulator + 10*M_PI/6}; //3*M_PI/6, 4*M_PI/6, 5*M_PI/6, 6*M_PI/6, 7*M_PI/6, 8*M_PI/6, 9*M_PI/6,};
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
        visual_tools->trigger();

        ROS_INFO("=== Total poses in queue: %zu ===", pose_queue.size());

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

    // Plan a Cartesian path
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planCartesian(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();
        
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);
        move_group_.setMaxVelocityScalingFactor(maxV_scale_factor);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, trajectory);
        
        if (fraction >= cartesian_threshold) {
            ROS_INFO_NAMED("vader_planner", "Cutter cartesian path computed successfully.");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Cutter cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }


    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planGuidedCartesian(const geometry_msgs::Pose& target_pose) {
        auto current_pose = move_group_.getCurrentPose().pose;

        double dist = std::sqrt(
            std::pow(target_pose.position.x - current_pose.position.x, 2) +
            std::pow(target_pose.position.y - current_pose.position.y, 2) +
            std::pow(target_pose.position.z - current_pose.position.z, 2)
        );

        int num_waypoints = static_cast<int>(dist / eef_step);

        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 1; i <= num_waypoints; ++i) {
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

        move_group_.setMaxVelocityScalingFactor(maxV_scale_factor);
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, trajectory);

        robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), CUTTER_MOVE_GROUP);
        rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory); // computed_trajectory from computeCartesianPath()
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool iptp_success = iptp.computeTimeStamps(rt);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(iptp_success) {
            rt.getRobotTrajectoryMsg(trajectory);
            plan.trajectory_ = trajectory;
        } else {
            // Handle error
            ROS_WARN_NAMED("vader_planner", "Gripper: IPTP procedure error");
        }


        if (fraction >= cartesian_threshold) {
            ROS_INFO_NAMED("vader_planner", "Gripper guided cartesian path computed successfully.");
            return plan;
        } else {
            ROS_ERROR_NAMED("vader_planner", "Cutter guided cartesian path computation failed with coverage fraction: %f", fraction);
            return std::nullopt;
        }
    }

    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setStartStateToCurrentState();
        auto curr_pose = move_group_.getCurrentPose().pose;

        ROS_WARN_NAMED("vader_planner", "current pose: position=(%f, %f, %f) quat=(%f, %f, %f, %f)",
                        curr_pose.position.x, curr_pose.position.y, curr_pose.position.z,
                        curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w);
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_WARN_NAMED("vader_planner", "Cutter plan computed.");
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
        const moveit::core::JointModelGroup* joint_model_group = state_copy.getJointModelGroup(GRIPPER_MOVE_GROUP);

        // Check if the pose is reachable
        bool found_ik = state_copy.setFromIK(joint_model_group, target_pose, 10, 0.1);
        return found_ik;
    }


    std::queue<geometry_msgs::Pose> generate_parametric_circle_poses(geometry_msgs::Pose &fruit_pose, double approach_dist, double angle_offset_manipulator)
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

        ROS_INFO("=== Parametric Circle Pose Queue ===");

        std::vector<double> test_radii = {approach_dist, 0.3, 0.2, 0.35, 0.15, 0.4};
        
        for (size_t r_idx = 0; r_idx < test_radii.size(); r_idx++)
        {
            double radius = test_radii[r_idx];
            
            std::vector<double> angle_offsets;
            if (r_idx == 0)
            {
                angle_offsets = {angle_offset_manipulator, angle_offset_manipulator + M_PI/6,  angle_offset_manipulator + 11*M_PI/6,  angle_offset_manipulator + 2*M_PI/6,  angle_offset_manipulator + 10*M_PI/6}; //3*M_PI/6, 4*M_PI/6, 5*M_PI/6, 6*M_PI/6, 7*M_PI/6, 8*M_PI/6, 9*M_PI/6,};
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
        visual_tools->trigger();

        ROS_INFO("=== Total poses in queue: %zu ===", pose_queue.size());

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

        planning_scene_diff_pub =
            node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);


        display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/
        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        setupWorkspaceCollision();
        add_ground_plane_collision();

        // Load pose strings (expected "x y z qx qy qz qw" or "x,y,z,qx,qy,qz,qw") and other params from ROS parameter server
        auto parseXYZQuat = [&](const std::string &s, geometry_msgs::Pose &pose) -> bool {
            std::string tmp = s;
            for (char &c : tmp) if (c == ',') c = ' ';
            std::istringstream iss(tmp);
            std::vector<double> vals;
            double v;
            while (iss >> v) vals.push_back(v);
            if (vals.size() != 7) return false; // expecting x y z qx qy qz qw
            pose.position.x = vals[0];
            pose.position.y = vals[1];
            pose.position.z = vals[2];
            // Quaternion expected as qx, qy, qz, qw
            tf2::Quaternion q(vals[3], vals[4], vals[5], vals[6]);
            q.normalize();
            pose.orientation = tf2::toMsg(q);
            return true;
        };

        // Retrieve params (with sensible defaults)
        std::string storage_bin_pose_str, gripper_home_pose_str, cutter_home_pose_str;
        if (node_handle.getParam("storage_bin_pose", storage_bin_pose_str)) {
            if (!parseXYZQuat(storage_bin_pose_str, storageBinPose)) {
                ROS_WARN_NAMED("vader_planner", "storage_bin_pose param found but could not parse. Using default.");
                storageBinPose = makePose(0.2, -0.4, 0.3, QUAT_DOWN());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "storage_bin_pose param not found. Using default.");
            storageBinPose = makePose(0.2, -0.4, 0.3, QUAT_DOWN());
        }

        if (node_handle.getParam("gripper_home_pose", gripper_home_pose_str)) {
            if (!parseXYZQuat(gripper_home_pose_str, gripperHomePose)) {
                ROS_WARN_NAMED("vader_planner", "gripper_home_pose param found but could not parse. Using default.");
                gripperHomePose = makePose(0.6, 0.0, 0.3, QUAT_TOWARD_PLANT());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "gripper_home_pose param not found. Using default.");
            gripperHomePose = makePose(0.6, 0.0, 0.3, QUAT_TOWARD_PLANT());
        }

        if (node_handle.getParam("cutter_home_pose", cutter_home_pose_str)) {
            if (!parseXYZQuat(cutter_home_pose_str, cutterHomePose)) {
                ROS_WARN_NAMED("vader_planner", "cutter_home_pose param found but could not parse. Using default.");
                cutterHomePose = makePose(0.6, 0.5, 0.3, QUAT_TOWARD_PLANT());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "cutter_home_pose param not found. Using default.");
            cutterHomePose = makePose(0.6, 0.5, 0.3, QUAT_TOWARD_PLANT());
        }

        // Other numeric/string params
        if (!node_handle.getParam("arm_spacing", armSpacing)) {
            armSpacing = 0.5;
            ROS_WARN_NAMED("vader_planner", "arm_spacing param not found. Using default: %f", armSpacing);
        }
        if (!node_handle.getParam("arm_height_horizontal_mount", armHeightHorizontalMount)) {
            armHeightHorizontalMount = 0.5;
            ROS_WARN_NAMED("vader_planner", "arm_height_horizontal_mount param not found. Using default: %f", armHeightHorizontalMount);
        }

        ros::Duration(1.0).sleep(); // give publisher time to connect
        ROS_INFO_NAMED("vader_planner", "VADER Dual Planner Server Initialized");

        {
            // Print acquired parameters for debugging (print quaternion directly)
            ROS_WARN_NAMED("vader_planner", "storage_bin_pose: position=(%f, %f, %f) quat=(%f, %f, %f, %f)",
                           storageBinPose.position.x, storageBinPose.position.y, storageBinPose.position.z,
                           storageBinPose.orientation.x, storageBinPose.orientation.y, storageBinPose.orientation.z, storageBinPose.orientation.w);

            ROS_WARN_NAMED("vader_planner", "gripper_home_pose: position=(%f, %f, %f) quat=(%f, %f, %f, %f)",
                           gripperHomePose.position.x, gripperHomePose.position.y, gripperHomePose.position.z,
                           gripperHomePose.orientation.x, gripperHomePose.orientation.y, gripperHomePose.orientation.z, gripperHomePose.orientation.w);

            ROS_WARN_NAMED("vader_planner", "cutter_home_pose: position=(%f, %f, %f) quat=(%f, %f, %f, %f)",
                           cutterHomePose.position.x, cutterHomePose.position.y, cutterHomePose.position.z,
                           cutterHomePose.orientation.x, cutterHomePose.orientation.y, cutterHomePose.orientation.z, cutterHomePose.orientation.w);

            ROS_WARN_NAMED("vader_planner", "arm_spacing: %f", armSpacing);
            ROS_WARN_NAMED("vader_planner", "arm_height_horizontal_mount: %f", armHeightHorizontalMount);
        }

        //Register service handler

        planning_service = node_handle.advertiseService("vader_planning_service", &VADERPlannerServer::planningServiceHandler, this);
    }


    void add_ground_plane_collision()
    {
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
        ground_pose.position.z = 0.15 - (ground_primitive.dimensions[2] / 2.0);
        ground_pose.orientation.w = 1.0; // No rotation

        ground_plane.primitives.push_back(ground_primitive);
        ground_plane.primitive_poses.push_back(ground_pose);
        ground_plane.operation = moveit_msgs::CollisionObject::ADD;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Add the ground plane to the planning scene
        planning_scene_interface.applyCollisionObject(ground_plane);
    }


    void setupWorkspaceCollision(){
        //Warthog body
        moveit_msgs::CollisionObject warthog_body;
        warthog_body.id = "warthog_body";
        warthog_body.header.frame_id = "world";

        // Box dimensions: x = 0.5 (from -0.6 to -0.1), y = 1.0 (from -0.25 to 0.75), z = 1.0 (from 0 to 1)
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
        box_pose.position.z = 0.5; // center at z = 0.5
        box_pose.orientation.w = 1.0;

        warthog_body.primitives.push_back(box_primitive);
        warthog_body.primitive_poses.push_back(box_pose);
        warthog_body.operation = warthog_body.ADD;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // Define an orange bright color as std_msgs::ColorRGBA
        std_msgs::ColorRGBA orange_bright;
        orange_bright.r = 1.0f;
        orange_bright.g = 0.5f;
        orange_bright.b = 0.0f;
        orange_bright.a = 1.0f; // fully opaque

        planning_scene_interface.applyCollisionObject(warthog_body, orange_bright);
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
        // Start server logic here
        spinner.start();
    }

    void stop() {
        spinner.stop();
    }

    bool homeGripper(){
        auto plan = gripper_planner_.planRRT(gripperHomePose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper home movement.");
            return false;
        }
        show_trails(plan, std::nullopt);
        return gripper_planner_.execSync(plan.value());
    }

    bool homeCutter(){
        auto plan = cutter_planner_.planRRT(cutterHomePose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter home movement.");
            return false;
        }
        show_trails(std::nullopt, plan);
        return cutter_planner_.execSync(plan.value());
    }

    bool gripperGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        // Translate the target pose along its local -Z axis by final_approach_dist
        tf::Quaternion tq;
        tf::quaternionMsgToTF(target_pose.orientation, tq);
        tf::Vector3 world_offset = tf::quatRotate(tq, tf::Vector3(0.0, 0.0, -final_approach_dist));
        geometry_msgs::Pose approach_pose = target_pose; //copy for later use
        approach_pose.position.x += world_offset.x();
        approach_pose.position.y += world_offset.y();
        approach_pose.position.z += world_offset.z();

        auto plan = gripper_planner_.planGuidedCartesian(approach_pose);
        // plan = gripper_planner_.planRRT(approach_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper grasp movement.");
            return false;
        }
        show_trails(plan, std::nullopt);
        bool success = gripper_planner_.execSync(plan.value());

        // // Move cartesian to actual target pose
        // if(success) {
        //     auto cartesian_plan = gripper_planner_.planCartesian(target_pose);
        //     if(cartesian_plan == std::nullopt) {
        //         ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper final approach movement.");
        //         return false;
        //     }
        //     show_trails(cartesian_plan, std::nullopt);
        //     success &= gripper_planner_.execSync(cartesian_plan.value());
        // }
        return success;
    }

    bool cutterGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        //Rotate target

        // Create a rotation matrix for a 90-degree clockwise rotation around the z-axis
        // tf::Matrix3x3 rotation_matrix(
        //     0, 1, 0,  // First column
        //     -1, 0, 0, // Second column
        //     0, 0, 1   // Third column
        // );
        // tf::Quaternion tq_;
        // tf::quaternionMsgToTF(target_pose.orientation, tq_);

        // tf::Quaternion rotation_quat;
        // rotation_matrix.getRotation(rotation_quat);

        // // Apply rotation and normalize
        // tf::Quaternion rotated = tq_ * rotation_quat;
        // rotated.normalize();

        // // Write back to pose
        // tf::quaternionTFToMsg(rotated, target_pose.orientation);
        // Translate the target pose along its local -Z axis by final_approach_dist
        tf::Quaternion tq;
        tf::quaternionMsgToTF(target_pose.orientation, tq);
        tf::Vector3 world_offset = tf::quatRotate(tq, tf::Vector3(0.0, 0.0, -final_approach_dist));
        geometry_msgs::Pose approach_pose = target_pose; //copy for later use
        approach_pose.position.x += world_offset.x();
        approach_pose.position.y += world_offset.y();
        approach_pose.position.z += world_offset.z();
        visual_tools->publishAxisLabeled(approach_pose, "Cutter Approach Pose", rvt::SMALL);
        visual_tools->trigger();
        // auto plan = cutter_planner_.planRRT(approach_pose);
        auto plan = cutter_planner_.planGuidedCartesian(approach_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter grasp movement.");
            return false;
        }
        show_trails(std::nullopt, plan);
        bool success = cutter_planner_.execSync(plan.value());

        // Move cartesian to actual target pose
        if(success) {
            auto cartesian_plan = cutter_planner_.planGuidedCartesian(target_pose);// planCartesian(target_pose);
            if(cartesian_plan == std::nullopt) {
                ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter final approach movement.");
                return false;
            }
            show_trails(std::nullopt, cartesian_plan);
            success &= cutter_planner_.execSync(cartesian_plan.value());
        }
        return success;
    }

    bool parallelMovePregrasp(geometry_msgs::Pose& gripper_target_pose, geometry_msgs::Pose& cutter_target_pose){
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

        auto gripper_plan = gripper_planner_.planGuidedCartesian(gripper_target_pose);// planRRT(gripper_target_pose);

        // gripper_plan = gripper_planner_.planRRT(gripper_target_pose);
        if(gripper_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to pregrasp.");
            return false;
        }
        show_trails(gripper_plan, std::nullopt);
        success &= gripper_planner_.execSync(gripper_plan.value());

        auto cutter_plan = cutter_planner_.planGuidedCartesian(cutter_target_pose);

        // cutter_plan = cutter_planner_.planRRT(cutter_target_pose);
        if(cutter_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter movement to pregrasp.");
            return false;
        }
        show_trails(std::nullopt, cutter_plan);
        success &= cutter_planner_.execSync(cutter_plan.value());
        return success;
    }

    bool parallelMoveStorage(){

        bool success = true;
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


        auto gripper_plan = gripper_planner_.planRRT(storageBinPose);

        gripper_plan = gripper_planner_.planRRT(storageBinPose);
        if(gripper_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to storage.");
            return false;
        }
        show_trails(gripper_plan, std::nullopt);
        success &= gripper_planner_.execSync(gripper_plan.value());

        success &= homeCutter();
        return success;
    }

    bool planningServiceHandler(vader_msgs::PlanningRequest::Request &req,
                                vader_msgs::PlanningRequest::Response &res) {
        // Implement planning service logic here
        switch(req.mode) {
            case vader_msgs::PlanningRequest::Request::HOME_CUTTER:{

                res.success = homeCutter();
                break;
            }
            case vader_msgs::PlanningRequest::Request::HOME_GRIPPER:{

                res.success = homeGripper();
                break;
            }
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_PREGRASP:
            {
                vader_msgs::Pepper pepper_estimate = req.pepper;

                ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                        pepper_estimate.fruit_data.pose.position.x, pepper_estimate.fruit_data.pose.position.y, pepper_estimate.fruit_data.pose.position.z);

                pepper_estimate.fruit_data.pose.orientation.x = 0.0;
                pepper_estimate.fruit_data.pose.orientation.y = 0.0;
                pepper_estimate.fruit_data.pose.orientation.z = 0.0;
                pepper_estimate.fruit_data.pose.orientation.w = 1.0;

                pepper_estimate.peduncle_data.pose.orientation.x = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.y = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.z = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.w = 1.0;
                auto gripper_target_poses = gripper_planner_.generate_parametric_circle_poses(pepper_estimate.fruit_data.pose, 0.25, 3*M_PI/12);
                auto cutter_target_poses = cutter_planner_.generate_parametric_circle_poses(pepper_estimate.peduncle_data.pose, 0.25, -3* M_PI/12);
                visual_tools->trigger();
                // setUpSharedWorkspaceCollision(math.min(0.4, math.max(0.1, pepper_estimate.fruit_data.pose.position.y)), 0.6);
                // setUpSharedWorkspaceCollision(0.15, 0.6);


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
                        ROS_INFO("Found valid IK pose for gripper pregrasp.");
                        gripper_pregrasp_pose = gripper_test_pose;
                        visual_tools->deleteAllMarkers();
                        visual_tools->publishAxisLabeled(gripper_test_pose, "Gripper Pregrasp", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for gripper pregrasp.");
                    res.success = false;
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
                        ROS_INFO("Found valid IK pose for cutter pregrasp.");
                        cutter_pregrasp_pose = cutter_test_pose;
                        visual_tools->publishAxisLabeled(cutter_test_pose, "Cutter Pregrasp", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for cutter pregrasp.");
                    res.success = false;
                    return res.success;
                }
                visual_tools->trigger();
                res.success = parallelMovePregrasp(gripper_pregrasp_pose, cutter_pregrasp_pose);
                // setUpSharedWorkspaceCollision(0.25, 0.4);
                break;
            }
            case vader_msgs::PlanningRequest::Request::GRIPPER_GRASP:{
                // TODO here, calculate desired pose based off of pepper estimate
                vader_msgs::Pepper pepper_estimate = req.pepper;

                pepper_estimate.fruit_data.pose.orientation.x = 0.0;
                pepper_estimate.fruit_data.pose.orientation.y = 0.0;
                pepper_estimate.fruit_data.pose.orientation.z = 0.0;
                pepper_estimate.fruit_data.pose.orientation.w = 1.0;

                pepper_estimate.peduncle_data.pose.orientation.x = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.y = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.z = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.w = 1.0;
                double final_approach_dist = 0.10;

                ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                        pepper_estimate.fruit_data.pose.position.x, pepper_estimate.fruit_data.pose.position.y, pepper_estimate.fruit_data.pose.position.z);
                auto gripper_target_poses = gripper_planner_.generate_parametric_circle_poses(pepper_estimate.fruit_data.pose, final_approach_dist, 2*M_PI/12);
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
                        ROS_INFO("Found valid IK pose for gripper pregrasp.");
                        gripper_pregrasp_pose = gripper_test_pose;
                        visual_tools->deleteAllMarkers();
                        visual_tools->publishAxisLabeled(gripper_test_pose, "Gripper Grasp Pose", rvt::MEDIUM);
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for gripper pregrasp.");
                    res.success = false;
                    return res.success;
                }
                visual_tools->trigger();

                // Plan a Cartesian approach to the target pose

                // Translate the target pose along its local -Z axis by final_approach_dist
                tf::Quaternion tq;
                tf::quaternionMsgToTF(gripper_pregrasp_pose.orientation, tq);
                tf::Vector3 world_offset = tf::quatRotate(tq, tf::Vector3(0.0, 0.0, 0.25));
                gripper_pregrasp_pose.position.x += world_offset.x();
                gripper_pregrasp_pose.position.y += world_offset.y();
                gripper_pregrasp_pose.position.z += world_offset.z();
                visual_tools->publishAxisLabeled(gripper_pregrasp_pose, "Gripper Grasp Pose", rvt::MEDIUM);

                res.success = gripperGrasp(gripper_pregrasp_pose, final_approach_dist);


                // res.success = gripperGrasp(req., req.final_approach_dist);
                // res.success = false; // Not implemented
                break;
            }
            case vader_msgs::PlanningRequest::Request::CUTTER_GRASP:{
                // TODO here, calculate desired pose based off of pepper estimate
                vader_msgs::Pepper pepper_estimate = req.pepper;

                pepper_estimate.fruit_data.pose.orientation.x = 0.0;
                pepper_estimate.fruit_data.pose.orientation.y = 0.0;
                pepper_estimate.fruit_data.pose.orientation.z = 0.0;
                pepper_estimate.fruit_data.pose.orientation.w = 1.0;

                pepper_estimate.peduncle_data.pose.orientation.x = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.y = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.z = 0.0;
                pepper_estimate.peduncle_data.pose.orientation.w = 1.0;
                double final_approach_dist = 0.1;

                ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f",
                        pepper_estimate.peduncle_data.pose.position.x, pepper_estimate.peduncle_data.pose.position.y, pepper_estimate.peduncle_data.pose.position.z);
                auto cutter_target_poses = cutter_planner_.generate_parametric_circle_poses(pepper_estimate.peduncle_data.pose, final_approach_dist, -3*M_PI/12);
                // Test IK for each pose in the queue until we find one that is valid
                bool found_valid_poses = false;
                geometry_msgs::Pose cutter_pregrasp_pose;
                while(!cutter_target_poses.empty())
                {
                    geometry_msgs::Pose cutter_test_pose = cutter_target_poses.front();
                    cutter_target_poses.pop();

                    bool cutter_ik_valid = cutter_planner_.testPoseIK(cutter_test_pose);

                    if (cutter_ik_valid)
                    {
                        ROS_INFO("Found valid IK pose for cutter pregrasp.");
                        cutter_pregrasp_pose = cutter_test_pose;
                        // visual_tools->deleteAllMarkers();
                        found_valid_poses = true;
                        break;
                    }
                }
                if (!found_valid_poses)
                {
                    ROS_ERROR_NAMED("vader_planner", "Could not find valid IK pose for cutter pregrasp.");
                    res.success = false;
                    return res.success;
                }
                visual_tools->trigger();

                // Translate the target pose along its local -Z axis by final_approach_dist
                tf::Quaternion tq;
                tf::quaternionMsgToTF(cutter_pregrasp_pose.orientation, tq);
                tf::Vector3 world_offset = tf::quatRotate(tq, tf::Vector3(0.0, 0.0, 0.1));
                cutter_pregrasp_pose.position.x += world_offset.x();
                cutter_pregrasp_pose.position.y += world_offset.y();
                cutter_pregrasp_pose.position.z += world_offset.z() + 0.1;
                visual_tools->publishAxisLabeled(cutter_pregrasp_pose, "Cutter Grasp Pose", rvt::MEDIUM);

                // Plan a Cartesian approach to the target pose
                res.success = cutterGrasp(cutter_pregrasp_pose, final_approach_dist);

                break;
            }
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_STORAGE:{
                res.success = parallelMoveStorage();
                break;
            }
            default:{
                ROS_ERROR_NAMED("vader_planner", "Unknown planning mode requested: %d", req.mode);
                res.success = false;
                break;
            }
        }
        return res.success;
    }


    // void parallelPlanExecuteDemo(){
    //     ROS_WARN_NAMED("vader_planner", "Starting parallel planning and execution demo in ten seconds...");

    //     ros::Duration(10.0).sleep();
    //     // Define target poses for gripper and cutter
    //     geometry_msgs::Pose gripper_target_pose = makePose(0.5, 0.25, 0.4, QUAT_TOWARD_PLANT());
    //     geometry_msgs::Pose cutter_target_pose = makePose(0.5, 0.5, 0.4, QUAT_TOWARD_PLANT());

    //     // Plan for gripper
    //     auto gripper_plan = gripper_planner_.planRRT(gripper_target_pose);

    //     ROS_WARN_NAMED("vader_planner", "Gripper plan computed, executing asynchronously.");
        
    //     auto cutter_plan_future = std::async(&VADERCutterPlanner::planRRT, &cutter_planner_, cutter_target_pose);
    //     std::thread gripper_exec_thread([&]() {
    //         gripper_planner_.execSync(gripper_plan);
    //         ROS_WARN_NAMED("vader_planner", "Gripper movement finished.");
    //     });
    //     auto cutter_plan = cutter_plan_future.get();
    //     gripper_exec_thread.join();

    //     cutter_planner_.execSync(cutter_plan);
    // }

    void setUpSharedWorkspaceCollision(double divide_workspace_y, double x_depth) {
        moveit_msgs::CollisionObject left_workspace;
        left_workspace.id = "left_workspace";
        left_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive left_primitive = makeBoxPrimitive(x_depth, divide_workspace_y, 1);
        left_workspace.primitives.push_back(left_primitive);

        geometry_msgs::Pose left_pose = makePose(x_depth / 2.0, divide_workspace_y / 2.0, 0.5, QUAT_IDENTITY());
        left_workspace.primitive_poses.push_back(left_pose);
        left_workspace.operation = left_workspace.ADD;

        moveit_msgs::CollisionObject right_workspace;
        right_workspace.id = "right_workspace";
        right_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive right_primitive = makeBoxPrimitive(x_depth, 0.5 - divide_workspace_y, 1);
        right_workspace.primitives.push_back(right_primitive);

        geometry_msgs::Pose right_pose = makePose(x_depth / 2.0, divide_workspace_y + (0.5 - divide_workspace_y) / 2.0, 0.5, QUAT_IDENTITY());
        right_workspace.primitive_poses.push_back(right_pose);
        right_workspace.operation = right_workspace.ADD;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_interface.applyCollisionObject(left_workspace, COLOR_ORANGE_TRANSLUCENT());
        planning_scene_interface.applyCollisionObject(right_workspace, COLOR_BLUE_TRANSLUCENT());

        

        // Allow collision between box and a specific link (e.g. "wrist_3_link")
        std::vector<std::pair<std::string, std::string>> allowed_entries;
        for (const auto& link_name_suffix : LINK_SUFFIXES) {
            std::string gripper_link_name = "L_" + link_name_suffix;
            std::string cutter_link_name = "R_" + link_name_suffix;
            allowed_entries.emplace_back("left_workspace", gripper_link_name);
            allowed_entries.emplace_back("right_workspace", cutter_link_name);
        }
        allowed_entries.emplace_back("vader_gripper_base_link", "left_workspace");
        allowed_entries.emplace_back("fing_1", "left_workspace");
        allowed_entries.emplace_back("fing_2", "left_workspace");
        allowed_entries.emplace_back("thumb_1", "left_workspace");
        allowed_entries.emplace_back("vader_cutter_base_link", "right_workspace");
        allowed_entries.emplace_back("blade_moving", "right_workspace");
        allowed_entries.emplace_back("blade_stationary", "right_workspace");
        

        setACMEntries(psm, planning_scene_diff_pub, allowed_entries);
        ROS_WARN_NAMED("vader_planner", "Shared workspace demo collision objs setup complete.");
    }

private:
    geometry_msgs::Pose storageBinPose;
    geometry_msgs::Pose gripperHomePose;
    geometry_msgs::Pose cutterHomePose;
    double armSpacing;
    double armHeightHorizontalMount;    
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world"); //shared from planner server main instance
    ros::Publisher display_path;
    
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_pub;
    ros::ServiceServer planning_service;

    planning_scene_monitor::PlanningSceneMonitorPtr psm;
    VADERGripperPlanner gripper_planner_;
    VADERCutterPlanner cutter_planner_;
    ros::AsyncSpinner spinner;

    std::string GRIPPER_PLANNING_FRAME;
    std::string CUTTER_PLANNING_FRAME;

    const std::vector<std::string> LINK_SUFFIXES = {"link1", "link2", "link3", "link4", "link5", "link6", "link7", "link_base", "link_eef"};

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_dual_planner");

    VADERPlannerServer plannerServer;

    plannerServer.start();

    // plannerServer.homeGripper();
    // plannerServer.homeCutter();

    // plannerServer.setUpSharedWorkspaceCollision(0.25);
    // ros::Duration(0.5).sleep();
    // plannerServer.homeGripper();
    // plannerServer.parallelMoveStorage();

    // tf2::Quaternion q;
    // double angle_rad = 20.0 * M_PI / 180.0;
    // q.setRPY(0.0, angle_rad, 0.0); // roll, pitch, yaw -> rotate 20 deg about Y (pitch)
    // q.normalize();
    // geometry_msgs::Quaternion quat_msg = tf2::toMsg(q);
    // geometry_msgs::Pose fruit_pose = makePose(0.7, 0.0, 0.5, quat_msg);
    
    // plannerServer.generate_parametric_circle_poses(
    //     fruit_pose, 0.2);
    // geometry_msgs::Pose target_pose = makePose(0.7, 0.0, 0.5, QUAT_TOWARD_PLANT());
    // plannerServer.gripperGrasp(target_pose, 0.0);

    // ros::Duration(5).sleep();

    // double divide_workspace_y = 0.375;

    // plannerServer.parallelPlanExecuteDemo();
    ros::waitForShutdown();
    return 0;
}
