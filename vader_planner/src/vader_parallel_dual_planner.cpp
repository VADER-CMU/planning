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

#include <geometric_shapes/shapes.h> // Include for cylinder shape
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <vader_msgs/BimanualPlanRequest.h>
#include <vader_msgs/BimanualExecRequest.h>
#include <vader_msgs/MoveToStorageRequest.h>
#include <vader_msgs/GoHomeRequest.h>
#include <vader_msgs/SimulationPepperSequence.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <geometric_shapes/shape_operations.h>
#include "utils/utils.h"

#include <thread>
#include <future>
#include <iostream>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

const std::string GRIPPER_MOVE_GROUP = "L_xarm7";
const std::string CUTTER_MOVE_GROUP = "R_xarm7";

namespace rvt = rviz_visual_tools;

class VADERGripperPlanner {
public:
    VADERGripperPlanner()
        : move_group_(GRIPPER_MOVE_GROUP) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Gripper Planner");
        }

    // Plan a Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan planCartesian(const std::vector<geometry_msgs::Pose>& waypoints) {
        // moveit_msgs::RobotTrajectory trajectory;
        // double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // plan.trajectory_ = trajectory;
        return plan;
    }

    // Plan using RRT (default MoveIt planner)
    moveit::planning_interface::MoveGroupInterface::Plan planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ROS_WARN_NAMED("vader_planner", "Planning for gripper...");
        move_group_.plan(plan);
        ROS_WARN_NAMED("vader_planner", "Gripper plan computed.");
        return plan;
    }

    // Execute synchronously
    bool execSync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // Execute asynchronously
    void execAsync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_.asyncExecute(plan);
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
};

class VADERCutterPlanner {
public:
    VADERCutterPlanner()
        : move_group_(CUTTER_MOVE_GROUP) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Cutter Planner");
        }

    // Plan a Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan planCartesian(const std::vector<geometry_msgs::Pose>& waypoints) {
        // moveit_msgs::RobotTrajectory trajectory;
        // double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // plan.trajectory_ = trajectory;
        return plan;
    }

    // Plan using RRT (default MoveIt planner)
    moveit::planning_interface::MoveGroupInterface::Plan planRRT(const geometry_msgs::Pose& target_pose) {
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_.plan(plan);
        ROS_WARN_NAMED("vader_planner", "Cutter plan computed.");
        return plan;
    }

    // Execute synchronously
    bool execSync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // Execute asynchronously
    void execAsync(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_.asyncExecute(plan);
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
};

class VADERPlannerServer {
public:
    VADERPlannerServer()
        : spinner(SPINNER_THREAD_NUM),
          gripper_planner_(),
          cutter_planner_() {
        
        planning_scene_diff_pub =
            node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        // Load pose strings (expected "x y z roll pitch yaw" or "x,y,z,roll,pitch,yaw") and other params from ROS parameter server
        auto parseXYZRPY = [&](const std::string &s, geometry_msgs::Pose &pose) -> bool {
            std::string tmp = s;
            for (char &c : tmp) if (c == ',') c = ' ';
            std::istringstream iss(tmp);
            std::vector<double> vals;
            double v;
            while (iss >> v) vals.push_back(v);
            if (vals.size() != 6) return false;
            pose.position.x = vals[0];
            pose.position.y = vals[1];
            pose.position.z = vals[2];
            tf2::Quaternion q;
            q.setRPY(vals[3], vals[4], vals[5]);
            pose.orientation = tf2::toMsg(q);
            return true;
        };

        // Retrieve params (with sensible defaults)
        std::string storage_bin_pose_str, gripper_home_pose_str, cutter_home_pose_str;
        if (node_handle.getParam("storage_bin_pose", storage_bin_pose_str)) {
            if (!parseXYZRPY(storage_bin_pose_str, storageBinPose)) {
                ROS_WARN_NAMED("vader_planner", "storage_bin_pose param found but could not parse. Using default.");
                storageBinPose = makePose(0.4, 0.0, 0.2, QUAT_IDENTITY());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "storage_bin_pose param not found. Using default.");
            storageBinPose = makePose(0.4, 0.0, 0.2, QUAT_IDENTITY());
        }

        if (node_handle.getParam("gripper_home_pose", gripper_home_pose_str)) {
            if (!parseXYZRPY(gripper_home_pose_str, gripperHomePose)) {
                ROS_WARN_NAMED("vader_planner", "gripper_home_pose param found but could not parse. Using default.");
                gripperHomePose = makePose(0.3, 0.2, 0.3, QUAT_IDENTITY());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "gripper_home_pose param not found. Using default.");
            gripperHomePose = makePose(0.3, 0.2, 0.3, QUAT_IDENTITY());
        }

        if (node_handle.getParam("cutter_home_pose", cutter_home_pose_str)) {
            if (!parseXYZRPY(cutter_home_pose_str, cutterHomePose)) {
                ROS_WARN_NAMED("vader_planner", "cutter_home_pose param found but could not parse. Using default.");
                cutterHomePose = makePose(0.3, -0.2, 0.3, QUAT_IDENTITY());
            }
        } else {
            ROS_WARN_NAMED("vader_planner", "cutter_home_pose param not found. Using default.");
            cutterHomePose = makePose(0.3, -0.2, 0.3, QUAT_IDENTITY());
        }

        // Other numeric/string params
        if (!node_handle.getParam("inter_arm_distance", interArmDistance)) {
            interArmDistance = 0.3;
            ROS_WARN_NAMED("vader_planner", "inter_arm_distance param not found. Using default: %f", interArmDistance);
        }
        if (!node_handle.getParam("arm_ground_clearance", armGroundClearance)) {
            armGroundClearance = 0.05;
            ROS_WARN_NAMED("vader_planner", "arm_ground_clearance param not found. Using default: %f", armGroundClearance);
        }

        ros::Duration(1.0).sleep(); // give publisher time to connect
        ROS_INFO_NAMED("vader_planner", "VADER Dual Planner Server Initialized");

        {
            // Print acquired parameters for debugging
            auto poseToRPY = [&](const geometry_msgs::Pose &pose, double &roll, double &pitch, double &yaw) {
                tf::Quaternion q;
                tf::quaternionMsgToTF(pose.orientation, q);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            };

            double r, p, y;
            poseToRPY(storageBinPose, r, p, y);
            ROS_WARN_NAMED("vader_planner", "storage_bin_pose: position=(%f, %f, %f) rpy=(%f, %f, %f)",
                           storageBinPose.position.x, storageBinPose.position.y, storageBinPose.position.z, r, p, y);

            poseToRPY(gripperHomePose, r, p, y);
            ROS_WARN_NAMED("vader_planner", "gripper_home_pose: position=(%f, %f, %f) rpy=(%f, %f, %f)",
                           gripperHomePose.position.x, gripperHomePose.position.y, gripperHomePose.position.z, r, p, y);

            poseToRPY(cutterHomePose, r, p, y);
            ROS_WARN_NAMED("vader_planner", "cutter_home_pose: position=(%f, %f, %f) rpy=(%f, %f, %f)",
                           cutterHomePose.position.x, cutterHomePose.position.y, cutterHomePose.position.z, r, p, y);

            ROS_WARN_NAMED("vader_planner", "inter_arm_distance: %f", interArmDistance);
            ROS_WARN_NAMED("vader_planner", "arm_ground_clearance: %f", armGroundClearance);
        }
    }

    void start() {
        // Start server logic here
        spinner.start();
    }

    void stop() {
        spinner.stop();
    }

    bool homeGripper(){

    }

    bool homeCutter(){

    }

    bool gripperGrasp(){

    }

    bool cutterGrasp(){

    }

    bool parallelMovePregrasp(){

    }

    bool parallelMoveStorage(){

    }

    void handleCallback(){
        //switchboard for above movement functions
    }


    void parallelPlanExecuteDemo(){
        ROS_WARN_NAMED("vader_planner", "Starting parallel planning and execution demo in ten seconds...");

        ros::Duration(10.0).sleep();
        // Define target poses for gripper and cutter
        geometry_msgs::Pose gripper_target_pose = makePose(0.5, 0.25, 0.4, QUAT_TOWARD_PLANT());
        geometry_msgs::Pose cutter_target_pose = makePose(0.5, 0.5, 0.4, QUAT_TOWARD_PLANT());

        // Plan for gripper
        auto gripper_plan = gripper_planner_.planRRT(gripper_target_pose);

        ROS_WARN_NAMED("vader_planner", "Gripper plan computed, executing asynchronously.");
        
        auto cutter_plan_future = std::async(&VADERCutterPlanner::planRRT, &cutter_planner_, cutter_target_pose);
        std::thread gripper_exec_thread([&]() {
            gripper_planner_.execSync(gripper_plan);
            ROS_WARN_NAMED("vader_planner", "Gripper movement finished.");
        });
        auto cutter_plan = cutter_plan_future.get();
        gripper_exec_thread.join();

        cutter_planner_.execSync(cutter_plan);
    }

    void setUpSharedWorkspaceCollision(double divide_workspace_y) {
        moveit_msgs::CollisionObject left_workspace;
        left_workspace.id = "left_workspace";
        left_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive left_primitive = makeBoxPrimitive(0.4, divide_workspace_y, 0.5);
        left_workspace.primitives.push_back(left_primitive);

        geometry_msgs::Pose left_pose = makePose(0.2, divide_workspace_y / 2.0, 0.25, QUAT_IDENTITY());
        left_workspace.primitive_poses.push_back(left_pose);
        left_workspace.operation = left_workspace.ADD;

        moveit_msgs::CollisionObject right_workspace;
        right_workspace.id = "right_workspace";
        right_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive right_primitive = makeBoxPrimitive(0.4, 0.5 - divide_workspace_y, 0.5);
        right_workspace.primitives.push_back(right_primitive);

        geometry_msgs::Pose right_pose = makePose(0.2, divide_workspace_y + (0.5 - divide_workspace_y) / 2.0, 0.25, QUAT_IDENTITY());
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

        setACMEntries(psm, planning_scene_diff_pub, allowed_entries);
        ROS_WARN_NAMED("vader_planner", "Shared workspace demo collision objs setup complete.");
    }

private:
    geometry_msgs::Pose storageBinPose;
    geometry_msgs::Pose gripperHomePose;
    geometry_msgs::Pose cutterHomePose;
    double interArmDistance;
    double armGroundClearance;

    
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_pub;
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

    // ros::Duration(5).sleep();

    // double divide_workspace_y = 0.375;

    // plannerServer.setUpSharedWorkspaceCollision(divide_workspace_y);
    // plannerServer.parallelPlanExecuteDemo();
    ros::waitForShutdown();
    return 0;
}
