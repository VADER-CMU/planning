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

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;
const double cartesian_threshold = 0.9;

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
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planCartesian(const geometry_msgs::Pose& target_pose) {
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

    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
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

    moveit::planning_interface::MoveGroupInterface move_group_;
};

class VADERCutterPlanner {
public:
    VADERCutterPlanner()
        : move_group_(CUTTER_MOVE_GROUP) {
            ROS_INFO_NAMED("vader_planner", "Initialized VADER Cutter Planner");
        }

    // Plan a Cartesian path
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planCartesian(const geometry_msgs::Pose& target_pose) {
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose);
        move_group_.setMaxVelocityScalingFactor(maxV_scale_factor);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
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

    // Plan using RRT (default MoveIt planner)
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planRRT(const geometry_msgs::Pose& target_pose) {
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


        display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/
        visual_tools = new moveit_visual_tools::MoveItVisualTools("L_link_base");

        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        setupWorkspaceCollision();

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
        box_pose.position.x = -0.5;
        box_pose.position.y = (-0.25 + 0.75) / 2.0; // 0.25
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

    ~VADERPlannerServer() {
        delete visual_tools;
    }

    void show_trails(const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& plan_gripper,
                     const std::optional<moveit::planning_interface::MoveGroupInterface::Plan>& plan_cutter)
    {
        visual_tools->deleteAllMarkers();

        if (plan_gripper.has_value()) {
            const robot_state::JointModelGroup *joint_model_group_gripper = gripper_planner_.move_group_.getCurrentState()->getJointModelGroup(GRIPPER_MOVE_GROUP);
            // const robot_state::LinkModel* eef_link_model = joint_model_group_gripper->getParentModel().getLinkModel("L_link_eef");
            visual_tools->publishTrajectoryLine(plan_gripper->trajectory_, joint_model_group_gripper);
        }
        if (plan_cutter.has_value()) {
            const robot_state::JointModelGroup *joint_model_group_cutter = cutter_planner_.move_group_.getCurrentState()->getJointModelGroup(CUTTER_MOVE_GROUP);
            // const robot_state::LinkModel* eef_link_model = joint_model_group_cutter->getParentModel().getLinkModel("R_link_eef");
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
        // show_trails(plan, std::nullopt);
        return gripper_planner_.execSync(plan.value());
    }

    bool homeCutter(){
        auto plan = cutter_planner_.planRRT(cutterHomePose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter home movement.");
            return false;
        }
        // show_trails(std::nullopt, plan);
        return cutter_planner_.execSync(plan.value());
    }

    bool gripperGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        // return false;
        auto plan = gripper_planner_.planCartesian(target_pose);
        if(plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper grasp movement.");
            return false;
        }
        // show_trails(plan, std::nullopt);
        return gripper_planner_.execSync(plan.value());
    }

    bool cutterGrasp(geometry_msgs::Pose& target_pose, double final_approach_dist){
        return false;
    }

    bool parallelMovePregrasp(geometry_msgs::Pose& gripper_target_pose, geometry_msgs::Pose& cutter_target_pose){
        return false;
    }

    bool parallelMoveStorage(){
        // auto gripper_plan = gripper_planner_.planRRT(storageBinPose);
        // if(gripper_plan == std::nullopt) {
        //     ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to storage bin.");
        //     return false;
        // }

        // // show_trails(gripper_plan, std::nullopt);

        // bool success = gripper_planner_.execSync(gripper_plan.value());

        // if(!success) {
        //     ROS_ERROR_NAMED("vader_planner", "Execution to storage location failed");
        //     return false;
        // }

        // return homeCutter();

        bool success = true;
        auto gripper_plan = gripper_planner_.planRRT(storageBinPose);
        if(gripper_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan gripper movement to storage bin.");
            return false;
        }
        auto cutter_plan_future = std::async(&VADERCutterPlanner::planRRT, &cutter_planner_, cutterHomePose);
        std::thread gripper_exec_thread([&]() {
            gripper_planner_.execSync(gripper_plan.value());
            ROS_WARN_NAMED("vader_planner", "Gripper movement to storage bin finished.");
        });
        auto cutter_plan = cutter_plan_future.get();
        if(cutter_plan == std::nullopt) {
            ROS_ERROR_NAMED("vader_planner", "Failed to plan cutter home movement.");
            success = false;
        }
        gripper_exec_thread.join();
        success &= cutter_planner_.execSync(cutter_plan.value());
        return success;
    }

    std::vector<geometry_msgs::Pose> generatePregraspPoses(const vader_msgs::Pepper& pepper, bool is_gripper){
        std::vector<geometry_msgs::Pose> poses;
        // TODO implement
        return poses;
    }

    bool planningServiceHandler(vader_msgs::PlanningRequest::Request &req,
                                vader_msgs::PlanningRequest::Response &res) {
        // Implement planning service logic here
        switch(req.mode) {
            case vader_msgs::PlanningRequest::Request::HOME_CUTTER:
                res.success = homeCutter();
                break;
            case vader_msgs::PlanningRequest::Request::HOME_GRIPPER:
                res.success = homeGripper();
                break;
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_PREGRASP:
                //TODO here, calculate desired pose based off of pepper estimate
                // res.success = parallelMovePregrasp(req.gripper_target_pose, req.cutter_target_pose);
                res.success = false; // Not implemented
                break;
            case vader_msgs::PlanningRequest::Request::GRIPPER_GRASP:
                // TODO here, calculate desired pose based off of pepper estimate
                // res.success = gripperGrasp(req., req.final_approach_dist);
                res.success = false; // Not implemented
                break;
            case vader_msgs::PlanningRequest::Request::CUTTER_GRASP:
                // TODO here, calculate desired pose based off of pepper estimate
                // res.success = cutterGrasp(req., req.final_approach_dist);
                res.success = false; // Not implemented
                break;
            case vader_msgs::PlanningRequest::Request::PARALLEL_MOVE_STORAGE:
                res.success = parallelMoveStorage();
                break;
            default:
                ROS_ERROR_NAMED("vader_planner", "Unknown planning mode requested: %d", req.mode);
                res.success = false;
                break;
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

    void setUpSharedWorkspaceCollision(double divide_workspace_y) {
        moveit_msgs::CollisionObject left_workspace;
        left_workspace.id = "left_workspace";
        left_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive left_primitive = makeBoxPrimitive(0.6, divide_workspace_y, 1);
        left_workspace.primitives.push_back(left_primitive);

        geometry_msgs::Pose left_pose = makePose(0.3, divide_workspace_y / 2.0, 0.5, QUAT_IDENTITY());
        left_workspace.primitive_poses.push_back(left_pose);
        left_workspace.operation = left_workspace.ADD;

        moveit_msgs::CollisionObject right_workspace;
        right_workspace.id = "right_workspace";
        right_workspace.header.frame_id = "world";
        shape_msgs::SolidPrimitive right_primitive = makeBoxPrimitive(0.6, 0.5 - divide_workspace_y, 1);
        right_workspace.primitives.push_back(right_primitive);

        geometry_msgs::Pose right_pose = makePose(0.3, divide_workspace_y + (0.5 - divide_workspace_y) / 2.0, 0.5, QUAT_IDENTITY());
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
    moveit_visual_tools::MoveItVisualTools *visual_tools;
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

    plannerServer.setUpSharedWorkspaceCollision(0.25);
    ros::Duration(0.5).sleep();
    plannerServer.homeGripper();
    plannerServer.parallelMoveStorage();
    // geometry_msgs::Pose target_pose = makePose(0.7, 0.0, 0.5, QUAT_TOWARD_PLANT());
    // plannerServer.gripperGrasp(target_pose, 0.0);

    // ros::Duration(5).sleep();

    // double divide_workspace_y = 0.375;

    // plannerServer.parallelPlanExecuteDemo();
    ros::waitForShutdown();
    return 0;
}
