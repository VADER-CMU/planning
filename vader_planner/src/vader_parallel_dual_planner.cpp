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
        // Initialize ROS services, subscribers, publishers here
        
        planning_scene_diff_pub =
            node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        ros::Duration(1.0).sleep(); // give publisher time to connect
        ROS_INFO_NAMED("vader_planner", "VADER Dual Planner Server Initialized");
    }

    void start() {
        // Start server logic here
        spinner.start();
    }

    void stop() {
        spinner.stop();
    }

    // Add collision object to cutter planning frame
    void addCollisionObjectToCutter(const moveit_msgs::CollisionObject& collision_object) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        moveit_msgs::CollisionObject obj = collision_object;
        obj.header.frame_id = CUTTER_PLANNING_FRAME;
        collision_objects.push_back(obj);
        planning_scene_interface.addCollisionObjects(collision_objects);
    }

    // Add collision object to gripper planning frame
    void addCollisionObjectToGripper(const moveit_msgs::CollisionObject& collision_object) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        moveit_msgs::CollisionObject obj = collision_object;
        obj.header.frame_id = GRIPPER_PLANNING_FRAME;
        collision_objects.push_back(obj);
        planning_scene_interface.addCollisionObjects(collision_objects);
    }
    
    void addCollisionObjectGlobal(const moveit_msgs::CollisionObject& collision_object) {
        addCollisionObjectToCutter(collision_object);
        addCollisionObjectToGripper(collision_object);
    }

    void parallelPlanExecuteDemo(){
        ros::Duration(10.0).sleep();
        ROS_WARN_NAMED("vader_planner", "Starting parallel planning and execution demo");

        // Define target poses for gripper and cutter
        geometry_msgs::Pose gripper_target_pose;
        gripper_target_pose.position.x = 0.4;
        gripper_target_pose.position.y = 0.2;
        gripper_target_pose.position.z = 0.3;
        gripper_target_pose.orientation.w = 1.0;

        geometry_msgs::Pose cutter_target_pose;
        cutter_target_pose.position.x = 0.1;
        cutter_target_pose.position.y = 0.3;
        cutter_target_pose.position.z = 0.6;
        cutter_target_pose.orientation.w = 1.0;


        // Plan for gripper
        auto gripper_plan = gripper_planner_.planRRT(gripper_target_pose);


        ROS_WARN_NAMED("vader_planner", "Gripper plan computed, executing asynchronously.");
        gripper_planner_.execAsync(gripper_plan);

        // Plan for cutter (while gripper is executing)
        auto cutter_plan = cutter_planner_.planRRT(gripper_target_pose);
        ROS_WARN_NAMED("vader_planner", "Cutter plan computed, waiting for gripper to finish movement.");
        cutter_planner_.execSync(cutter_plan);

        ros::Duration(5.0).sleep(); // wait for gripper to finish
    }

    void sharedWorkspaceDemo(double divide_workspace_y) {
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

    ros::Duration(5).sleep();

    plannerServer.sharedWorkspaceDemo(0.4);
    // moveit_msgs::CollisionObject collision_object;
    // collision_object.id = "box1";
    // collision_object.header.frame_id = "world";
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 0.2;
    // primitive.dimensions[1] = 0.2;
    // primitive.dimensions[2] = 0.2;
    // collision_object.primitives.push_back(primitive);


    // geometry_msgs::Pose box_pose;
    // box_pose.position.x = 0.4;
    // box_pose.position.y = 0.2;
    // box_pose.position.z = 0.1;
    // box_pose.orientation.w = 1.0;
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // std_msgs::ColorRGBA object_color;
    // object_color.r = 1.0;
    // object_color.a = .2;
    
    // std_msgs::ColorRGBA object_color2;
    

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // planning_scene_interface.applyCollisionObject(collision_object, object_color);

    // plannerServer.parallelPlanExecuteDemo();
    ros::waitForShutdown();
    return 0;
}
