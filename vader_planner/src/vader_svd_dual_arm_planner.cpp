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

#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <vader_msgs/BimanualPlanRequest.h>
#include <vader_msgs/BimanualExecRequest.h>
#include <vader_msgs/MoveToStorageRequest.h>

#include <iostream>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

const std::string PLAN_GROUP_GRIPPER_PREFIX = "L_xarm7";
const std::string PLAN_GROUP_CUTTER_PREFIX = "R_xarm7";

namespace rvt = rviz_visual_tools;

class VADERPlanner
{
public:
    VADERPlanner() : spinner(SPINNER_THREAD_NUM),
                     PLANNING_GROUP_GRIPPER(PLAN_GROUP_GRIPPER_PREFIX),
                     PLANNING_GROUP_CUTTER(PLAN_GROUP_CUTTER_PREFIX),
                     group_gripper(PLAN_GROUP_GRIPPER_PREFIX),
                     group_cutter(PLAN_GROUP_CUTTER_PREFIX)
    {
        init();
    };
    ~VADERPlanner() { delete visual_tools; };
    void start();
    void stop();

private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group_gripper, group_cutter;
    moveit::planning_interface::MoveGroupInterface::Plan plan_gripper, plan_cutter;
    moveit_visual_tools::MoveItVisualTools *visual_tools;

    uint8_t plan_type = 0;

    std::string PLANNING_GROUP_GRIPPER;
    std::string PLANNING_GROUP_CUTTER;

    ros::ServiceServer planning_service, execution_service;
    ros::ServiceServer move_to_storage_service;

    ros::Publisher display_path;

    void init();
    bool planning_service_handler(vader_msgs::BimanualPlanRequest::Request &req, vader_msgs::BimanualPlanRequest::Response &res);
    bool execution_service_handler(vader_msgs::BimanualExecRequest::Request &req, vader_msgs::BimanualExecRequest::Response &res);
    bool move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res);
    void show_trail(bool plan_cutteresult);
};

void VADERPlanner::init()
{
    display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

    visual_tools = new moveit_visual_tools::MoveItVisualTools("L_link_base");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.8;

    planning_service = node_handle.advertiseService("vader_plan", &VADERPlanner::planning_service_handler, this);
    execution_service = node_handle.advertiseService("vader_exec", &VADERPlanner::execution_service_handler, this);
    // move_to_storage_service = node_handle.advertiseService("move_to_storage", &VADERPlanner::move_to_storage_service_handler, this);

    // Initialize subscriber and publisher
    ROS_INFO("Planner initialized with left planning group: %s and right planning group: %s",
             PLANNING_GROUP_GRIPPER.c_str(), PLANNING_GROUP_CUTTER.c_str());
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

void VADERPlanner::show_trail(bool plan_cutteresult)
{
    if (plan_cutteresult)
    {
        ROS_INFO_NAMED("vader_planner", "Visualizing plan as trajectory line");

        visual_tools->deleteAllMarkers();
        const robot_state::JointModelGroup *joint_model_group_gripper = group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        visual_tools->publishTrajectoryLine(plan_gripper.trajectory_, joint_model_group_gripper);
        const robot_state::JointModelGroup *joint_model_group_cutter = group_cutter.getCurrentState()->getJointModelGroup(PLANNING_GROUP_CUTTER);
        visual_tools->publishTrajectoryLine(plan_cutter.trajectory_, joint_model_group_cutter);
        visual_tools->trigger();
    }
}


bool VADERPlanner::planning_service_handler(vader_msgs::BimanualPlanRequest::Request &req, vader_msgs::BimanualPlanRequest::Response &res){
    if(req.mode == req.CUTTER_GRASP_PLAN) {
        // Cutter
    } else {
        // Gripper
        plan_type = req.mode; //Store which plan is being used for gripper and check this when executing
    }
}

bool VADERPlanner::execution_service_handler(vader_msgs::BimanualExecRequest::Request &req, vader_msgs::BimanualExecRequest::Response &res){
    if(req.mode == req.CUTTER_GRASP_EXEC) {
        // Cutter
        bool exec_ok = (group_cutter.execute(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        res.result = exec_ok;
        return exec_ok;
    } else {
        // Gripper
        if (plan_type != req.mode) {
            ROS_ERROR("The plan type does not match the execution type. Aborting execution");
            res.result = false;
            return false;
        }

        bool exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        res.result = exec_ok;
        return exec_ok;
    }
}

bool VADERPlanner::move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res){

}


// // Rotates given pose by 90 deg around the y axis
// geometry_msgs::Pose calculateFruitPregraspPose(const geometry_msgs::Pose &target_pose)
// {
//     // Assuming target_pose contains the necessary information to calculate the pregrasp pose
//     tf::Quaternion pregrasp_pose;

//     tf::Quaternion rotation_quat;
//     rotation_quat.setRPY(-M_PI / 2, 0, 0);

//     tf::Quaternion original_quat;
//     tf::quaternionMsgToTF(target_pose.orientation, original_quat);

//     pregrasp_pose = rotation_quat * original_quat;
//     pregrasp_pose.normalize();

//     geometry_msgs::Pose pregrasp_pose_result;
//     tf::quaternionTFToMsg(pregrasp_pose, pregrasp_pose_result.orientation);

//     pregrasp_pose_result.position.x = target_pose.position.x; // + 0.1; // Adjust as needed
//     pregrasp_pose_result.position.y = target_pose.position.y; // + 0.1; // Adjust as needed
//     pregrasp_pose_result.position.z = target_pose.position.z; // + 0.1; // Adjust as needed
//     return pregrasp_pose_result;
// }

// geometry_msgs::Pose calculatePedunclePregraspPose(const geometry_msgs::Pose &target_pose)
// {
//     // Assuming target_pose contains the necessary information to calculate the pregrasp pose
//     tf::Quaternion pregrasp_pose;

//     tf::Quaternion rotation_quat;
//     rotation_quat.setRPY(M_PI / 2, 0, 0);

//     tf::Quaternion original_quat;
//     tf::quaternionMsgToTF(target_pose.orientation, original_quat);

//     pregrasp_pose = rotation_quat * original_quat;
//     pregrasp_pose.normalize();

//     geometry_msgs::Pose pregrasp_pose_result;
//     tf::quaternionTFToMsg(pregrasp_pose, pregrasp_pose_result.orientation);

//     pregrasp_pose_result.position.x = target_pose.position.x; // + 0.1; // Adjust as needed
//     pregrasp_pose_result.position.y = target_pose.position.y; // + 0.1; // Adjust as needed
//     pregrasp_pose_result.position.z = target_pose.position.z; // + 0.1; // Adjust as needed
//     return pregrasp_pose_result;
// }

// void VADERPlanner::do_pepper_plan(const vader_msgs::Pepper::ConstPtr &msg)
// {
//     // std::cout << "Hello" << std::endl;
//     ROS_INFO_NAMED("do_pepper_plan", "Received new pepper planning request");

//     vader_msgs::Fruit fruit = msg->fruit_data;
//     vader_msgs::Peduncle peduncle = msg->peduncle_data;
//     // Make it a collision object
//     moveit_msgs::CollisionObject fruit_collision_obj;
//     fruit_collision_obj.id = "fruit_1";
//     fruit_collision_obj.header.frame_id = "L_link_base";
//     fruit_collision_obj.primitives.push_back(fruit.shape);
//     fruit_collision_obj.primitive_poses.push_back(fruit.pose);
//     fruit_collision_obj.operation = fruit_collision_obj.ADD;
//     planning_scene_interface.applyCollisionObject(fruit_collision_obj);
//     // Make it a collision object
//     moveit_msgs::CollisionObject peduncle_collision_obj;
//     peduncle_collision_obj.id = "peduncle_1";
//     peduncle_collision_obj.header.frame_id = "L_link_base";
//     peduncle_collision_obj.primitives.push_back(peduncle.shape);
//     peduncle_collision_obj.primitive_poses.push_back(peduncle.pose);
//     peduncle_collision_obj.operation = peduncle_collision_obj.ADD;
//     planning_scene_interface.applyCollisionObject(peduncle_collision_obj);

//     // Do gripper movement
//     bool gripper_success = VADERPlanner::gripper_plan_and_execute(fruit);
//     // bool gripper_success = true;
//     ROS_INFO_NAMED("do_pepper_plan", "Gripper planning and execution completed");

//     // Do cutter movement

//     bool cutter_success = VADERPlanner::cutter_plan_and_execute(peduncle);

//     // Conclude
//     ROS_INFO_NAMED("do_pepper_plan", "Dual arm execution completed");
//     ROS_INFO_NAMED("do_pepper_plan", "Gripper success: %s, Cutter success: %s",
//                    gripper_success ? "true" : "false", cutter_success ? "true" : "false");
//     // std::cout << "Done" << std::endl;
// }

// bool VADERPlanner::gripper_plan_and_execute(const vader_msgs::Fruit &fruit)
// {
//     ROS_INFO_NAMED("gripper_plan_and_execute", "Received new gripper planning request");
//     geometry_msgs::Pose target_pose = calculateFruitPregraspPose(fruit.pose);
//     target_pose.position.x = target_pose.position.x; // fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];
//     target_pose.position.y = target_pose.position.y - fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];
//     target_pose.position.z = target_pose.position.z; // + fruit.shape.dimensions[fruit.shape.CYLINDER_RADIUS];

//     group_gripper.setPoseTarget(target_pose);

//     bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (!success)
//     {
//         ROS_ERROR_NAMED("gripper_plan_and_execute", "Failed to plan for gripper");
//         return false;
//     }
//     show_trail(success);
//     bool exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     return exec_ok;
// }

// bool VADERPlanner::cutter_plan_and_execute(const vader_msgs::Peduncle &peduncle)
// {
//     ROS_INFO("Received new cutter planning request");
//     geometry_msgs::Pose target_pose = calculatePedunclePregraspPose(peduncle.pose);
//     target_pose.position.x = target_pose.position.x;                                                             // peduncle.shape.dimensions[peduncle.shape.CYLINDER_RADIUS];
//     target_pose.position.y = target_pose.position.y + peduncle.shape.dimensions[peduncle.shape.CYLINDER_RADIUS]; // + 0.05;
//     target_pose.position.z = target_pose.position.z + peduncle.shape.dimensions[peduncle.shape.CYLINDER_HEIGHT] / 3;

//     // TODO: Abhi: Add collision object for CUTTER ARM

//     group_cutter.setPoseTarget(target_pose);

//     bool success = (group_cutter.plan(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (!success)
//     {
//         ROS_ERROR_NAMED("gripper_plan_and_execute", "Failed to plan for gripper");
//         return false;
//     }
//     show_trail(success);
//     bool exec_ok = (group_cutter.execute(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     return exec_ok;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_dual_planner");

    VADERPlanner planner;

    planner.start();

    ros::waitForShutdown();
    return 0;
}
