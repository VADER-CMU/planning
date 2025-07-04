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

#include <geometric_shapes/shapes.h> // Include for cylinder shape
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <vader_msgs/SingleArmPlanRequest.h>
#include <vader_msgs/SingleArmExecutionRequest.h>
#include <vader_msgs/MoveToStorageRequest.h>
#include <vader_msgs/GoHomeRequest.h>

#include <cmath>
#include <iostream>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

const std::string PLAN_GROUP_GRIPPER_PREFIX = "xarm7";

namespace rvt = rviz_visual_tools;

class VADERPlanner
{
public:
    VADERPlanner() : spinner(SPINNER_THREAD_NUM),
                     PLANNING_GROUP_GRIPPER(PLAN_GROUP_GRIPPER_PREFIX),
                     group_gripper(PLAN_GROUP_GRIPPER_PREFIX)
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
    moveit::planning_interface::MoveGroupInterface group_gripper;
    moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
    moveit_visual_tools::MoveItVisualTools *visual_tools;

    geometry_msgs::Pose pregraspFinalGripperPose;

    uint8_t plan_type = 0;

    std::string PLANNING_GROUP_GRIPPER;

    ros::ServiceServer planning_service, execution_service;
    ros::ServiceServer move_to_storage_service;
    ros::ServiceServer go_home_service;

    ros::Publisher display_path;

    void init();
    void _add_ground_plane_collision();
    void _add_collision_wall(vader_msgs::SingleArmPlanRequest::Request &req);
    void _add_pepper_collision(vader_msgs::Pepper &pepper);
    void _clear_pepper_collision();
    void _add_storage_box_collision();

    bool _test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose);
    bool _test_PC_gripper_approach(tf::Vector3 &axis, tf::Vector3 &centroid, double angle, double radius);

    bool _plan_cartesian_gripper(geometry_msgs::Pose &goal_pose, double threshold);

    bool planGripperPregraspPose(vader_msgs::SingleArmPlanRequest::Request &req);
    bool planGripperGraspPose(vader_msgs::SingleArmPlanRequest::Request &req);

    bool planning_service_handler(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool execution_service_handler(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);
    bool move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res);
    bool go_home_service_handler(vader_msgs::GoHomeRequest::Request &req, vader_msgs::GoHomeRequest::Response &res);
    void show_trail(bool plan_result, bool is_planner);
};

void VADERPlanner::init()
{
    display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);

    visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");

    _add_ground_plane_collision();

    planning_service = node_handle.advertiseService("vader_plan", &VADERPlanner::planning_service_handler, this);
    execution_service = node_handle.advertiseService("vader_exec", &VADERPlanner::execution_service_handler, this);
    move_to_storage_service = node_handle.advertiseService("move_to_storage", &VADERPlanner::move_to_storage_service_handler, this);
    go_home_service = node_handle.advertiseService("go_home", &VADERPlanner::go_home_service_handler, this);

    // Initialize subscriber and publisher
    ROS_INFO("Planner initialized with planning group: %s",
             PLANNING_GROUP_GRIPPER.c_str());
}

void VADERPlanner::_add_ground_plane_collision()
{
    moveit_msgs::CollisionObject ground_plane;
    ground_plane.header.frame_id = group_gripper.getPlanningFrame();
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
    ground_pose.position.z = -0.01 - (ground_primitive.dimensions[2] / 2.0);
    ground_pose.orientation.w = 1.0; // No rotation

    ground_plane.primitives.push_back(ground_primitive);
    ground_plane.primitive_poses.push_back(ground_pose);
    ground_plane.operation = moveit_msgs::CollisionObject::ADD;

    // Add the ground plane to the planning scene
    planning_scene_interface.applyCollisionObject(ground_plane);
}

void VADERPlanner::_add_pepper_collision(vader_msgs::Pepper &pepper)
{
    moveit_msgs::CollisionObject cylinder_object;
    cylinder_object.header.frame_id = group_gripper.getPlanningFrame();
    cylinder_object.id = "pepper";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = pepper.fruit_data.shape.dimensions[0];
    primitive.dimensions[primitive.CYLINDER_RADIUS] = pepper.fruit_data.shape.dimensions[1];
    cylinder_object.primitives.push_back(primitive);
    cylinder_object.primitive_poses.push_back(pepper.fruit_data.pose);
    cylinder_object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyCollisionObject(cylinder_object);
}

void VADERPlanner::_add_collision_wall(vader_msgs::SingleArmPlanRequest::Request &req)
{
    moveit_msgs::CollisionObject wall_object;
    wall_object.header.frame_id = group_gripper.getPlanningFrame();
    wall_object.id = "wall";

    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);
    wall_primitive.dimensions[0] = 0.01; // Length in x-direction
    wall_primitive.dimensions[1] = 1.0;  // Width in y-direction
    wall_primitive.dimensions[2] = 1.0;  // Height in z-direction
    double ORIENTATION_X = 0;
    double ORIENTATION_Y = 0;
    double ORIENTATION_Z = -0.3826834;
    double ORIENTATION_W = 0.9238795;

    geometry_msgs::Pose wall_pose;
    wall_pose.position.x = req.pepper.fruit_data.pose.position.x - 0.2;
    wall_pose.position.y = req.pepper.fruit_data.pose.position.y;
    wall_pose.position.z = req.pepper.fruit_data.pose.position.z;
    wall_pose.orientation.x = ORIENTATION_X;
    wall_pose.orientation.y = ORIENTATION_Y;
    wall_pose.orientation.z = ORIENTATION_Z;
    wall_pose.orientation.w = ORIENTATION_W;

    wall_object.primitives.push_back(wall_primitive);
    wall_object.primitive_poses.push_back(wall_pose);
    wall_object.operation = moveit_msgs::CollisionObject::ADD;

    // double RADIUS_PLANT_CYL = 0.2;
    // double DIST_PLANT_CYL = 0.5;

    // // Add cylinders at 45-degree angles
    // for (int i = 0; i < 2; ++i)
    // {
    //     moveit_msgs::CollisionObject cylinder_object;
    //     cylinder_object.header.frame_id = group_gripper.getPlanningFrame();
    //     cylinder_object.id = "cylinder_" + std::to_string(i);

    //     shape_msgs::SolidPrimitive cylinder_primitive;
    //     cylinder_primitive.type = cylinder_primitive.CYLINDER;
    //     cylinder_primitive.dimensions.resize(2);
    //     cylinder_primitive.dimensions[0] = 1.0; // Height of the cylinder
    //     cylinder_primitive.dimensions[1] = RADIUS_PLANT_CYL; // Radius of the cylinder

    //     geometry_msgs::Pose cylinder_pose;
    //     double angle = (i == 0) ? M_PI / 4 : -M_PI / 4; // 45 degrees and -45 degrees
    //     cylinder_pose.position.x = wall_pose.position.x + DIST_PLANT_CYL * cos(angle);
    //     cylinder_pose.position.y = wall_pose.position.y + DIST_PLANT_CYL * sin(angle);
    //     cylinder_pose.position.z = wall_pose.position.z + 0.5; // Centered vertically (half the height)
    //     cylinder_pose.orientation.w = 1.0; // No rotation

    //     cylinder_object.primitives.push_back(cylinder_primitive);
    //     cylinder_object.primitive_poses.push_back(cylinder_pose);
    //     cylinder_object.operation = moveit_msgs::CollisionObject::ADD;

    //     planning_scene_interface.applyCollisionObject(cylinder_object);
    // }
    planning_scene_interface.applyCollisionObject(wall_object);
}

void VADERPlanner::_clear_pepper_collision()
{
    planning_scene_interface.removeCollisionObjects({"pepper"});
}

void VADERPlanner::_add_storage_box_collision()
{
    double center_x = 0.4;
    double center_y = 0.4;
    double center_z = 0.2;

    double box_length = 0.35;
    double box_width = 0.24;
    double box_height = 0.25;

    double ORIENTATION_X = 0;
    double ORIENTATION_Y = 0;
    double ORIENTATION_Z = 0.3826834;
    double ORIENTATION_W = 0.9238795;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Bottom face
    moveit_msgs::CollisionObject bottom_face;
    bottom_face.header.frame_id = group_gripper.getPlanningFrame();
    bottom_face.id = "storage_box_bottom";
    shape_msgs::SolidPrimitive bottom_primitive;
    bottom_primitive.type = bottom_primitive.BOX;
    bottom_primitive.dimensions = {box_length, box_width, 0.01};
    geometry_msgs::Pose bottom_pose;
    bottom_pose.position.x = center_x;
    bottom_pose.position.y = center_y;
    bottom_pose.position.z = center_z - (box_height / 2.0);
    bottom_pose.orientation.x = ORIENTATION_X;
    bottom_pose.orientation.y = ORIENTATION_Y;
    bottom_pose.orientation.z = ORIENTATION_Z;
    bottom_pose.orientation.w = ORIENTATION_W;
    bottom_face.primitives.push_back(bottom_primitive);
    bottom_face.primitive_poses.push_back(bottom_pose);
    bottom_face.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(bottom_face);

    // Front face
    moveit_msgs::CollisionObject front_face;
    front_face.header.frame_id = group_gripper.getPlanningFrame();
    front_face.id = "storage_box_front";
    shape_msgs::SolidPrimitive front_primitive;
    front_primitive.type = front_primitive.BOX;
    front_primitive.dimensions = {box_length, 0.01, box_height};
    geometry_msgs::Pose front_pose;
    front_pose.position.x = center_x + (box_width / 2.0) * 0.707;
    front_pose.position.y = center_y - (box_width / 2.0) * 0.707;
    front_pose.position.z = center_z;
    front_pose.orientation.x = ORIENTATION_X;
    front_pose.orientation.y = ORIENTATION_Y;
    front_pose.orientation.z = ORIENTATION_Z;
    front_pose.orientation.w = ORIENTATION_W;
    front_face.primitives.push_back(front_primitive);
    front_face.primitive_poses.push_back(front_pose);
    front_face.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(front_face);

    // Back face
    moveit_msgs::CollisionObject back_face;
    back_face.header.frame_id = group_gripper.getPlanningFrame();
    back_face.id = "storage_box_back";
    shape_msgs::SolidPrimitive back_primitive;
    back_primitive.type = back_primitive.BOX;
    back_primitive.dimensions = {box_length, 0.01, box_height};
    geometry_msgs::Pose back_pose;
    back_pose.position.x = center_x - (box_width / 2.0) * 0.707;
    back_pose.position.y = center_y + (box_width / 2.0) * 0.707;
    back_pose.position.z = center_z;
    back_pose.orientation.x = ORIENTATION_X;
    back_pose.orientation.y = ORIENTATION_Y;
    back_pose.orientation.z = ORIENTATION_Z;
    back_pose.orientation.w = ORIENTATION_W;
    back_face.primitives.push_back(back_primitive);
    back_face.primitive_poses.push_back(back_pose);
    back_face.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(back_face);

    // Left face
    moveit_msgs::CollisionObject left_face;
    left_face.header.frame_id = group_gripper.getPlanningFrame();
    left_face.id = "storage_box_left";
    shape_msgs::SolidPrimitive left_primitive;
    left_primitive.type = left_primitive.BOX;
    left_primitive.dimensions = {0.01, box_width, box_height};
    geometry_msgs::Pose left_pose;
    left_pose.position.x = center_x - (box_length / 2.0) * 0.707;
    left_pose.position.y = center_y - (box_length / 2.0) * 0.707;
    left_pose.position.z = center_z;
    left_pose.orientation.x = ORIENTATION_X;
    left_pose.orientation.y = ORIENTATION_Y;
    left_pose.orientation.z = ORIENTATION_Z;
    left_pose.orientation.w = ORIENTATION_W;
    left_face.primitives.push_back(left_primitive);
    left_face.primitive_poses.push_back(left_pose);
    left_face.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(left_face);

    // Right face
    moveit_msgs::CollisionObject right_face;
    right_face.header.frame_id = group_gripper.getPlanningFrame();
    right_face.id = "storage_box_right";
    shape_msgs::SolidPrimitive right_primitive;
    right_primitive.type = right_primitive.BOX;
    right_primitive.dimensions = {0.01, box_width, box_height};
    geometry_msgs::Pose right_pose;
    right_pose.position.x = center_x + (box_length / 2.0) * 0.707;
    right_pose.position.y = center_y + (box_length / 2.0) * 0.707;
    right_pose.position.z = center_z;
    right_pose.orientation.x = ORIENTATION_X;
    right_pose.orientation.y = ORIENTATION_Y;
    right_pose.orientation.z = ORIENTATION_Z;
    right_pose.orientation.w = ORIENTATION_W;
    right_face.primitives.push_back(right_primitive);
    right_face.primitive_poses.push_back(right_pose);
    right_face.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(right_face);

    planning_scene_interface.applyCollisionObjects(collision_objects);
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

void VADERPlanner::show_trail(bool plan_result, bool is_planner)
{
    if (plan_result)
    {
        ROS_INFO_NAMED("vader_planner", "Visualizing plan as trajectory line");

        visual_tools->deleteAllMarkers();
        const robot_state::JointModelGroup *joint_model_group_gripper = group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        visual_tools->publishTrajectoryLine(plan_gripper.trajectory_, joint_model_group_gripper);
        visual_tools->trigger();
    }
}

bool VADERPlanner::_plan_cartesian_gripper(geometry_msgs::Pose &goal_pose, double threshold)
{
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(goal_pose);
    group_gripper.setMaxVelocityScalingFactor(maxV_scale_factor);
    moveit_msgs::RobotTrajectory trajectory;

    double fraction = group_gripper.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    plan_gripper.trajectory_ = trajectory;
    fprintf(stderr, "Gripper cartesian plan coverage: %lf\n", fraction);

    if (fraction < threshold)
    {
        ROS_ERROR("Cartesian plan coverage lower than threshold!");
        return false;
    }
    return true;
}

bool VADERPlanner::planning_service_handler(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{
    // Gripper
    plan_type = req.mode; // Store which plan is being used for gripper and check this when executing
    bool success;
    switch (plan_type)
    {
    case req.GRIPPER_PREGRASP_PLAN:
    {
        success = planGripperPregraspPose(req);
        break;
    }
    case req.GRIPPER_GRASP_PLAN:
    {
        success = planGripperGraspPose(req);
        break;
    }
    }

    res.result = success;
    return success;
}

bool VADERPlanner::execution_service_handler(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res)
{
    // Gripper
    if (plan_type != req.mode)
    {
        ROS_ERROR("The plan type does not match the execution type. Aborting execution");
        res.result = false;
        return false;
    }
    bool exec_ok;
    if (plan_type == req.GRIPPER_GRASP_EXEC)
    {
        _clear_pepper_collision();

        // Execute the plan

        exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (exec_ok)
        {
            ROS_INFO("Successfully moved to grasp position");
            geometry_msgs::Pose current_pose = pregraspFinalGripperPose;

            tf::Vector3 approach(0.0, 0.0, 0.2);

            tf::Quaternion curr_quat;
            tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
            tf::Matrix3x3 curr_rot(curr_quat);

            tf::Vector3 transformed_approach = curr_rot * approach;

            current_pose.position.x += transformed_approach.x();
            current_pose.position.y += transformed_approach.y();
            current_pose.position.z += transformed_approach.z();
            // moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
            // const robot_state::JointModelGroup *joint_model_group2 = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
            // bool _found = state_copy.setFromIK(joint_model_group2, current_pose, 10, 0.1);
            // assert(_found);

            // std::vector<double> joint_values;

            // state_copy.copyJointGroupPositions(joint_model_group2, joint_values);

            // group_gripper.setJointValueTarget(joint_values);

            // exec_ok = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            bool cartesian_plan_success = _plan_cartesian_gripper(current_pose, 0.5);

            show_trail(cartesian_plan_success, true);
            if (cartesian_plan_success)
            {
                exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                geometry_msgs::PoseStamped current_pose_stamped = group_gripper.getCurrentPose();
                geometry_msgs::Pose current_pose = current_pose_stamped.pose;
                ROS_INFO("Current Gripper Position: x=%f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
                ROS_INFO("Current Gripper Orientation: x=%f, y=%f, z=%f, w=%f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
            } else {
                exec_ok = false;
            }
        }
    }
    else if (plan_type == req.GRIPPER_PREGRASP_EXEC)
    {
        exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    else
    {
        ROS_ERROR("Why are we here");
    }

    res.result = exec_ok;
    return exec_ok;
}

bool VADERPlanner::move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res)
{
    // Extract bin location from the request and store it in a pose message
    // Use a preset joint configuration for the storage bin
    std::vector<double> joint_values = {-84, -36.1, 89, 119.8, 40.5, 114.9, -114.1}; // Example joint configuration
    for (int i = 0; i < 7; i++)
    {
        joint_values[i] *= (M_PI / 180.0);
        joint_values[i] = fmod((joint_values[i] + M_PI), (2 * M_PI)) - M_PI;
        ROS_INFO("%f", joint_values[i]);
    }
    group_gripper.setJointValueTarget(joint_values);
    bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    show_trail(success, true);
    if (success)
    {
        ROS_INFO("Plan to storage location succeeded. Executing...");
        bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!exec_result)
        {
            ROS_ERROR("Execution to storage location failed");
            res.result = false;
            return false;
        }
        
        geometry_msgs::PoseStamped current_pose_stamped = group_gripper.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_stamped.pose;
        double downward_dist = 0.25;
        current_pose.position.z -= downward_dist;
        bool success = _plan_cartesian_gripper(current_pose, 0.5);
        if (!success)
        {
            ROS_ERROR("Cartesian path planning failed");
            res.result = false;
            return false;
        }

        ROS_INFO("Executing downward Cartesian path...");
        exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!exec_result)
        {
            ROS_ERROR("Execution of downward Cartesian path failed");
            res.result = false;
            return false;
        }

        res.result = true;
        return true;
    }
    else
    {
        ROS_ERROR("Plan to storage location failed.");
        res.result = false;
        return false;
    }
}

bool VADERPlanner::go_home_service_handler(vader_msgs::GoHomeRequest::Request &req, vader_msgs::GoHomeRequest::Response &res)
{   //{57.3,-78.9,4.7,55.2,35.3,78.7,57.7} to right side, lower
    std::vector<double> joint_values = {103.8,-76.2,-1.5,77.8,28.6,84.6,64.7};//{72,-53.9,-40.2,95.1,64.1,140.2,45.8};
    for (int i = 0; i < 7; i++)
    {
        joint_values[i] *= (M_PI / 180.0);
        joint_values[i] = fmod((joint_values[i] + M_PI), (2 * M_PI)) - M_PI;
        ROS_INFO("%f", joint_values[i]);
    }
    group_gripper.setJointValueTarget(joint_values);
    bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    show_trail(success, true);
    if (success)
    {
        ROS_INFO("Plan to storage location succeeded. Executing...");
        bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!exec_result)
        {
            ROS_ERROR("Execution to storage location failed");
        }
        res.result = exec_result;
        return exec_result;
    }
    else
    {
        ROS_ERROR("Plan to storage location failed.");
        res.result = false;
        return false;
    }
}

static tf::Quaternion _get_norm_quat_from_axes(tf::Vector3 &ax_x, tf::Vector3 &ax_y, tf::Vector3 &ax_z)
{
    // Create rotation matrix for end effector orientation
    tf::Matrix3x3 rot_matrix;
    rot_matrix.setValue(
        ax_x.x(), ax_y.x(), ax_z.x(),
        ax_x.y(), ax_y.y(), ax_z.y(),
        ax_x.z(), ax_y.z(), ax_z.z());

    // Convert rotation matrix to quaternion
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

bool VADERPlanner::_test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose)
{
    moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
    const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);

    bool success = state_copy.setFromIK(joint_model_group, test_pose, 10, 0.1);
    return success;
}

// Uses Parametric Circle method to get the viable pregrasp pose and plan of the gripper.
bool VADERPlanner::planGripperPregraspPose(vader_msgs::SingleArmPlanRequest::Request &req)
{
    _add_storage_box_collision();
    // display pepper as collision obj
    _add_pepper_collision(req.pepper);
    // Add collision wall to the scene
    _add_collision_wall(req);

    tf::Quaternion pepper_quat;
    tf::quaternionMsgToTF(req.pepper.fruit_data.pose.orientation, pepper_quat);

    tf::Vector3 pepper_axis = tf::quatRotate(pepper_quat, tf::Vector3(0, 0, 1)).normalized();

    tf::Vector3 pepper_centroid(
        req.pepper.fruit_data.pose.position.x,
        req.pepper.fruit_data.pose.position.y,
        req.pepper.fruit_data.pose.position.z);

    double radius = req.reserve_dist;

    // Generate two orthonormal vectors u,v that are perpendicular to pepper_axis
    tf::Vector3 u, v;

    // Find first perpendicular vector u
    tf::Vector3 ref(0, 0, 1);
    if (std::abs(pepper_axis.dot(ref)) > 0.9)
    {
        // If cylinder axis is nearly vertical, use a different reference
        ref = tf::Vector3(1, 0, 0);
    }

    u = pepper_axis.cross(ref).normalized();
    v = pepper_axis.cross(u).normalized();

    // Calculate the values A and B
    double A = -pepper_centroid.dot(u); // Note the negative signs as per the formula
    double B = -pepper_centroid.dot(v);

    // Calculate theta_min to find the closest point to origin
    double theta_min = atan2(B, A);

    // Calculate the closest point on the parametric circle
    tf::Vector3 closest_point = pepper_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);

    ROS_INFO("Closest point on circle: (%f, %f, %f)",
             closest_point.x(), closest_point.y(), closest_point.z());

    // Calculate end effector orientation
    //  The gripper z-axis should point from the goal point to the cylinder centroid
    tf::Vector3 ee_z = (pepper_centroid - closest_point).normalized();

    // Ensure end effector y-axis is perpendicular to cylinder axis
    tf::Vector3 ee_y = pepper_axis.cross(ee_z).normalized();

    // Calculate end effector x-axis to complete right-handed coordinate system
    tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

    tf::Quaternion ee_quat = _get_norm_quat_from_axes(ee_x, ee_y, ee_z);

    // Set end effector pose
    geometry_msgs::Pose end_effector_pose = _get_pose_from_pos_and_quat(closest_point, ee_quat);

    // Log end effector pose
    ROS_INFO("End effector position: (%f, %f, %f)", end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
    ROS_INFO("End effector orientation: (%f, %f, %f, %f)", end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w);

    // group_gripper.setPoseTarget(end_effector_pose);

    bool found_ik = _test_IK_for_gripper_pose(end_effector_pose);

    if (!found_ik)
    {

        ROS_WARN("Initial IK solution failed, trying alternative points on the parametric circle");
        // Try different angles around the parametric circle in 30-degree increments
        for (int i = 1; i <= 11 && !found_ik; i++)
        {
            // Try i*30 degrees away from the optimal angle
            double test_angle = theta_min + i * (M_PI / 6);

            // Calculate test point
            tf::Vector3 test_point = pepper_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

            // Calculate orientation (gripper z-axis still points at cylinder centroid)
            tf::Vector3 test_ee_z = (pepper_centroid - test_point).normalized();
            tf::Vector3 test_ee_y = pepper_axis.cross(test_ee_z).normalized();

            // Check if the cross product result is valid (non-zero length)
            if (test_ee_y.length() < 0.1)
            {
                // If ee_z is nearly parallel to pepper_axis, use different approach
                tf::Vector3 world_up(0, 0, 1);
                test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : world_up.cross(test_ee_z).normalized();
            }

            tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

            // Create rotation matrix and convert to quaternion
            tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
            geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(closest_point, test_quat);

            // Try IK for this pose
            found_ik = _test_IK_for_gripper_pose(test_pose);

            if (found_ik)
            {
                ROS_INFO("Found IK solution at alternative angle: theta = %.2f radians (%.2f degrees from optimal)", test_angle, i * 30.0);
                end_effector_pose = test_pose;
                // group_gripper.setPoseTarget(end_effector_pose);
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
                tf::Vector3 test_point = pepper_centroid + test_radius * (cos(theta_min) * u + sin(theta_min) * v);

                // Calculate orientation
                tf::Vector3 test_ee_z = (pepper_centroid - test_point).normalized();
                tf::Vector3 test_ee_y = pepper_axis.cross(test_ee_z).normalized();
                tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

                // Create rotation matrix and convert to quaternion
                tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
                geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(closest_point, test_quat);

                // Try IK for this pose
                found_ik = _test_IK_for_gripper_pose(test_pose);

                if (found_ik)
                {
                    ROS_INFO("Found IK solution with adjusted radius: %.2f m", test_radius);
                    end_effector_pose = test_pose;
                    // group_gripper.setPoseTarget(end_effector_pose);
                    break;
                }
            }
        }
    }
    bool success;
    if (found_ik)
    {
        end_effector_pose.position.z += 0.08;
        // moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
        // const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
        // bool _found = state_copy.setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
        // assert(_found);

        // std::vector<double> joint_values;
        // state_copy.copyJointGroupPositions(joint_model_group, joint_values);
        // ROS_INFO("Found valid IK solution");

        // // Add 90-deg cam rotation
        if (req.gripper_camera_rotation == req.GRIPPER_DO_ROTATE_CAMERA)
        {
            //     int num_joints = joint_values.size();
            //     gripper_pregrasp_cam_orig_value = joint_values[num_joints - 1];
            //     if (num_joints > 0)
            //     {

            //         // Rotate only the final joint by 90 degrees
            //         joint_values[num_joints - 1] -= M_PI / 2.0;

            //         // Normalize the joint angle to the range (-PI, PI) if needed
            //         while (joint_values[num_joints - 1] > M_PI)
            //         {
            //             joint_values[num_joints - 1] -= 2.0 * M_PI;
            //         }
            //         while (joint_values[num_joints - 1] < -M_PI)
            //         {
            //             joint_values[num_joints - 1] += 2.0 * M_PI;
            //         }

            //         ROS_INFO("Final joint before rotation: %f, after rotation: %f radians",
            //                  joint_values[num_joints - 1] - M_PI / 2.0, joint_values[num_joints - 1]);
            //     }
            tf::Quaternion curr_ori;
            tf::quaternionMsgToTF(end_effector_pose.orientation, curr_ori);
            tf::Quaternion rotate_90;
            rotate_90.setRPY(0, 0, -M_PI / 2);
            tf::Quaternion new_ori = curr_ori * rotate_90;
            new_ori.normalize();
            end_effector_pose.orientation.x = new_ori.x();
            end_effector_pose.orientation.y = new_ori.y();
            end_effector_pose.orientation.z = new_ori.z();
            end_effector_pose.orientation.w = new_ori.w();
        }

        // pregraspFinalGripperPose = end_effector_pose;

        // group_gripper.setJointValueTarget(joint_values);
        // success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // show_trail(success, true);
        bool cartesian_plan_success = _plan_cartesian_gripper(end_effector_pose, 0.5);
        show_trail(cartesian_plan_success, true);
        success = cartesian_plan_success;
    }
    else
    {
        ROS_ERROR("Could not find IK solution for any tested pose");
        success = false;
    }
    ROS_INFO("Finished planning");
    return success;
}

bool VADERPlanner::planGripperGraspPose(vader_msgs::SingleArmPlanRequest::Request &req)
{

    // display pepper as collision obj
    // _add_pepper_collision(req.pepper);
    _clear_pepper_collision();

    tf::Quaternion pepper_quat;
    tf::quaternionMsgToTF(req.pepper.fruit_data.pose.orientation, pepper_quat);

    tf::Vector3 pepper_axis = tf::quatRotate(pepper_quat, tf::Vector3(0, 0, 1)).normalized();

    tf::Vector3 pepper_centroid(
        req.pepper.fruit_data.pose.position.x,
        req.pepper.fruit_data.pose.position.y,
        req.pepper.fruit_data.pose.position.z);

    double radius = req.reserve_dist;

    // Generate two orthonormal vectors u,v that are perpendicular to pepper_axis
    tf::Vector3 u, v;

    // Find first perpendicular vector u
    tf::Vector3 ref(0, 0, 1);
    if (std::abs(pepper_axis.dot(ref)) > 0.9)
    {
        // If cylinder axis is nearly vertical, use a different reference
        ref = tf::Vector3(1, 0, 0);
    }

    u = pepper_axis.cross(ref).normalized();
    v = pepper_axis.cross(u).normalized();

    // Calculate the values A and B
    double A = -pepper_centroid.dot(u); // Note the negative signs as per the formula
    double B = -pepper_centroid.dot(v);

    // Calculate theta_min to find the closest point to origin
    double theta_min = atan2(B, A);

    // Calculate the closest point on the parametric circle
    tf::Vector3 closest_point = pepper_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);

    ROS_INFO("Dot with world Z: %f", std::abs(pepper_axis.dot(ref)));
    ROS_INFO("Closest point on circle: (%f, %f, %f)",
             closest_point.x(), closest_point.y(), closest_point.z());

    // Calculate end effector orientation
    //  The gripper z-axis should point from the goal point to the cylinder centroid
    tf::Vector3 ee_z = (pepper_centroid - closest_point).normalized();

    // Ensure end effector y-axis is perpendicular to cylinder axis
    tf::Vector3 ee_y = pepper_axis.cross(ee_z).normalized();

    // Calculate end effector x-axis to complete right-handed coordinate system
    tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

    tf::Quaternion ee_quat = _get_norm_quat_from_axes(ee_x, ee_y, ee_z);

    // Set end effector pose
    geometry_msgs::Pose end_effector_pose = _get_pose_from_pos_and_quat(closest_point, ee_quat);

    // Log end effector pose
    ROS_INFO("End effector position: (%f, %f, %f)", end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
    ROS_INFO("End effector orientation: (%f, %f, %f, %f)", end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w);

    // group_gripper.setPoseTarget(end_effector_pose);

    bool found_ik = _test_IK_for_gripper_pose(end_effector_pose);

    if (!found_ik)
    {

        ROS_WARN("Initial IK solution failed, trying alternative points on the parametric circle");
        // Try different angles around the parametric circle in 30-degree increments
        for (int i = 1; i <= 11 && !found_ik; i++)
        {
            // Try i*30 degrees away from the optimal angle
            double test_angle = theta_min + i * (M_PI / 6);

            // Calculate test point
            tf::Vector3 test_point = pepper_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

            // Calculate orientation (gripper z-axis still points at cylinder centroid)
            tf::Vector3 test_ee_z = (pepper_centroid - test_point).normalized();
            tf::Vector3 test_ee_y = pepper_axis.cross(test_ee_z).normalized();

            // Check if the cross product result is valid (non-zero length)
            if (test_ee_y.length() < 0.1)
            {
                // If ee_z is nearly parallel to pepper_axis, use different approach
                tf::Vector3 world_up(0, 0, 1);
                test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : world_up.cross(test_ee_z).normalized();
            }

            tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

            // Create rotation matrix and convert to quaternion
            tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
            geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(closest_point, test_quat);

            // Try IK for this pose
            found_ik = _test_IK_for_gripper_pose(test_pose);

            if (found_ik)
            {
                ROS_INFO("Found IK solution at alternative angle: theta = %.2f radians (%.2f degrees from optimal)", test_angle, i * 30.0);
                end_effector_pose = test_pose;
                break;
            }
        }
    }
    bool success;
    if (found_ik)
    {
        // moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
        // const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
        // bool _found = state_copy.setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
        // assert(_found);

        // std::vector<double> joint_values;
        // state_copy.copyJointGroupPositions(joint_model_group, joint_values);
        // ROS_INFO("Found valid IK solution");

        // pregraspFinalGripperPose = end_effector_pose;

        // group_gripper.setJointValueTarget(joint_values);
        // success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // show_trail(success, true);
        bool cartesian_plan_success = _plan_cartesian_gripper(end_effector_pose, 0.5);
        pregraspFinalGripperPose = end_effector_pose;
        show_trail(cartesian_plan_success, true);
        success = cartesian_plan_success;
    }
    else
    {
        ROS_ERROR("Could not find IK solution for any tested pose");
        success = false;
    }
    ROS_INFO("Finished planning");
    return success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_single_planner");

    VADERPlanner planner;

    planner.start();

    ros::waitForShutdown();
    return 0;
}
