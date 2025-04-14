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
\
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

    geometry_msgs::Pose pregraspFinalGripperPose;

    uint8_t plan_type = 0;
    double gripper_pregrasp_cam_rotated_amount = 0.0;

    std::string PLANNING_GROUP_GRIPPER;
    std::string PLANNING_GROUP_CUTTER;

    ros::ServiceServer planning_service, execution_service;
    ros::ServiceServer move_to_storage_service;

    ros::Publisher display_path;

    void init();
    void _add_ground_plane_collision();
    void _add_pepper_collision(vader_msgs::Pepper &pepper);

    bool _test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose);
    bool _test_PC_gripper_approach(tf::Vector3 &axis, tf::Vector3 &centroid, double angle, double radius);
    bool planGripperPregraspPose(vader_msgs::BimanualPlanRequest::Request &req);
    bool planGripperGraspPose(vader_msgs::BimanualPlanRequest::Request &req);
    bool planCutterGraspPose(vader_msgs::BimanualPlanRequest::Request &req);

    bool planning_service_handler(vader_msgs::BimanualPlanRequest::Request &req, vader_msgs::BimanualPlanRequest::Response &res);
    bool execution_service_handler(vader_msgs::BimanualExecRequest::Request &req, vader_msgs::BimanualExecRequest::Response &res);
    bool move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res);
    void show_trail(bool plan_result);
};

void VADERPlanner::init()
{
    //TODO joint names?
    std::vector<std::string> gripper_joint_names = {
        "L_joint1", "L_joint2", "L_joint3", "L_joint4", "L_joint5", "L_joint6", "L_joint7"
    };
    std::vector<std::string> cutter_joint_names = {
        "R_joint1", "R_joint2", "R_joint3", "R_joint4", "R_joint5", "R_joint6", "R_joint7"
    };

    group_gripper.setJointNames(gripper_joint_names);
    group_cutter.setJointNames(cutter_joint_names);
    display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true); /*necessary?*/

    visual_tools = new moveit_visual_tools::MoveItVisualTools("L_link_base");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.8;

    _add_ground_plane_collision();

    planning_service = node_handle.advertiseService("vader_plan", &VADERPlanner::planning_service_handler, this);
    execution_service = node_handle.advertiseService("vader_exec", &VADERPlanner::execution_service_handler, this);
    move_to_storage_service = node_handle.advertiseService("move_to_storage", &VADERPlanner::move_to_storage_service_handler, this);

    // Initialize subscriber and publisher
    ROS_INFO("Planner initialized with left planning group: %s and right planning group: %s",
             PLANNING_GROUP_GRIPPER.c_str(), PLANNING_GROUP_CUTTER.c_str());
}

void VADERPlanner::_add_ground_plane_collision() {
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

void VADERPlanner::_add_pepper_collision(vader_msgs::Pepper &pepper) {
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
        bool success;
        switch(plan_type){
            case req.GRIPPER_PREGRASP_PLAN: {
                success = planGripperPregraspPose(req);
                break;
            }
            case req.GRIPPER_GRASP_PLAN: {
                success = planGripperGraspPose(req);
                break;
            }
        }
        
        res.result = success;
        return success;
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

        if (plan_type == req.GRIPPER_GRASP_EXEC) {
            planning_scene_interface.removeCollisionObjects({"pepper"});
        }

        bool exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        res.result = exec_ok;
        return exec_ok;
    }
}

bool VADERPlanner::move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res){
    // Extract bin location from the request and store it in a pose message
    geometry_msgs::Pose bin_location_pose;
    bin_location_pose.position.x = req.binLocation.x;
    bin_location_pose.position.y = req.binLocation.y;
    bin_location_pose.position.z = req.binLocation.z + req.reserve_dist;
    
    // Set the orientation to point in the negative z direction
    bin_location_pose.orientation.x = 0.0;
    bin_location_pose.orientation.y = 1.0;
    bin_location_pose.orientation.z = 0.0;
    bin_location_pose.orientation.w = 0.0;
    
    // Log the bin location for debugging
    ROS_INFO("Bin location received: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
             bin_location_pose.position.x, bin_location_pose.position.y, bin_location_pose.position.z,
             bin_location_pose.orientation.x, bin_location_pose.orientation.y, bin_location_pose.orientation.z, bin_location_pose.orientation.w);
    
    group_gripper.setPoseTarget(bin_location_pose);
    bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Plan to storage location succeeded. Executing...");
        bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!exec_result) {
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

// static tf::Vector3 _calculate_closest_PC_point(tf::Vector3 &axis, tf::Vector3 &centroid, double radius) {
//     // Generate two orthonormal vectors u,v that are perpendicular to pepper_axis
//     tf::Vector3 u, v;

//     // Find first perpendicular vector u
//     tf::Vector3 ref(0, 0, 1);
//     if (std::abs(axis.dot(ref)) > 0.9)
//     {
//     // If cylinder axis is nearly vertical, use a different reference
//     ref = tf::Vector3(1, 0, 0);
//     }

//     u = axis.cross(ref).normalized();
//     v = axis.cross(u).normalized();

//     // Calculate the values A and B
//     double A = -centroid.dot(u); // Note the negative signs as per the formula
//     double B = -centroid.dot(v);

//     // Calculate theta_min to find the closest point to origin
//     double theta_min = atan2(B, A);

//     // Calculate the closest point on the parametric circle
//     tf::Vector3 closest_point = pepper_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);
//     return closest_point;
// }

static tf::Quaternion _get_norm_quat_from_axes(tf::Vector3 &ax_x, tf::Vector3 &ax_y, tf::Vector3 &ax_z) {
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

static geometry_msgs::Pose _get_pose_from_pos_and_quat(tf::Vector3 &pos, tf::Quaternion &quat) {
    geometry_msgs::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();
    tf::quaternionTFToMsg(quat, pose.orientation);
    return pose;
}

bool VADERPlanner::_test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose){
    moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
    const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    bool success = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);
    return success;
}

// bool VADERPlanner::_test_PC_gripper_approach(tf::Vector3 &axis, tf::Vector3 &centroid, double angle, double radius){
//     // Generate two orthonormal vectors u,v that are perpendicular to pepper_axis
//     tf::Vector3 u, v;

//     // Find first perpendicular vector u
//     tf::Vector3 ref(0, 0, 1);
//     if (std::abs(axis.dot(ref)) > 0.9)
//     {
//     // If cylinder axis is nearly vertical, use a different reference
//     ref = tf::Vector3(1, 0, 0);
//     }

//     u = axis.cross(ref).normalized();
//     v = axis.cross(u).normalized();

//     // Calculate the values A and B
//     double A = -centroid.dot(u); // Note the negative signs as per the formula
//     double B = -centroid.dot(v);

//     // Calculate theta_min to find the closest point to origin
//     double theta_min = atan2(B, A);

//     // Calculate the closest point on the parametric circle
//     tf::Vector3 closest_point = pepper_centroid + radius * (cos(theta_min) * u + sin(theta_min) * v);
//     return closest_point;
// }

// Uses Parametric Circle method to get the viable pregrasp pose and plan of the gripper.
bool VADERPlanner::planGripperPregraspPose(vader_msgs::BimanualPlanRequest::Request &req) {

    //display pepper as collision obj
    _add_pepper_collision(req.pepper);

    tf::Quaternion pepper_quat;
    tf::quaternionMsgToTF(req.pepper.fruit_data.pose.orientation, pepper_quat);

    tf::Vector3 pepper_axis = tf::quatRotate(pepper_quat, tf::Vector3(0, 0, 1)).normalized();

    tf::Vector3 pepper_centroid(
        req.pepper.fruit_data.pose.position.x,
        req.pepper.fruit_data.pose.position.y,
        req.pepper.fruit_data.pose.position.z
    );

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


    //Calculate end effector orientation
    // The gripper z-axis should point from the goal point to the cylinder centroid
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

    group_gripper.setPoseTarget(end_effector_pose);

    bool found_ik = _test_IK_for_gripper_pose(end_effector_pose);
    
    if(!found_ik) {

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
                group_gripper.setPoseTarget(end_effector_pose);
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
            group_gripper.setPoseTarget(end_effector_pose);
            break;
            }
        }
        }
    }
    bool success;
    if(found_ik){
        moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
        const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        current_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
      
        std::vector<double> joint_values;
        current_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found valid IK solution");

        //Add 90-deg cam rotation
        if(req.gripper_camera_rotation == req.GRIPPER_DO_ROTATE_CAMERA) {
            int num_joints = joint_values.size();
            if (num_joints > 0)
            {
                // Rotate only the final joint by 90 degrees
                joint_values[num_joints - 1] += M_PI / 2.0;

                // Normalize the joint angle to the range (-PI, PI) if needed
                while (joint_values[num_joints - 1] > M_PI)
                {
                    joint_values[num_joints - 1] -= 2.0 * M_PI;
                }
                while (joint_values[num_joints - 1] < -M_PI)
                {
                    joint_values[num_joints - 1] += 2.0 * M_PI;
                }

                ROS_INFO("Final joint before rotation: %f, after rotation: %f radians",
                    joint_values[num_joints - 1] - M_PI / 2.0, joint_values[num_joints - 1]);
            }
            gripper_pregrasp_cam_rotated_amount = M_PI / 2.0;
        }

        pregraspFinalGripperPose = end_effector_pose;
    
        group_gripper.setJointValueTarget(joint_values);
        success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    }else{
        ROS_ERROR("Could not find IK solution for any tested pose");
        success = false;
    }
    ROS_INFO("Finished planning");
    return success;
}

bool VADERPlanner::planCutterGraspPose(vader_msgs::BimanualPlanRequest::Request &req){
    // Add peduncle as collision object (if needed)
    // _add_peduncle_collision(req.pepper.peduncle_data);  // This function would need to be implemented

    tf::Quaternion peduncle_quat;
    tf::quaternionMsgToTF(req.pepper.peduncle_data.pose.orientation, peduncle_quat);

    tf::Vector3 peduncle_axis = tf::quatRotate(peduncle_quat, tf::Vector3(0, 0, 1)).normalized();

    tf::Vector3 peduncle_centroid(
        req.pepper.peduncle_data.pose.position.x,
        req.pepper.peduncle_data.pose.position.y,
        req.pepper.peduncle_data.pose.position.z
    );

    double radius = req.reserve_dist;

    // Generate two orthonormal vectors u,v that are perpendicular to peduncle_axis
    tf::Vector3 u, v;

    // Find first perpendicular vector u
    tf::Vector3 ref(0, 0, 1);
    if (std::abs(peduncle_axis.dot(ref)) > 0.9)
    {
        // If cylinder axis is nearly vertical, use a different reference
        ref = tf::Vector3(1, 0, 0);
    }

    u = peduncle_axis.cross(ref).normalized();
    v = peduncle_axis.cross(u).normalized();

    // Calculate the values A and B
    double A = -peduncle_centroid.dot(u); // Note the negative signs as per the formula
    double B = -peduncle_centroid.dot(v);

    // Calculate theta_min to find the closest point to origin
    double theta_min = atan2(B, A);
    
    // Add offset to position the cutter opposite to the gripper
    // This will position the cutter approximately 180 degrees from where the gripper would be
    double theta_offset = M_PI;  // 180 degrees
    double theta_cutter = theta_min + theta_offset;

    // Calculate the closest point on the parametric circle
    tf::Vector3 cutter_point = peduncle_centroid + radius * (cos(theta_cutter) * u + sin(theta_cutter) * v);

    ROS_INFO("Cutter approach point on circle: (%f, %f, %f)",
        cutter_point.x(), cutter_point.y(), cutter_point.z());

    // Calculate end effector orientation
    // The cutter z-axis should point from the goal point to the peduncle centroid
    tf::Vector3 ee_z = (peduncle_centroid - cutter_point).normalized();

    // Position the cutter to cut the peduncle near its base (closer to pepper)
    // Move the cutting point a bit along the peduncle axis towards the pepper
    tf::Vector3 cutting_point = peduncle_centroid + 
                               (req.pepper.peduncle_data.shape.dimensions[0] * 0.25) * (-peduncle_axis);

    // Ensure end effector y-axis is perpendicular to peduncle axis
    tf::Vector3 ee_y = peduncle_axis.cross(ee_z).normalized();

    // Check if the cross product result is valid (non-zero length)
    if (ee_y.length() < 0.1)
    {
        // If ee_z is nearly parallel to peduncle_axis, use different approach
        tf::Vector3 world_up(0, 0, 1);
        ee_y = (std::abs(ee_z.dot(world_up)) > 0.9) ? 
               tf::Vector3(1, 0, 0).cross(ee_z).normalized() : 
               world_up.cross(ee_z).normalized();
    }

    // Calculate end effector x-axis to complete right-handed coordinate system
    tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

    tf::Quaternion ee_quat = _get_norm_quat_from_axes(ee_x, ee_y, ee_z);

    // Set end effector pose
    geometry_msgs::Pose end_effector_pose = _get_pose_from_pos_and_quat(cutter_point, ee_quat);

    // Log end effector pose
    ROS_INFO("Cutter end effector position: (%f, %f, %f)", 
             end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
    ROS_INFO("Cutter end effector orientation: (%f, %f, %f, %f)", 
             end_effector_pose.orientation.x, end_effector_pose.orientation.y, 
             end_effector_pose.orientation.z, end_effector_pose.orientation.w);

    group_cutter.setPoseTarget(end_effector_pose);

    // Test if IK solution exists for this pose
    moveit::core::RobotStatePtr current_state = group_cutter.getCurrentState();
    const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_CUTTER);
    bool found_ik = current_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
    
    if(!found_ik) {
        ROS_WARN("Initial IK solution failed for cutter, trying alternative points on the parametric circle");
        
        // Try different angles around the parametric circle in 30-degree increments
        for (int i = 1; i <= 11 && !found_ik; i++)
        {
            // Try i*30 degrees away from the optimal angle
            double test_angle = theta_cutter + i * (M_PI / 6);

            // Calculate test point
            tf::Vector3 test_point = peduncle_centroid + radius * (cos(test_angle) * u + sin(test_angle) * v);

            // Calculate orientation (cutter z-axis still points at peduncle centroid)
            tf::Vector3 test_ee_z = (peduncle_centroid - test_point).normalized();
            tf::Vector3 test_ee_y = peduncle_axis.cross(test_ee_z).normalized();

            // Check if the cross product result is valid (non-zero length)
            if (test_ee_y.length() < 0.1)
            {
                // If ee_z is nearly parallel to peduncle_axis, use different approach
                tf::Vector3 world_up(0, 0, 1);
                test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? 
                           tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : 
                           world_up.cross(test_ee_z).normalized();
            }

            tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

            // Create rotation matrix and convert to quaternion
            tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
            geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(test_point, test_quat);

            // Try IK for this pose
            found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

            if (found_ik)
            {
                ROS_INFO("Found cutter IK solution at alternative angle: theta = %.2f radians (%.2f degrees from optimal)", 
                         test_angle, i * 30.0);
                end_effector_pose = test_pose;
                group_cutter.setPoseTarget(end_effector_pose);
                cutter_point = test_point;
                break;
            }
        }

        // If circle positions fail, try with adjusted radius
        if (!found_ik)
        {
            ROS_WARN("Circle position IK solutions failed for cutter, trying with adjusted radius");

            // Try different radii
            std::vector<double> test_radii = {0.25, 0.15, 0.35, 0.2, 0.4};

            for (const auto &test_radius : test_radii)
            {
                // Try the optimal angle first with new radius
                tf::Vector3 test_point = peduncle_centroid + test_radius * (cos(theta_cutter) * u + sin(theta_cutter) * v);

                // Calculate orientation
                tf::Vector3 test_ee_z = (peduncle_centroid - test_point).normalized();
                tf::Vector3 test_ee_y = peduncle_axis.cross(test_ee_z).normalized();
                
                if (test_ee_y.length() < 0.1)
                {
                    tf::Vector3 world_up(0, 0, 1);
                    test_ee_y = (std::abs(test_ee_z.dot(world_up)) > 0.9) ? 
                               tf::Vector3(1, 0, 0).cross(test_ee_z).normalized() : 
                               world_up.cross(test_ee_z).normalized();
                }
                
                tf::Vector3 test_ee_x = test_ee_y.cross(test_ee_z).normalized();

                // Create rotation matrix and convert to quaternion
                tf::Quaternion test_quat = _get_norm_quat_from_axes(test_ee_x, test_ee_y, test_ee_z);
                geometry_msgs::Pose test_pose = _get_pose_from_pos_and_quat(test_point, test_quat);

                // Try IK for this pose
                found_ik = current_state->setFromIK(joint_model_group, test_pose, 10, 0.1);

                if (found_ik)
                {
                    ROS_INFO("Found cutter IK solution with adjusted radius: %.2f m", test_radius);
                    end_effector_pose = test_pose;
                    group_cutter.setPoseTarget(end_effector_pose);
                    cutter_point = test_point;
                    break;
                }
            }
        }
    }

    bool success;
    if(found_ik){
        std::vector<double> joint_values;
        current_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found valid IK solution for cutter pregrasp");
        
        // Plan to the pregrasp pose
        group_cutter.setJointValueTarget(joint_values);
        success = (group_cutter.plan(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            ROS_INFO("Successfully planned cutter pregrasp motion");
            
            // Execute the plan to move to pregrasp position
            bool exec_result = (group_cutter.execute(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (exec_result) {
                ROS_INFO("Successfully moved cutter to pregrasp position");
                
                // Now plan the approach to cutting position
                // Calculate the approach vector along the z-axis of the end effector
                tf::Vector3 approach(0.0, 0.0, req.reserve_dist);
                
                tf::Quaternion curr_quat;
                tf::quaternionMsgToTF(end_effector_pose.orientation, curr_quat);
                tf::Matrix3x3 curr_rot(curr_quat);
                
                // Transform the approach vector by the rotation matrix
                tf::Vector3 transformed_approach = curr_rot * approach;
                
                // Calculate the final cutting position
                geometry_msgs::Pose cutting_pose = end_effector_pose;
                cutting_pose.position.x += transformed_approach.x();
                cutting_pose.position.y += transformed_approach.y();
                cutting_pose.position.z += transformed_approach.z();
                
                // Check IK for the cutting pose
                found_ik = current_state->setFromIK(joint_model_group, cutting_pose, 10, 0.1);
                
                if (found_ik) {
                    group_cutter.setPoseTarget(cutting_pose);
                    
                    // Plan the approach to cutting position
                    success = (group_cutter.plan(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    
                    if (success) {
                        ROS_INFO("Successfully planned cutter approach motion");
                        show_trail(success);
                    } else {
                        ROS_ERROR("Failed to plan cutter approach motion");
                    }
                } else {
                    ROS_ERROR("No IK solution found for cutter final pose");
                    success = false;
                }
            } else {
                ROS_ERROR("Failed to execute cutter pregrasp motion");
                success = false;
            }
        } else {
            ROS_ERROR("Failed to plan cutter pregrasp motion");
        }
    } else {
        ROS_ERROR("Could not find IK solution for any tested cutter pose");
        success = false;
    }
    
    ROS_INFO("Finished planning cutter grasp and approach");
    return success;
}

bool VADERPlanner::planGripperGraspPose(vader_msgs::BimanualPlanRequest::Request &req){
    //If we rotated for the camera before, restore that
    moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
    const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    int num_joints = joint_values.size();
    joint_values[num_joints - 1] -= gripper_pregrasp_cam_rotated_amount;

    group_gripper.setJointValueTarget(joint_values);
    bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        // Execute the plan
        bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (exec_result) {
            ROS_INFO("Successfully moved to grasp position");
            geometry_msgs::Pose current_pose = pregraspFinalGripperPose;

            tf::Vector3 approach(0.0, 0.0, req.reserve_dist + 0.03);

            tf::Quaternion curr_quat;
            tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
            tf::Matrix3x3 curr_rot(curr_quat);
          
            tf::Vector3 transformed_approach = curr_rot * approach;
          
            current_pose.position.x += transformed_approach.x();
            current_pose.position.y += transformed_approach.y();
            current_pose.position.z += transformed_approach.z();

            bool found_ik = _test_IK_for_gripper_pose(current_pose);

            if (found_ik) {
                // moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
                // const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
                // current_state->copyJointGroupPositions(joint_model_group, joint_values);
                
                // ROS_INFO("Found IK solution for pre-grasp pose");
                
                // // Use joint values target instead of pose target
                // group_gripper.setJointValueTarget(joint_values);
                group_gripper.setPoseTarget(current_pose);

                
                // Plan using joint space goal
                success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                
                ROS_INFO_NAMED("move_group_planner", "This plan (joint-space goal) %s", success ? "SUCCEEDED" : "FAILED");
                
                show_trail(success);
                
              } else {
                ROS_ERROR("Did not find IK solution for pre-grasp pose");
                success = false;
              }

        } else {
          ROS_ERROR("Failed to execute grasp plan");
          success = false;
        }
    }
    return success;
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
//     moveit_msgs::CollisionObject peduncle_collision_obj;
    // peduncle_collision_obj.header.frame_id = group_cutter.getPlanningFrame();
    // peduncle_collision_obj.id = "peduncle";

    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.CYLINDER;
    // primitive.dimensions.resize(2);
    // primitive.dimensions[primitive.CYLINDER_HEIGHT] = peduncle.shape.dimensions[0];
    // primitive.dimensions[primitive.CYLINDER_RADIUS] = peduncle.shape.dimensions[1];
    
    // peduncle_collision_obj.primitives.push_back(primitive);
    // peduncle_collision_obj.primitive_poses.push_back(peduncle.pose);
    // peduncle_collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    
    // planning_scene_interface.applyCollisionObject(peduncle_collision_obj);
    // ROS_INFO("Added peduncle collision object with height %f and radius %f", 
    //          primitive.dimensions[primitive.CYLINDER_HEIGHT], 
    //          primitive.dimensions[primitive.CYLINDER_RADIUS]);

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
