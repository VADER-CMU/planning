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
    void _create_pepper_and_peduncle();

private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group_gripper, group_cutter;
    moveit::planning_interface::MoveGroupInterface::Plan plan_gripper, plan_cutter;
    moveit_visual_tools::MoveItVisualTools *visual_tools;

    geometry_msgs::Pose pregraspFinalGripperPose;

    uint8_t plan_type = 0;
    double gripper_pregrasp_cam_orig_value = 0.0;

    std::string PLANNING_GROUP_GRIPPER;
    std::string PLANNING_GROUP_CUTTER;

    ros::ServiceServer planning_service, execution_service;
    ros::ServiceServer move_to_storage_service;

    ros::Publisher display_path;

    void init();
    void _add_ground_plane_collision();
    void _add_pepper_collision(vader_msgs::Pepper &pepper);
    void _add_peduncle_collision(vader_msgs::Pepper &pepper);
    void _clear_pepper_collision();

    bool _test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose);
    bool _test_PC_gripper_approach(tf::Vector3 &axis, tf::Vector3 &centroid, double angle, double radius);
    bool planGripperPregraspPose(vader_msgs::BimanualPlanRequest::Request &req);
    bool planGripperGraspPose(vader_msgs::BimanualPlanRequest::Request &req);
    bool planCutterGraspPose(vader_msgs::BimanualPlanRequest::Request &req);
    // void planAndExecuteGripper(const vader_msgs::Pepper &pepper, double reserve_dist);
    // void planAndExecuteCutter(const vader_msgs::Pepper &peduncle);

    bool planning_service_handler(vader_msgs::BimanualPlanRequest::Request &req, vader_msgs::BimanualPlanRequest::Response &res);
    bool execution_service_handler(vader_msgs::BimanualExecRequest::Request &req, vader_msgs::BimanualExecRequest::Response &res);
    bool move_to_storage_service_handler(vader_msgs::MoveToStorageRequest::Request &req, vader_msgs::MoveToStorageRequest::Response &res);
    void show_trail(bool plan_result);
};

void VADERPlanner::init()
{
    //TODO joint names?
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

void VADERPlanner::_add_ground_plane_collision()
{
    // Add the ground plane
    moveit_msgs::CollisionObject ground_plane;
    ground_plane.header.frame_id = group_gripper.getPlanningFrame();
    ground_plane.id = "ground_plane";

    // Define the ground plane as a flat box
    shape_msgs::SolidPrimitive ground_primitive;
    ground_primitive.type = ground_primitive.BOX;
    ground_primitive.dimensions.resize(3);
    ground_primitive.dimensions[0] = 2.0;  // Length in x-direction
    ground_primitive.dimensions[1] = 2.0;  // Width in y-direction
    ground_primitive.dimensions[2] = 0.01; // Height in z-direction (flat)

    geometry_msgs::Pose ground_pose;
    ground_pose.position.x = -1.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -0.005; // Slightly below the robot's base
    ground_pose.orientation.w = 1.0;

    ground_plane.primitives.push_back(ground_primitive);
    ground_plane.primitive_poses.push_back(ground_pose);
    ground_plane.operation = moveit_msgs::CollisionObject::ADD;

    // Apply the ground plane to the planning scene
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

void VADERPlanner::_add_peduncle_collision(vader_msgs::Pepper &pepper)
{
    moveit_msgs::CollisionObject cylinder_object;
    cylinder_object.header.frame_id = group_cutter.getPlanningFrame();
    cylinder_object.id = "peduncle";

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

void VADERPlanner::_clear_pepper_collision() {
    planning_scene_interface.removeCollisionObjects({"pepper"});
}

void VADERPlanner::start()
{
    ROS_INFO("Spinning");
    spinner.start();
    _create_pepper_and_peduncle();
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
        // Cutter logic here
        res.result = false;
        return false;
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

        

        bool exec_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (plan_type == req.GRIPPER_GRASP_EXEC) {
            _clear_pepper_collision();
        }
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
    
    // moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
    // const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
    // bool _found = state_copy.setFromIK(joint_model_group, bin_location_pose, 10, 0.1);
    // assert(_found);
    // std::vector<double> joint_values;
    // state_copy.copyJointGroupPositions(joint_model_group, joint_values);

    // group_gripper.setJointValueTarget(joint_values);

    group_gripper.setPoseTarget(bin_location_pose);
    bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    show_trail(success);
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

static tf::Quaternion _get_norm_quat_from_axes(tf::Vector3 &ax_x, tf::Vector3 &ax_y, tf::Vector3 &ax_z) {
    // Ensure the axes are normalized
    ax_x.normalize();
    ax_y.normalize();
    ax_z.normalize();

    // Check for invalid axes (e.g., zero-length vectors)
    if (ax_x.length() < 1e-6 || ax_y.length() < 1e-6 || ax_z.length() < 1e-6) {
        ROS_ERROR("Invalid axes provided for quaternion calculation. Returning identity quaternion.");
        return tf::Quaternion(0.0, 0.0, 0.0, 1.0); // Identity quaternion
    }

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
    moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
    const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);

    bool success = state_copy.setFromIK(joint_model_group, test_pose, 0.1);
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
    ROS_INFO("Quaternion calculation inputs:");
    ROS_INFO("X-axis: (%f, %f, %f)", ee_x.x(), ee_x.y(), ee_x.z());
    ROS_INFO("Y-axis: (%f, %f, %f)", ee_y.x(), ee_y.y(), ee_y.z());
    ROS_INFO("Z-axis: (%f, %f, %f)", ee_z.x(), ee_z.y(), ee_z.z());
    // group_gripper.setPoseTarget(end_effector_pose);

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
    if(found_ik){
        moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
        const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
        bool _found = state_copy.setFromIK(joint_model_group, end_effector_pose, 0.1);
        assert(_found);

        std::vector<double> joint_values;
        state_copy.copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found valid IK solution");

        //Add 90-deg cam rotation
        if(req.gripper_camera_rotation == req.GRIPPER_DO_ROTATE_CAMERA) {
            int num_joints = joint_values.size();
            gripper_pregrasp_cam_orig_value = joint_values[num_joints - 1];
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
        }

        pregraspFinalGripperPose = end_effector_pose;
    
        group_gripper.setJointValueTarget(joint_values);
        success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        show_trail(success);
    
    }else{
        ROS_ERROR("Could not find IK solution for any tested pose");
        success = false;
    }
    ROS_INFO("Finished planning");
    return success;
}

bool VADERPlanner::planGripperGraspPose(vader_msgs::BimanualPlanRequest::Request &req){

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

    // group_gripper.setPoseTarget(end_effector_pose);

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
                break;
            }
        }
    }
    bool success;
    if(found_ik){
        moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
        const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);
        bool _found = state_copy.setFromIK(joint_model_group, end_effector_pose, 0.1);
        assert(_found);

        std::vector<double> joint_values;
        state_copy.copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found valid IK solution");

        pregraspFinalGripperPose = end_effector_pose;
    
        group_gripper.setJointValueTarget(joint_values);
        success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        show_trail(success);

        // Execute the plan

        bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(exec_result){
            ROS_INFO("Successfully moved to grasp position");
            geometry_msgs::Pose current_pose = end_effector_pose;
        
            tf::Vector3 approach(0.0, 0.0, 0.1);

            tf::Quaternion curr_quat;
            tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
            tf::Matrix3x3 curr_rot(curr_quat);
          
            tf::Vector3 transformed_approach = curr_rot * approach;
          
            current_pose.position.x += transformed_approach.x();
            current_pose.position.y += transformed_approach.y();
            current_pose.position.z += transformed_approach.z();
            

            group_gripper.setPoseTarget(current_pose);

            success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            show_trail(success);
        }
    
    }else{
        ROS_ERROR("Could not find IK solution for any tested pose");
        success = false;
    }
    ROS_INFO("Finished planning");
    return success;
}

bool VADERPlanner::planCutterGraspPose(vader_msgs::BimanualPlanRequest::Request &req)
{
    // Display peduncle as a collision object
    _add_peduncle_collision(req.pepper);

    tf::Quaternion peduncle_quat;
    tf::quaternionMsgToTF(req.pepper.fruit_data.pose.orientation, peduncle_quat);

    tf::Vector3 peduncle_axis = tf::quatRotate(peduncle_quat, tf::Vector3(0, 0, 1)).normalized();

    tf::Vector3 peduncle_centroid(
        req.pepper.fruit_data.pose.position.x,
        req.pepper.fruit_data.pose.position.y,
        req.pepper.fruit_data.pose.position.z);

    double radius = req.reserve_dist;

    // Generate two orthonormal vectors u, v perpendicular to peduncle_axis
    tf::Vector3 u, v;
    tf::Vector3 ref(0, 0, 1);
    if (std::abs(peduncle_axis.dot(ref)) > 0.9)
        ref = tf::Vector3(1, 0, 0);

    u = peduncle_axis.cross(ref).normalized();
    v = peduncle_axis.cross(u).normalized();

    // Calculate the closest point on the parametric circle
    tf::Vector3 closest_point = peduncle_centroid + radius * u;

    // Calculate end effector orientation
    tf::Vector3 ee_z = (peduncle_centroid - closest_point).normalized();
    tf::Vector3 ee_y = peduncle_axis.cross(ee_z).normalized();
    tf::Vector3 ee_x = ee_y.cross(ee_z).normalized();

    tf::Quaternion ee_quat = _get_norm_quat_from_axes(ee_x, ee_y, ee_z);
    geometry_msgs::Pose end_effector_pose = _get_pose_from_pos_and_quat(closest_point, ee_quat);

    // Plan motion
    group_cutter.setPoseTarget(end_effector_pose);
    bool success = (group_cutter.plan(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    show_trail(success);

    if (!success)
        ROS_ERROR("Cutter grasp planning failed.");
    return success;
}

// void VADERPlanner::planAndExecuteGripper(const vader_msgs::Pepper &pepper, double reserve_dist)
// {
//     ROS_INFO("Planning and executing gripper motion to the pepper.");

//     // Plan motion for the gripper to the pepper
//     vader_msgs::BimanualPlanRequest::Request gripper_req;
//     gripper_req.pepper = pepper;
//     gripper_req.reserve_dist = reserve_dist;

//     bool success = planGripperPregraspPose(gripper_req);
//     if (success) {
//         ROS_INFO("Gripper successfully planned and executed motion to the pepper.");
//     } else {
//         ROS_ERROR("Gripper failed to plan or execute motion to the pepper.");
//     }
// }

// void VADERPlanner::planAndExecuteCutter(const vader_msgs::Pepper &peduncle)
// {
//     ROS_INFO("Planning and executing cutter motion to the peduncle.");

//     // Set planning parameters
//     group_cutter.setPlanningTime(10.0); // Allow more time for planning
//     group_cutter.setNumPlanningAttempts(5); // Increase the number of planning attempts

//     // Plan motion for the cutter to the peduncle
//     group_cutter.setPoseTarget(peduncle.fruit_data.pose);
//     bool success = (group_cutter.plan(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//     if (success) {
//         ROS_INFO("Cutter successfully planned motion to the peduncle. Executing...");
//         bool exec_result = (group_cutter.execute(plan_cutter) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         if (exec_result) {
//             ROS_INFO("Cutter successfully executed motion to the peduncle.");
//         } else {
//             ROS_ERROR("Cutter failed to execute motion to the peduncle.");
//         }
//     } else {
//         ROS_ERROR("Cutter failed to plan motion to the peduncle.");
//     }
// }
// bool VADERPlanner::planGripperGraspPose(vader_msgs::BimanualPlanRequest::Request &req){
//     //If we rotated for the camera before, restore that
//     moveit::core::RobotState _state = *group_gripper.getCurrentState();
//     const robot_state::JointModelGroup *joint_model_group = _state.getJointModelGroup(PLANNING_GROUP_GRIPPER);
    
//     std::vector<double> joint_values;
//     _state.copyJointGroupPositions(joint_model_group, joint_values);
//     joint_values[joint_values.size() - 1] = gripper_pregrasp_cam_orig_value;

//     group_gripper.setJointValueTarget(joint_values);
//     bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success) {
//         // Execute the plan
//         bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         if (exec_result) {
//             ROS_INFO("Successfully moved to grasp position");
//             geometry_msgs::Pose current_pose = pregraspFinalGripperPose;

//             tf::Vector3 approach(0.0, 0.0, req.reserve_dist + 0.06);

//             tf::Quaternion curr_quat;
//             tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
//             tf::Matrix3x3 curr_rot(curr_quat);
          
//             tf::Vector3 transformed_approach = curr_rot * approach;
          
//             current_pose.position.x += transformed_approach.x();
//             current_pose.position.y += transformed_approach.y();
//             current_pose.position.z += transformed_approach.z();

//             bool found_ik = _test_IK_for_gripper_pose(current_pose);

//             if (found_ik) {
//                 // moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
//                 // const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
//                 // current_state->copyJointGroupPositions(joint_model_group, joint_values);
                
//                 // ROS_INFO("Found IK solution for pre-grasp pose");
                
//                 // // Use joint values target instead of pose target
//                 // group_gripper.setJointValueTarget(joint_values);
//                 group_gripper.setPoseTarget(current_pose);
//                 // moveit::core::RobotState _state = *group_gripper.getCurrentState();
//                 // const robot_state::JointModelGroup *joint_model_group = _state.getJointModelGroup(PLANNING_GROUP_GRIPPER);
//                 // //Solve IK using this state
//                 // bool _found = _state.setFromIK(joint_model_group, current_pose, 10, 0.1);
//                 // assert(_found);
//                 // std::vector<double> joint_values;
//                 // _state.copyJointGroupPositions(joint_model_group, joint_values);
//                 // group_gripper.setJointValueTarget(joint_values);



                
//                 // Plan using joint space goal
//                 success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                
//                 ROS_INFO_NAMED("move_group_planner", "This plan (joint-space goal) %s", success ? "SUCCEEDED" : "FAILED");
                
//                 show_trail(success);
                
//               } else {
//                 ROS_ERROR("Did not find IK solution for pre-grasp pose");
//                 success = false;
//               }

//         } else {
//           ROS_ERROR("Failed to execute grasp plan");
//           success = false;
//         }
//     }
//     return success;
// }
void VADERPlanner::_create_pepper_and_peduncle()
{
    // Create the pepper object
    vader_msgs::Pepper pepper;
    pepper.fruit_data.pose.position.x = 0.5;
    pepper.fruit_data.pose.position.y = 0.0;
    pepper.fruit_data.pose.position.z = 0.3; // Above the table
    pepper.fruit_data.pose.orientation.w = 1.0;
    pepper.fruit_data.shape.dimensions = {0.2, 0.05}; // Height and radius

    _add_pepper_collision(pepper);

    // Create the peduncle object
    vader_msgs::Pepper peduncle;
    peduncle.fruit_data.pose.position.x = 0.5;
    peduncle.fruit_data.pose.position.y = 0.0;
    peduncle.fruit_data.pose.position.z = 0.6; // Above the pepper
    peduncle.fruit_data.pose.orientation.w = 1.0;
    peduncle.fruit_data.shape.dimensions = {0.1, 0.02}; // Height and radius

    _add_peduncle_collision(peduncle);

    // Reset gripper to a valid initial state
    group_gripper.setNamedTarget("home"); // Replace "home" with a valid named target for the gripper
    if (group_gripper.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move gripper to the initial state.");
        return;
    }

    // Reset cutter to a valid initial state
    // Reset cutter to a valid initial state
    group_cutter.setNamedTarget("home"); // Replace "home" with a valid named target for the cutter
    if (group_cutter.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move cutter to the initial state. Attempting to reset manually.");

        // Manually set a valid joint configuration if "home" fails
        std::vector<double> cutter_joint_values = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0, 0.0}; // Example joint values
        group_cutter.setJointValueTarget(cutter_joint_values);
        if (group_cutter.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to reset cutter to a valid state. Aborting.");
            return;
        }
    }

    // Plan and execute gripper motions
    vader_msgs::BimanualPlanRequest::Request gripper_req;
    gripper_req.pepper = pepper;
    gripper_req.reserve_dist = 0.1; // Reserve distance for pregrasp

    if (planGripperPregraspPose(gripper_req)) {
        ROS_INFO("Gripper pregrasp succeeded. Proceeding to grasp.");
        if (!planGripperGraspPose(gripper_req)) {
            ROS_ERROR("Gripper grasp failed.");
        }
    } else {
        ROS_ERROR("Gripper pregrasp failed.");
    }

    // Plan and execute cutter motion
    vader_msgs::BimanualPlanRequest::Request cutter_req;
    cutter_req.pepper = peduncle;
    cutter_req.reserve_dist = 0.0; // Directly to the peduncle center

    if (!planCutterGraspPose(cutter_req)) {
        ROS_ERROR("Cutter grasp failed.");
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vader_dual_planner");

    VADERPlanner planner;

    planner.start();

    ros::waitForShutdown();
    return 0;
}
