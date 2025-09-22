
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <vector>
#include "utils/utils.h"

//--------------------------------------------Moveit Collision Object Functions--------------------------------------------//


moveit_msgs::CollisionObject add_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::string &object_id,
    const std::string &frame_id,
    const shape_msgs::SolidPrimitive &primitive,
    const geometry_msgs::Pose &pose)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = object_id;
    collision_object.header.frame_id = frame_id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
    return collision_object;
}

void add_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    moveit_msgs::CollisionObject &collision_object,
    const std_msgs::ColorRGBA &object_color)
{
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyCollisionObject(collision_object, object_color);
}

void add_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    moveit_msgs::CollisionObject &collision_object)
{
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void remove_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::string &object_id)
{
    planning_scene_interface.removeCollisionObjects({object_id});
}

void remove_moveit_collision_objects(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::vector<std::string> &object_ids)
{
    planning_scene_interface.removeCollisionObjects(object_ids);
}

void remove_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    moveit_msgs::CollisionObject &collision_object)
{
    collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface.applyCollisionObject(collision_object);
}

//--------------------------------------------Gazebo Collision Object Functions--------------------------------------------//


std::string generateBoxURDF(const std::string &model_name, const std::vector<double> &sizes, const bool is_static)
{
    assert (sizes.size() == 3 && "Sizes vector must contain elements for x, y, and z dimensions.");
    std::ostringstream urdf;
    double x = sizes[0];
    double y = sizes[1];
    double z = sizes[2];

    urdf << "<robot name=\"" << model_name << "\">"
         << "<link name=\"box_link\">"
         << "<visual>"
         << "<geometry>"
         << "<box size=\"" << x << " " << y << " " << z << "\"/>"
         << "</geometry>"
         << "<material name=\"white\">"
         << "<color rgba=\"1 1 1 1\"/>"
         << "</material>"
         << "</visual>"
         << "<collision>"
         << "<geometry>"
         << "<box size=\"" << x << " " << y << " " << z << "\"/>"
         << "</geometry>"
         << "</collision>"
         << "<inertial>"
         << "<mass value=\"1\"/>"
         << "<origin xyz=\"0 0 0\"/>"
         << "<inertia ixx=\"0.166\" ixy=\"0\" ixz=\"0\" iyy=\"0.166\" iyz=\"0\" izz=\"0.166\"/>"
         << "</inertial>"
         << "</link>";
    if (is_static)
    {
        urdf << "<gazebo>"
                << "<static>true</static>"
                << "</gazebo>";
    }
    urdf << "</robot>";
    return urdf.str();
}

ros::ServiceClient createGazeboSpawnURDFModelClient(ros::NodeHandle &nh)
{
    return nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
}

ros::ServiceClient createGazeboDeleteModelClient(ros::NodeHandle &nh)
{
    return nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
}


gazebo_msgs::SpawnModel::Response spawnGazeboCollisionObject(
    ros::ServiceClient &spawn_client,
    const gazebo_msgs::SpawnModel::Request &spawn_model_req)
{
    gazebo_msgs::SpawnModel::Response spawn_model_resp;
    if (spawn_client.call(spawn_model_req, spawn_model_resp))
    {
        if (spawn_model_resp.success)
        {
            ROS_INFO_STREAM("Model has been spawned: " << spawn_model_resp.status_message);
        }
        else
        {
            ROS_ERROR_STREAM("Model spawn failed: " << spawn_model_resp.status_message);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/spawn_urdf_model");
    }
    return spawn_model_resp;
}

gazebo_msgs::DeleteModel::Response deleteGazeboCollisionObject(
    ros::ServiceClient &delete_client,
    const std::string &model_name)
{
    gazebo_msgs::DeleteModel::Request delete_model_req;
    delete_model_req.model_name = model_name;

    gazebo_msgs::DeleteModel::Response delete_model_resp;
    if (delete_client.call(delete_model_req, delete_model_resp))
    {
        if (delete_model_resp.success)
        {
            ROS_INFO_STREAM("Model has been deleted: " << delete_model_resp.status_message);
        }
        else
        {
            ROS_ERROR_STREAM("Model deletion failed: " << delete_model_resp.status_message);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/delete_model");
    }
    return delete_model_resp;
}

//-------------------------------------------Twin Collision Object Functions--------------------------------------------//


// Calls both Gazebo and Moveit add collision objects, using the size from the primitive.
void addTwinBoxCollisionObject(
    ros::ServiceClient &spawn_client,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::string &object_name,
    const std::string &reference_frame,
    const shape_msgs::SolidPrimitive &primitive,
    const geometry_msgs::Pose &pose)
{
    add_moveit_collision_object(planning_scene_interface, object_name, reference_frame, primitive, pose);

    gazebo_msgs::SpawnModel::Request spawn_request;
    spawn_request.model_name = object_name;
    spawn_request.model_xml = generateBoxURDF(object_name, primitive.dimensions, true);
    spawn_request.robot_namespace = "";
    spawn_request.initial_pose = pose;
    spawn_request.reference_frame = reference_frame;

    gazebo_msgs::SpawnModel::Response spawn_response = spawnGazeboCollisionObject(spawn_client, spawn_request);
}


/**
 * @brief Computes the forward kinematics for the XArm robot.
 * 
 * Calculates the end-effector transformation matrix given the joint positions.
 * 
 * @param joint_positions Array of joint angles (in radians) for each of the 7 joints.
 * @return Matrix4d Homogeneous transformation matrix representing the end-effector pose.
 */
XArmForwardKinematics::Matrix4d XArmForwardKinematics::forward_kinematics(const std::array<double, N_JOINTS>& joint_positions) const {
    XArmForwardKinematics::Matrix4d T = Matrix4d::Identity();
    for (size_t i = 0; i < N_JOINTS; ++i) {
        T = T * joint_transform(i, joint_positions[i]);
    }
    return T;
}

XArmForwardKinematics::Matrix4d XArmForwardKinematics::joint_transform(size_t i, double theta) const {
    double ct = std::cos(theta);
    double st = std::sin(theta);

    XArmForwardKinematics::Matrix4d T;
    T << ct, -st*cos_alpha[i],  st*sin_alpha[i], dh[i].a*ct,
            st,  ct*cos_alpha[i], -ct*sin_alpha[i], dh[i].a*st,
            0,       sin_alpha[i],      cos_alpha[i],    dh[i].d,
            0,           0,                 0,           1;
    return T;
}
