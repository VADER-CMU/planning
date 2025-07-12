
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <vector>

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


std::string generateUnitBoxURDF(const std::string &model_name)
{
    std::ostringstream urdf;
    urdf << "<robot name=\"" << model_name << "\">"
         << "<link name=\"box_link\">"
         << "<visual>"
         << "<geometry>"
         << "<box size=\"1 1 1\"/>"
         << "</geometry>"
         << "<material name=\"white\">"
         << "<color rgba=\"1 1 1 1\"/>"
         << "</material>"
         << "</visual>"
         << "<collision>"
         << "<geometry>"
         << "<box size=\"1 1 1\"/>"
         << "</geometry>"
         << "</collision>"
         << "<inertial>"
         << "<mass value=\"1\"/>"
         << "<origin xyz=\"0 0 0\"/>"
         << "<inertia ixx=\"0.166\" ixy=\"0\" ixz=\"0\" iyy=\"0.166\" iyz=\"0\" izz=\"0.166\"/>"
         << "</inertial>"
         << "</link>"
         << "</robot>";
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