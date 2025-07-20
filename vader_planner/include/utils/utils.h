#pragma once


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
    const geometry_msgs::Pose &pose);

void add_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    moveit_msgs::CollisionObject &collision_object);

void remove_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::string &object_id);

void remove_moveit_collision_objects(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::vector<std::string> &object_ids);

void remove_moveit_collision_object(
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    moveit_msgs::CollisionObject &collision_object);

//--------------------------------------------Gazebo Collision Object Functions--------------------------------------------//

ros::ServiceClient createGazeboSpawnURDFModelClient(ros::NodeHandle &nh);
ros::ServiceClient createGazeboDeleteModelClient(ros::NodeHandle &nh);

std::string generateBoxURDF(const std::string &model_name, const std::vector<double> &sizes, const bool is_static = true);

gazebo_msgs::SpawnModel::Response spawnGazeboCollisionObject(
    ros::ServiceClient &spawn_client,
    const gazebo_msgs::SpawnModel::Request &spawn_model_req);

gazebo_msgs::DeleteModel::Response deleteGazeboCollisionObject(
    ros::ServiceClient &delete_client,
    const std::string &model_name);

//-------------------------------------------Twin Collision Object Functions--------------------------------------------//
void addTwinBoxCollisionObject(
    ros::ServiceClient &spawn_client,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const std::string &object_name,
    const std::string &reference_frame,
    const shape_msgs::SolidPrimitive &primitive,
    const geometry_msgs::Pose &pose);