#pragma once


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <vector>

std_msgs::ColorRGBA PORTAL_ORANGE_COLOR() {
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 93.0 / 255.0;
    color.a = 0.2;
    return color;
}

std_msgs::ColorRGBA PORTAL_BLUE_COLOR() {
    std_msgs::ColorRGBA color;
    color.g = 101.0 / 255.0;
    color.b = 1.0;
    color.a = 0.2;
    return color;
}
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

class XArmForwardKinematics {
public:
  using Matrix4d = Eigen::Matrix4d;
  static constexpr size_t N_JOINTS = 7;

  Matrix4d forward_kinematics(const std::array<double, N_JOINTS>& joint_positions) const;

private:
  struct DHParams {
    double d, a, alpha;
  };

static constexpr std::array<DHParams, N_JOINTS> dh = {{
    { 0.267, 0.0, -M_PI/2 },
    { 0.000, 0.0,  M_PI/2 },
    { 0.293, 0.0525, M_PI/2 },
    { 0.000, 0.0775, M_PI/2 },
    { 0.3425, 0.0, M_PI/2 },
    { 0.000, 0.076, -M_PI/2 },
    { 0.097, 0.0, 0.0 },
}};

static constexpr std::array<double, N_JOINTS> cos_alpha = []{
    std::array<double, N_JOINTS> c = {};
    for (size_t i = 0; i < N_JOINTS; ++i) c[i] = std::cos(dh[i].alpha);
    return c;
}();

static constexpr std::array<double, N_JOINTS> sin_alpha = []{
    std::array<double, N_JOINTS> s = {};
    for (size_t i = 0; i < N_JOINTS; ++i) s[i] = std::sin(dh[i].alpha);
    return s;
}();

  Matrix4d joint_transform(size_t i, double theta) const;
};