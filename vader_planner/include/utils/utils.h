#pragma once


#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <vector>

std_msgs::ColorRGBA COLOR_ORANGE_TRANSLUCENT() {
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 93.0 / 255.0;
    color.a = 0.2;
    return color;
}

std_msgs::ColorRGBA COLOR_BLUE_TRANSLUCENT() {
    std_msgs::ColorRGBA color;
    color.g = 101.0 / 255.0;
    color.b = 1.0;
    color.a = 0.2;
    return color;
}

inline geometry_msgs::Quaternion QUAT_UP() {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return q;
}

inline geometry_msgs::Quaternion QUAT_IDENTITY() {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return q;
}

inline geometry_msgs::Quaternion QUAT_DOWN() {
    geometry_msgs::Quaternion q;
    q.x = 1.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 0.0;
    return q;
}

inline geometry_msgs::Quaternion QUAT_TOWARD_PLANT() {
    geometry_msgs::Quaternion q;
    q.x = std::sqrt(2.0) / 2.0;
    q.y = 0.0;
    q.z = std::sqrt(2.0) / 2.0;
    q.w = 0.0;
    return q;
}

inline geometry_msgs::Pose makePose(double x, double y, double z, const geometry_msgs::Quaternion& quat) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = quat;
    return pose;
}

inline shape_msgs::SolidPrimitive makeBoxPrimitive(double x, double y, double z) {
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = x;
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y;
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z;
    return box;
}

inline void setACMEntries(
    planning_scene_monitor::PlanningSceneMonitorPtr& psm,
    ros::Publisher& planning_scene_diff_pub,
    const std::vector<std::pair<std::string, std::string>>& allowed_entries)
{
    if (!psm->getPlanningScene())
    {
        ROS_ERROR("PlanningSceneMonitor not properly initialized.");
        return;
    }
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);
    collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

    for (const auto& entry : allowed_entries) {
        acm.setEntry(entry.first, entry.second, true);
    }

    scene->getCurrentStateNonConst().update();
    psm->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    moveit_msgs::PlanningScene ps_msg;
    scene->getPlanningSceneMsg(ps_msg);
    ps_msg.is_diff = true;
    planning_scene_diff_pub.publish(ps_msg);
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