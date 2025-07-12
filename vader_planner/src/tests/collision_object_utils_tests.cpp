#include "utils/utils.h"
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>

int main() {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "collision_object_utils_tests");
    ros::NodeHandle node_handle;

    ros::Duration(7.0).sleep();
    // Create the Gazebo spawn client
    auto spawn_client = createGazeboSpawnURDFModelClient(node_handle);

    // Define the pose at the origin
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    // Generate the URDF string for a unit box
    std::string unit_box_urdf = generateUnitBoxURDF("unit_box");

    // Create the SpawnModel request
    gazebo_msgs::SpawnModel::Request spawn_request;
    spawn_request.model_name = "unit_box";
    spawn_request.model_xml = unit_box_urdf;
    spawn_request.robot_namespace = "";
    spawn_request.initial_pose = pose;
    spawn_request.reference_frame = "world";

    // Spawn a unit box collision object at the origin
    gazebo_msgs::SpawnModel::Response response = spawnGazeboCollisionObject(
        spawn_client,
        spawn_request
    );

    std::cout << "Spawned unit box collision object at the origin: "
              << (response.success ? "Success" : "Failure") << std::endl;

    return 0;
}