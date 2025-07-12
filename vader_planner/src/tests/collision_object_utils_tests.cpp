#include "utils/utils.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <gazebo_msgs/SpawnModel.h>

int main() {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "collision_object_utils_tests");
    ros::NodeHandle node_handle;    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    ros::Duration(3.0).sleep();
    // Create the Gazebo spawn client
    auto spawn_client = createGazeboSpawnURDFModelClient(node_handle);

    // Define the pose at the origin
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 2.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    std::vector<double> sizes = {0.1, 0.1, 1}; // Define the size of the box

    // Create a SolidPrimitive for the box
    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = sizes[0]; // x size
    box_primitive.dimensions[1] = sizes[1]; // y size
    box_primitive.dimensions[2] = sizes[2]; // z size

    addTwinBoxCollisionObject(
        spawn_client,
        planning_scene_interface,
        "twin_box_object",
        "world",
        box_primitive,
        pose
    );

    pose.position.x = -0.2;
    pose.position.z = 1.5;
    sizes = {0.2, 0.2, 0.2}; // Define the size of the second box
    box_primitive.dimensions[0] = sizes[0]; // x size
    box_primitive.dimensions[1] = sizes[1]; // y size
    box_primitive.dimensions[2] = sizes[2]; // z size

    addTwinBoxCollisionObject(
        spawn_client,
        planning_scene_interface,
        "twin_box_object_2",
        "world",
        box_primitive,
        pose
    );

    pose.position.x = 0.2;

    addTwinBoxCollisionObject(
        spawn_client,
        planning_scene_interface,
        "twin_box_object_3",
        "world",
        box_primitive,
        pose
    );

    return 0;
}