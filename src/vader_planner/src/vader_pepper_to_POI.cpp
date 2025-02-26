#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/String.h>


// Assuming Pepper is a custom message type
#include <vader_msgs/Pepper.h>
#include <vader_msgs/Peduncle.h>
#include <vader_msgs/Fruit.h>

#include <vader_planner/pose_plan.h>
#include <vader_planner/exec_plan.h>
#include <iostream>

class VaderPlanner
{
public:
    VaderPlanner(ros::NodeHandle& nh) : nh_(nh)
    {
        std::cout << "VaderPlanner waiting for messages" << std::endl;
        // Initialize subscriber and publisher
        pepper_sub_ = nh_.subscribe("random_pepper", 10, &VaderPlanner::pepperCallback, this);
    }

    void pepperCallback(const vader_msgs::Pepper::ConstPtr& msg)
    {
        ROS_INFO("Received Pepper message");
        // Process the Pepper message and create a Pose message
        geometry_msgs::Pose pose_msg = calculatePOI(msg);

        // Create a service client
        ros::ServiceClient planClient = nh_.serviceClient<vader_planner::pose_plan>("xarm_pose_plan");
        ros::ServiceClient execClient = nh_.serviceClient<vader_planner::exec_plan>("xarm_exec_plan");

        // Create a service request
        vader_planner::pose_plan srv;
        srv.request.target = pose_msg;

        ROS_INFO("Calling service xarm_plan_pose_service");

        // Call the service
        if (planClient.call(srv))
        {
            ROS_INFO("Planner call successful");
            // Call the execution service
            vader_planner::exec_plan exec_srv;
            exec_srv.request.exec = true;
            ROS_INFO("Calling service xarm_exec_plan");
            if (execClient.call(exec_srv))
            {
                ROS_INFO("Execution call successful");
            }
            else
            {
                ROS_ERROR("Failed to call service xarm_exec_plan");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service xarm_plan_pose_service");
        }
    }

    geometry_msgs::Pose calculatePOI(const vader_msgs::Pepper::ConstPtr& pepper) {
        // Calculate the Point of Interest (POI) based on the Pepper message
        geometry_msgs::Pose poi;
        // Fill in the POI with appropriate data
        // This is just an example, modify it according to your requirements
        poi.position.x = pepper->fruit_data.pose.position.x;
        poi.position.y = pepper->fruit_data.pose.position.y;
        poi.position.z = pepper->fruit_data.pose.position.z;
        poi.orientation.x = pepper->fruit_data.pose.orientation.x;
        poi.orientation.y = pepper->fruit_data.pose.orientation.y;
        poi.orientation.z = pepper->fruit_data.pose.orientation.z;
        poi.orientation.w = pepper->fruit_data.pose.orientation.w;

        return poi;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pepper_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vader_planner_poi_calc");
    ros::NodeHandle nh;
    VaderPlanner vp(nh);
    // std::cout << "VaderPlanner object created" << std::endl;
    ros::spin();
    return 0;
}