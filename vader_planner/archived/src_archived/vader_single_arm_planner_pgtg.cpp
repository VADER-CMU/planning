#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <vader_planner/pose_plan.h>
#include <vader_planner/joint_plan.h>
#include <vader_planner/exec_plan.h>
#include <vader_planner/single_straight_plan.h>

#include <vader_msgs/SingleArmPlanRequest.h>
#include <vader_msgs/SingleArmExecutionRequest.h>

#include <tf/transform_broadcaster.h>//include for quaternion rotation 
#include <tf/transform_listener.h>//include for quaternion rotation 
#include <tf/transform_datatypes.h>//include for quaternion rotation 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometric_shapes/shapes.h> // Include for cylinder shape
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#define SPINNER_THREAD_NUM 2

/* Used for Cartesian path computation, please modify as needed: */
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;


namespace rvt = rviz_visual_tools;

class VADERPlanner
{
  public:
    VADERPlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group_gripper(plan_group_name){init();};
    VADERPlanner():spinner(SPINNER_THREAD_NUM),group_gripper(PLANNING_GROUP_GRIPPER){init();};
    ~VADERPlanner(){ delete visual_tools;};
    void start();
    void stop();
    void _add_ground_plane_to_scene();

    static std::string PLANNING_GROUP_GRIPPER; // declaration of static class member

  private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_names;
    moveit::planning_interface::MoveGroupInterface group_gripper;
    moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    tf::TransformListener tf_listener;
    double gripper_pregrasp_cam_orig_value;

    geometry_msgs::Pose pregraspFinalGripperPose;

    ros::Publisher display_path;
    ros::ServiceServer plan_pose_srv;

    ros::ServiceServer plan_pose_exec_srv;

    ros::ServiceServer plan_final_pose_srv;

    ros::ServiceServer plan_final_pose_exec_srv;
    
    ros::ServiceServer plan_joint_srv;
    ros::ServiceServer plan_grasp_srv; // New service for grasp pose planning
    ros::ServiceServer plan_grasp_exec_srv; // New service for grasp pose planning

    ros::ServiceServer sing_cart_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    bool _test_IK_for_gripper_pose(geometry_msgs::Pose &test_pose);
    bool do_pregrasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool do_pregrasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);
    bool do_final_grasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res);
    bool do_final_grasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res);

    bool do_grasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res);
    bool do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res);
    bool do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res);
    bool exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
    void _add_collision_wall(vader_msgs::SingleArmPlanRequest::Request &req);
    void show_trail(bool plan_result);
    tf::Vector3 calculatePerpendicularVector(const tf::Vector3& cylinder_axis);
    
    // Store cylinder data for access between pregrasp and grasp operations
    geometry_msgs::Pose stored_cylinder_pose;
    tf::Vector3 stored_cylinder_axis;
};

void VADERPlanner::init()
{
  joint_names = group_gripper.getVariableNames(); // Changed from deprecated getJointNames()

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group_gripper.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group_gripper.getEndEffectorLink().c_str());

  _add_ground_plane_to_scene();

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("vader_plan", &VADERPlanner::do_pregrasp_pose_plan, this);
  plan_pose_exec_srv = node_handle.advertiseService("singleArmExec", &VADERPlanner::do_pregrasp_exec_plan, this);
  plan_final_pose_srv = node_handle.advertiseService("singleArmFinalPlan", &VADERPlanner::do_final_grasp_pose_plan, this);
  plan_final_pose_exec_srv = node_handle.advertiseService("singleArmFinalExec", &VADERPlanner::do_final_grasp_exec_plan, this);
  plan_grasp_srv = node_handle.advertiseService("xarm_grasp_plan", &VADERPlanner::do_grasp_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("xarm_joint_plan", &VADERPlanner::do_joint_plan, this);
  sing_cart_srv = node_handle.advertiseService("xarm_straight_plan", &VADERPlanner::do_single_cartesian_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &VADERPlanner::execute_plan_topic, this);
  exec_plan_srv = node_handle.advertiseService("xarm_exec_plan", &VADERPlanner::exec_plan_cb, this);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.8;
  visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  

}



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
  moveit::core::RobotState state_copy = *group_gripper.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = state_copy.getJointModelGroup(PLANNING_GROUP_GRIPPER);

  bool success = state_copy.setFromIK(joint_model_group, test_pose, 10, 0.1);
  return success;
}


void VADERPlanner::_add_ground_plane_to_scene(){
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

void VADERPlanner::_add_collision_wall(vader_msgs::SingleArmPlanRequest::Request &req){
    moveit_msgs::CollisionObject wall_object;
    wall_object.header.frame_id = group_gripper.getPlanningFrame();
    wall_object.id = "wall";
    

    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);
    wall_primitive.dimensions[0] = 0.01;  // Length in x-direction
    wall_primitive.dimensions[1] = 1.0;  // Width in y-direction
    wall_primitive.dimensions[2] = 1.0; // Height in z-direction

    geometry_msgs::Pose wall_pose;
    wall_pose.position.x = req.pepper.fruit_data.pose.position.x + 0.07;
    wall_pose.position.y = req.pepper.fruit_data.pose.position.y;
    wall_pose.position.z = req.pepper.fruit_data.pose.position.z;
    wall_pose.orientation.w = 1.0; // No rotation

    wall_object.primitives.push_back(wall_primitive);
    wall_object.primitive_poses.push_back(wall_pose);
    wall_object.operation = moveit_msgs::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(wall_object);

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
  if(plan_result)
  {
    ROS_INFO_NAMED("vader_planner", "Visualizing plan as trajectory line");
   
    visual_tools->deleteAllMarkers();
    const robot_state::JointModelGroup* joint_model_group = group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    visual_tools->publishTrajectoryLine(plan_gripper.trajectory_, joint_model_group);
    visual_tools->trigger();
  }
}

tf::Vector3 VADERPlanner::calculatePerpendicularVector(const tf::Vector3& cylinder_axis) {
  
  tf::Vector3 ref_vector(0, 0, 1);
  if (fabs(cylinder_axis.dot(ref_vector)) > 0.9) {
    ref_vector = tf::Vector3(1, 0, 0);
  }
  
  tf::Vector3 perpendicular = cylinder_axis.cross(ref_vector);
  perpendicular.normalize();
  return perpendicular;
}

bool VADERPlanner::do_pregrasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{ 
  // Store original request data for reference and later use in grasp planning
  geometry_msgs::Pose original_target;
  original_target.orientation.x = req.pepper.fruit_data.pose.orientation.x;    
  original_target.orientation.y = req.pepper.fruit_data.pose.orientation.y;
  original_target.orientation.z = req.pepper.fruit_data.pose.orientation.z;
  original_target.orientation.w = req.pepper.fruit_data.pose.orientation.w;
  original_target.position.x = req.pepper.fruit_data.pose.position.x;
  original_target.position.y = req.pepper.fruit_data.pose.position.y;
  original_target.position.z = req.pepper.fruit_data.pose.position.z; 
  stored_cylinder_pose = original_target;

  // Add collision wall to the scene
  _add_collision_wall(req);
  
  // Extract quaternion from the target
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
        bool _found = state_copy.setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
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

bool VADERPlanner::do_pregrasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res){
  // Execute the plan
  bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (exec_result) {
    ROS_INFO("Successfully moved to pre-grasp position");
  } else {
    ROS_ERROR("Failed to execute pre-grasp plan");
  }
  res.result = exec_result;
  return exec_result;
}


bool VADERPlanner::do_final_grasp_pose_plan(vader_msgs::SingleArmPlanRequest::Request &req, vader_msgs::SingleArmPlanRequest::Response &res)
{ 
  //request parameters are ignored :sunglasses:
  geometry_msgs::Pose current_pose = pregraspFinalGripperPose;//group_gripper.getCurrentPose("vader_gripper_base_link").pose;
  ROS_INFO("Planning to final grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    current_pose.position.x, current_pose.position.y, current_pose.position.z, 
    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  
  tf::Vector3 approach(0.0, 0.0, 0.08); // 0.15);

  tf::Quaternion curr_quat;
  tf::quaternionMsgToTF(current_pose.orientation, curr_quat);
  tf::Matrix3x3 curr_rot(curr_quat);

  tf::Vector3 transformed_approach = curr_rot * approach;

  current_pose.position.x += transformed_approach.x();
  current_pose.position.y += transformed_approach.y();
  current_pose.position.z += transformed_approach.z();
  ROS_INFO("Planning to final grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    current_pose.position.x, current_pose.position.y, current_pose.position.z, 
    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  // Use IK to find joint values for the target pose
  moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
  
  bool found_ik = current_state->setFromIK(joint_model_group, current_pose, 10, 0.1);
  bool success = false;
  
  if (found_ik) {
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    ROS_INFO("Found IK solution for pre-grasp pose");
    
    // Use joint values target instead of pose target
    group_gripper.setJointValueTarget(joint_values);
    
    // Plan using joint space goal
    success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res.result = success;
    
    ROS_INFO_NAMED("move_group_planner", "This plan (joint-space goal) %s", success ? "SUCCEEDED" : "FAILED");
    
    show_trail(success);
    
  } else {
    ROS_ERROR("Did not find IK solution for pre-grasp pose");
    success = false;
  }
  
  res.result = success;
  return success;
}


bool VADERPlanner::do_final_grasp_exec_plan(vader_msgs::SingleArmExecutionRequest::Request &req, vader_msgs::SingleArmExecutionRequest::Response &res){
  // Execute the plan
  bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (exec_result) {
    ROS_INFO("Successfully moved to final grasp position");
  } else {
    ROS_ERROR("Failed to execute final grasp plan");
  }
  res.result = exec_result;
  return exec_result;
}

bool VADERPlanner::do_grasp_pose_plan(vader_planner::pose_plan::Request &req, vader_planner::pose_plan::Response &res)
{
  // Use the stored cylinder pose and axis from pregrasp planning
  geometry_msgs::Pose cylinder_pose = stored_cylinder_pose;
  tf::Vector3 cylinder_axis = stored_cylinder_axis;
  
  // Get current robot state and transform for vader_gripper_base_link
  tf::StampedTransform gripper_transform;
  try {
    tf_listener.waitForTransform(group_gripper.getPlanningFrame(), "vader_gripper_base_link", 
                               ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform(group_gripper.getPlanningFrame(), "vader_gripper_base_link", 
                              ros::Time(0), gripper_transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("TF Exception: %s", ex.what());
    res.success = false;
    return false;
  }
  
  // Calculate the perpendicular vector to cylinder axis
  tf::Vector3 perpendicular_vector = calculatePerpendicularVector(cylinder_axis);
  
  // Calculate quaternion that aligns gripper's x-axis with perpendicular vector
  // and the approach direction (z-axis) with the vector from cylinder to gripper
  tf::Vector3 cylinder_position(cylinder_pose.position.x, 
                              cylinder_pose.position.y, 
                              cylinder_pose.position.z);
  tf::Vector3 gripper_position = gripper_transform.getOrigin();
  
  // Calculate vector from cylinder to gripper (this will be our approach vector)
  tf::Vector3 approach_vector = gripper_position - cylinder_position;
  approach_vector.normalize();
  
  // The x-axis of the gripper frame should be aligned with perpendicular_vector
  tf::Vector3 x_axis = perpendicular_vector;
  
  // The z-axis of the gripper frame should be aligned with approach_vector
  tf::Vector3 z_axis = approach_vector;
  
  // Calculate y-axis to complete the right-handed coordinate system
  tf::Vector3 y_axis = z_axis.cross(x_axis);
  y_axis.normalize();
  
  // Re-normalize x-axis to ensure orthogonality
  x_axis = y_axis.cross(z_axis);
  x_axis.normalize();
  
  // Create rotation matrix from three axis vectors
  tf::Matrix3x3 rotation_matrix(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z()
  );
  
  // Convert rotation matrix to quaternion
  tf::Quaternion grasp_quat;
  rotation_matrix.getRotation(grasp_quat);
  grasp_quat.normalize();
  
  // Calculate position offset from cylinder for grasp (slightly away from surface)
  double cylinder_radius = 0.075; // Same as in collision object
  double offset_distance = cylinder_radius + 0.02; // Offset for grasping
  
  // Calculate grasp position by moving offset_distance along approach_vector from cylinder surface
  tf::Vector3 offset_position = cylinder_position + (approach_vector * offset_distance);
  
  // Create the grasp pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.orientation.x = grasp_quat.x();
  grasp_pose.orientation.y = grasp_quat.y();
  grasp_pose.orientation.z = grasp_quat.z();
  grasp_pose.orientation.w = grasp_quat.w();
  grasp_pose.position.x = offset_position.x();
  grasp_pose.position.y = offset_position.y();
  grasp_pose.position.z = offset_position.z();
  
  ROS_INFO("Planning to grasp pose: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", 
    grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, 
    grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
  
  // Calculate Cartesian path for approach
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(grasp_pose);
  
  // Approach along the vector joining cylinder centroid and gripper frame
  geometry_msgs::Pose final_grasp_pose = grasp_pose;
  final_grasp_pose.position.x = cylinder_position.x() + (approach_vector.x() * 0.01); // Closer to cylinder
  final_grasp_pose.position.y = cylinder_position.y() + (approach_vector.y() * 0.01);
  final_grasp_pose.position.z = cylinder_position.z() + (approach_vector.z() * 0.01);
  waypoints.push_back(final_grasp_pose);
  
  // Compute Cartesian path
  group_gripper.setMaxVelocityScalingFactor(maxV_scale_factor * 0.5); // Slower for grasp
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group_gripper.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  bool success = true;
  if (fraction < 0.9) {
    // If Cartesian planning fails, try joint-space planning to the first waypoint
    ROS_WARN("Cartesian planning failed with coverage: %lf, trying joint-space planning", fraction);
    
    moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    
    bool found_ik = current_state->setFromIK(joint_model_group, grasp_pose, 10, 0.1);
    
    if (found_ik) {
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(joint_model_group, joint_values);
      
      group_gripper.setJointValueTarget(joint_values);
      success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
      ROS_ERROR("Could not find IK solution for grasp pose");
      success = false;
    }
  } else {
    // Use the Cartesian trajectory directly
    plan_gripper.trajectory_ = trajectory;
    success = true;
  }
  
  ROS_INFO_NAMED("move_group_planner", "This grasp plan %s", success ? "SUCCEEDED" : "FAILED");
  show_trail(success);
  
  if (success) {
    // Execute the plan
    bool exec_result = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (exec_result) {
      ROS_INFO("Successfully moved to grasp position");
    } else {
      ROS_ERROR("Failed to execute grasp plan");
      success = false;
    }
  }
  
  res.success = success;
  return success;
}

bool VADERPlanner::do_single_cartesian_plan(vader_planner::single_straight_plan::Request &req, vader_planner::single_straight_plan::Response &res)
{
  // For Cartesian path, we'll compute the path but then convert to joint space
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(req.target);
  group_gripper.setMaxVelocityScalingFactor(maxV_scale_factor);
  moveit_msgs::RobotTrajectory trajectory;
 
  double fraction = group_gripper.computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
  bool success = true;
  
  if(fraction < 0.9) {
    success = false;
  } else {
    // Get the first waypoint and compute IK
    moveit::core::RobotStatePtr current_state = group_gripper.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    
    bool found_ik = current_state->setFromIK(joint_model_group, req.target, 10, 0.1);
    
    if (found_ik) {
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(joint_model_group, joint_values);
      
      // Use joint-space planning instead
      group_gripper.setJointValueTarget(joint_values);
      success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
      ROS_ERROR("Could not find IK solution for Cartesian target");
      success = false;
    }
  }
  
  fprintf(stderr, "[VADERPlanner::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

  res.success = success;
  show_trail(success);
 
  return success;
}

bool VADERPlanner::do_joint_plan(vader_planner::joint_plan::Request &req, vader_planner::joint_plan::Response &res)
{
  ROS_INFO("move_group_planner received new plan Request");
  if(!group_gripper.setJointValueTarget(req.target))
  {
    ROS_ERROR("setJointValueTarget() Failed! Please check the dimension and range of given joint target.");
    return false;
  }
 
  bool success = (group_gripper.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
  show_trail(success);
  return success;
}

bool VADERPlanner::exec_plan_cb(vader_planner::exec_plan::Request &req, vader_planner::exec_plan::Response &res)
{
  if(req.exec)
  {
    ROS_INFO("Received Execution Service Request");
    bool finish_ok = (group_gripper.execute(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
    res.success = finish_ok;
    return finish_ok;
  }

  res.success = false;
  return false;
}

/* execution subscriber call-back function */
void VADERPlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
  { 
    ROS_INFO("Received Execution Command !!!!!");
    group_gripper.asyncExecute(plan_gripper); /* return without waiting for finish */
  }
}

std::string VADERPlanner::PLANNING_GROUP_GRIPPER; // Definition of static class member

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vader_single_arm_planner");
  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_name", robot_name);
  VADERPlanner::PLANNING_GROUP_GRIPPER = robot_name;

  VADERPlanner planner(VADERPlanner::PLANNING_GROUP_GRIPPER);

  planner.start();

  ROS_INFO("Waiting for \'pose_plan\' or \'joint_plan\' service Request ...");

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
}