#!/usr/bin/env python3
# This file publishes a static transform between the camera link (realsense in gripper hand) and the end effector link (XArm7 final link).
# This should be run at startup when an object in the camera frame needs to be transformed into global coordinates.
import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

calib = geometry_msgs.msg.TransformStamped()
calib.header.stamp = rospy.Time.now()
calib.header.frame_id = "L_link_eef"
calib.child_frame_id = "camera_link"
calib.transform.translation.x = 0.0
calib.transform.translation.y = 0.0
calib.transform.translation.z = 0.08 #8cm from end effector plane to top of camera

# Rotation: rotate 180 deg by X and 180 deg by Z
# In HRI file, x/y/z should be used as normal and not flipped like PR4. This should be correct for gripper link
# Verify this with https://www.andre-gaschler.com/rotationconverter/
calib.transform.rotation.x = .5
calib.transform.rotation.y = -.5
calib.transform.rotation.z = .5
calib.transform.rotation.w = .5
broadcaster = tf2_ros.StaticTransformBroadcaster()
# broadcaster.sendTransform(calib)
# print("transform sent")

# # Also register as gripper_cam_link

calib2 = geometry_msgs.msg.TransformStamped()
calib2.header.stamp = rospy.Time.now()
calib2.header.frame_id = "L_link_eef"
calib2.child_frame_id = "gripper_cam_link"
calib2.transform.translation.x = 0.0
calib2.transform.translation.y = 0.0
calib2.transform.translation.z = 0.08 #8cm from end effector plane to top of camera

# Rotation: rotate 180 deg by X and 180 deg by Z
# In HRI file, x/y/z should be used as normal and not flipped like PR4. This should be correct for gripper link
# Verify this with https://www.andre-gaschler.com/rotationconverter/
calib2.transform.rotation.x = .5
calib2.transform.rotation.y = -.5
calib2.transform.rotation.z = .5
calib2.transform.rotation.w = .5
# broadcaster = tf2_ros.StaticTransformBroadcaster()
# broadcaster.sendTransform(calib2)

# Cutter cam
calib3 = geometry_msgs.msg.TransformStamped()

calib3.header.stamp = rospy.Time.now()
calib3.header.frame_id = "R_link_eef"
calib3.child_frame_id = "cutter_cam_link"
calib3.transform.translation.x = 0.0
calib3.transform.translation.y = -0.060
calib3.transform.translation.z = 0.107

# calib3.transform.rotation.x = -0.5
# calib3.transform.rotation.y = -0.5
# calib3.transform.rotation.z = -0.5
# calib3.transform.rotation.w = 0.5

# After righting the camera
calib3.transform.rotation.x = 0.707
calib3.transform.rotation.y = 0
calib3.transform.rotation.z = 0.707
calib3.transform.rotation.w = 0



# broadcaster.sendTransform(calib3)

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
    # Update timestamps if needed for dynamic transforms
    calib3.header.stamp = rospy.Time.now()
    calib2.header.stamp = rospy.Time.now()
    calib.header.stamp = rospy.Time.now()

    broadcaster.sendTransform(calib3)
    broadcaster.sendTransform(calib2)
    broadcaster.sendTransform(calib)
    rate.sleep()