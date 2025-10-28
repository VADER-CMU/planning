#!/usr/bin/env python3
# This file publishes a static transform between the camera link (realsense in gripper hand) and the end effector link (XArm7 final link).
# This should be run at startup when an object in the camera frame needs to be transformed into global coordinates.
import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

ee_frame = "L_link_eef"
cam_frame = "camera_link"
calib = geometry_msgs.msg.TransformStamped()
calib.header.stamp = rospy.Time.now()
calib.header.frame_id = ee_frame
calib.child_frame_id = cam_frame
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
broadcaster.sendTransform(calib)
print("transform sent")
rospy.spin()