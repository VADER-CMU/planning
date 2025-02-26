#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

ee_frame = "link_eef"
cam_frame = "link_camera"
calib = geometry_msgs.msg.TransformStamped()
calib.header.stamp = rospy.Time.now()
calib.header.frame_id = ee_frame
calib.child_frame_id = cam_frame
calib.transform.translation.x = 0.0
calib.transform.translation.y = 0.0
calib.transform.translation.z = 0.08 #8cm from end effector plane to top of camera
#identity transform, Z axis is same as end effector frame.
calib.transform.rotation.x = 0.0
calib.transform.rotation.y = 0.0
calib.transform.rotation.z = 0.0
calib.transform.rotation.w = 1.0
broadcaster = tf2_ros.StaticTransformBroadcaster()
broadcaster.sendTransform(calib)
rospy.spin()