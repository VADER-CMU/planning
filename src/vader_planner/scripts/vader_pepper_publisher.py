#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle
import random

PEPPER_TOPIC = "fruit_pose"

def generate_random_pose():
    pose = Pose()
    pose.position.x = random.uniform(0, 0.5)
    pose.position.y = random.uniform(0, 0.5)
    pose.position.z = random.uniform(0.5, 0.8)
    #Assume identity rotation for now
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    return pose

def generate_solid_primitive():
    # Always use a cylinder
    solid_primitive = SolidPrimitive()
    solid_primitive.type = SolidPrimitive.CYLINDER
    solid_primitive.dimensions = [random.uniform(0, 1), random.uniform(0, 1)]
    return solid_primitive

def generate_fruit():
    fruit = Fruit()
    pose = Pose()
    pose.position.x = random.uniform(0.4, 0.5)
    pose.position.y = random.uniform(0.2, 0.3)
    pose.position.z = 0.3#random.uniform(0.5, 0.8)
    #Assume identity rotation for now
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    fruit.pose = pose
    fruit_primitive = SolidPrimitive()
    fruit_primitive.type = SolidPrimitive.CYLINDER
    fruit_primitive.dimensions = [0.1, 0.075]
    fruit.shape = fruit_primitive
    return fruit

def generate_peduncle(fruit):
    peduncle = Peduncle()

    peduncle_primitive = SolidPrimitive()
    peduncle_primitive.type = SolidPrimitive.CYLINDER
    peduncle_primitive.dimensions = [0.1, 0.01]
    pose = Pose()
    pose.position.x = fruit.pose.position.x
    pose.position.y = fruit.pose.position.y
    pose.position.z = fruit.pose.position.z + fruit.shape.dimensions[0] / 2 + peduncle_primitive.dimensions[0] / 2
    #Assume identity rotation for now
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    peduncle.pose = pose
    peduncle.shape = peduncle_primitive
    return peduncle

def generate_pepper():
    pepper = Pepper()

    pepper.header.stamp = rospy.Time.now()
    pepper.header.frame_id = "camera_depth_optical_frame"  # Use appropriate frame ID
    pepper.fruit_data = generate_fruit()
    pepper.peduncle_data = generate_peduncle(pepper.fruit_data)
    return pepper

def publisher():
    rospy.init_node('vader_pepper_publisher', anonymous=True)
    pub = rospy.Publisher(PEPPER_TOPIC, Pepper, queue_size=10)
    rate = rospy.Rate(0.1)  # once every 20 seconds

    rospy.sleep(1) # Wait for the publisher to be registered + everything launched

    while not rospy.is_shutdown():
        pepper = generate_pepper()
        rospy.loginfo(f"Publishing Pepper: {pepper}")
        pub.publish(pepper)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass