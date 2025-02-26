#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle
import random

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
    fruit.pose = generate_random_pose()
    fruit.shape = generate_solid_primitive()
    return fruit

def generate_peduncle():
    peduncle = Peduncle()
    peduncle.pose = generate_random_pose()
    peduncle.shape = generate_solid_primitive()
    return peduncle

def generate_pepper():
    pepper = Pepper()
    pepper.fruit_data = generate_fruit()
    pepper.peduncle_data = generate_peduncle()
    return pepper

def publisher():
    rospy.init_node('vader_pepper_publisher', anonymous=True)
    pub = rospy.Publisher('random_pepper', Pepper, queue_size=10)
    rate = rospy.Rate(0.05)  # once every 20 seconds

    rospy.sleep(20) # Wait for the publisher to be registered + everything launched

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