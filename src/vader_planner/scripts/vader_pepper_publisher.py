import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle
import random

#!/usr/bin/env python


def generate_random_pose():
    pose = Pose()
    pose.position.x = random.uniform(-1, 1)
    pose.position.y = random.uniform(-1, 1)
    pose.position.z = random.uniform(-1, 1)
    pose.orientation.x = random.uniform(-1, 1)
    pose.orientation.y = random.uniform(-1, 1)
    pose.orientation.z = random.uniform(-1, 1)
    pose.orientation.w = random.uniform(-1, 1)
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
    rate = rospy.Rate(0.1)  # 0.1Hz

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