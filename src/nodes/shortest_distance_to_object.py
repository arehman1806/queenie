#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Pose, Point

rospy.init_node('get_shortest_distance')

# set the dimensions of the cuboid
width = 1.0
height = 4.074220
depth = 1

# define a callback function to handle the robot pose messages
def robot_pose_callback(msg):
    robot_pos = msg.position

    # calculate the shortest distance between any vertex or center of cuboid and center of robot
    dx = max(object_pos.x - width / 2, min(robot_pos.x, object_pos.x + width / 2)) - robot_pos.x
    dy = max(object_pos.y - height / 2, min(robot_pos.y, object_pos.y + height / 2)) - robot_pos.y
    dz = max(object_pos.z - depth / 2, min(robot_pos.z, object_pos.z + depth / 2)) - robot_pos.z

    if dx == object_pos.x - width / 2:
        dx = -dx - width / 2
    elif dx == object_pos.x + width / 2:
        dx = -dx + width / 2

    if dy == object_pos.y - height / 2:
        dy = -dy - height / 2
    elif dy == object_pos.y + height / 2:
        dy = -dy + height / 2

    if dz == object_pos.z - depth / 2:
        dz = -dz - depth / 2
    elif dz == object_pos.z + depth / 2:
        dz = -dz + depth / 2

    distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    rospy.loginfo("Shortest distance between cuboid and robot is {:.2f} meters".format(distance))

# define a callback function to handle the object pose messages
def object_pose_callback(msg):
    global object_pos
    object_pos = msg.position


# subscribe to the robot pose and object pose topics
rospy.Subscriber('robot_pose', Pose, robot_pose_callback)
rospy.Subscriber('object_pose', Pose, object_pose_callback)

rospy.spin()
