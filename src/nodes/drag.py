#!/usr/bin/env python3
import rospy
from rospy import Subscriber
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from actionlib import SimpleActionServer
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time

# CONSTANTS
PICK = 0
NO_ACT = 1
DROP = 2
GRAB = 3
PUSH = 4


object_to_pick = np.array([0.128064, 0.115884, 0.258202])
palm_to_chassis_transform = np.array([0.485, 0, 0])
finger_length = 0.15

def listener(action_type, target_location, target_orientation):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('queenie_tf_listener', anonymous=True)
    queenie_driver = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    gripper_controller = rospy.Publisher('/queenie/gripper_controller/command', Float64MultiArray, queue_size=1)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    # estimate pick target
    place_target = np.array([-1.,-1.,0])
    pick_target = np.array([0., 0., 0.])
    while True or not rospy.is_shutdown():
        try:
            object_pose = tfBuffer.lookup_transform("world", "unit_box", rospy.Time())
            # palm_to_chassis = tfBuffer.lookup_transform("chassis", "palm", rospy.Time())
        except:
            continue
        object_pose_trans = object_pose.transform.translation
        print(object_pose_trans)
        pick_target[0] = object_pose_trans.x - object_to_pick[0]
        pick_target[1] = object_pose_trans.y
        pick_target[2] = object_pose_trans.z
        print(pick_target)
        break

    while True:
        try:
            currentPose = tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time())
            break
            # print(currentPose.transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("exception occurred")
            rate.sleep()
            continue
    current_position = vector_3d_to_array(currentPose.transform.translation)

    trajectory = create_trajectory(pick_target, 0, current_position=current_position, reverse=False)
    trajectory = np.vstack((trajectory, create_trajectory(place_target, 0, current_position=trajectory[-1], reverse=True)))
    grab_i = 2
    leave_i = 4
    grabbed=False
    # open gripper
    print("opening gripper")
    gripper_msg = Float64MultiArray()
    gripper_msg.data = [0, 0.5, 0.5]
    gripper_controller.publish(gripper_msg)
    gripper_controller.publish(gripper_msg)
    time.sleep(2)
    i = 0
    current_target = trajectory[i]
    print(f"current target: {trajectory[0]}")
    # temp_target = np.array([-1, -1])
    while not rospy.is_shutdown():
        rate = rospy.Rate(100)
        # current_target = trajectory[i]
        # current_target = temp_target
        try:
            currentPose = tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time())
            # print(currentPose.transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("exception occurred")
            rate.sleep()
            continue
        current_position = vector_3d_to_array(currentPose.transform.translation)
        currentRotationQuat = currentPose.transform.rotation
        orientation_list = [currentRotationQuat.x, currentRotationQuat.y, currentRotationQuat.z, currentRotationQuat.w]
        _, _, theta = euler_from_quaternion(orientation_list)
        msg, reached_target = calculate_twist_msg(current_position, theta, current_target, 0, reverse=grabbed)
        queenie_driver.publish(msg)
        if reached_target:
            i+= 1
            if i == grab_i:
                gripper_msg.data = [0., 0., 0.]
                gripper_controller.publish(gripper_msg)
                time.sleep(2)
                grabbed=True
            if i == len(trajectory):
                gripper_msg.data = [0., 0.5, 0.5]
                gripper_controller.publish(gripper_msg)
                gripper_controller.publish(gripper_msg)
                time.sleep(2)
                grabbed=False
            if i == len(trajectory):
                return
            current_target = trajectory[i]
        rate.sleep()


def calculate_twist_msg(current_position, theta, target_position, target_theta, reverse=False):
    msg = geometry_msgs.msg.Twist()

    # calculate the target angle:
    target_heading = math.atan2(target_position[1] - current_position[1], target_position[0] - current_position[0])
    if reverse:
        abs_target_heading = abs(target_heading)
        opp_quad_heading  = np.pi - abs_target_heading
        if target_heading >= 0:
            target_heading = -1*opp_quad_heading
        else:
            target_heading = opp_quad_heading

    errorInTheta = target_heading - theta
    
    # print(f"error in theta: {errorInTheta}")
    if errorInTheta > 0.02 and errorInTheta > 0 and not abs(current_position[0] - target_position[0]) < 0.01:
        msg.angular.z = 0.1
    elif errorInTheta < -0.02 and errorInTheta < 0:
        msg.angular.z = -0.1
    else:
        msg.angular.z = 0.0
    # error in position
    if msg.angular.z == 0 and math.sqrt((target_position[0] - current_position[0])**2 + (target_position[1] - current_position[1])**2) > 0.01:
        msg.linear.x = -0.1 if reverse else 0.1
    else:
        msg.linear.x = 0
    if  msg.linear.x == 0 and msg.angular.z == 0:
        return msg, True
    else:
        return msg, False


def vector_3d_to_array(vector_object):
    ar = np.array([0., 0., 0.])
    ar[0] = vector_object.x
    ar[1] = vector_object.y
    ar[2] = vector_object.z
    return ar

def create_trajectory(target_position, target_orientation=None, current_position=None, reverse=False):
    if reverse:
        if current_position is None:
            raise Exception("current position not given and mode is reverse")
        target_direction = target_position - current_position
        target_orientation = math.atan2(target_direction[1], target_direction[0])
        intermediary_point = target_position + np.array([(palm_to_chassis_transform[0])*np.cos(target_orientation), (palm_to_chassis_transform[0])*np.sin(target_orientation), 0]) 
        return np.array([intermediary_point])
    else:    
        intermediary_point_1 = target_position - np.array([(palm_to_chassis_transform[0] + finger_length)*np.cos(target_orientation), (palm_to_chassis_transform[0] + finger_length)*np.sin(target_orientation), 0]) 
        # intermediary_point -= (intermediary_point / np.linalg.norm(intermediary_point)) * finger_length
        intermediary_point_2 = target_position - np.array([(palm_to_chassis_transform[0])*np.cos(target_orientation), (palm_to_chassis_transform[0])*np.sin(target_orientation), 0]) 
        return np.array([intermediary_point_1, intermediary_point_2])



if __name__ == '__main__':
    print("node started")
    listener(PICK, 0, 0)
    # listener(GRAB, 0, 0)