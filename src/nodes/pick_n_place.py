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

class pick_n_place():

    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.current_position = None

    def start_moving(self):
        hasReachedTarget = False
        while not hasReachedTarget:
            try:
                self.currentPose = self.tf_buffer.lookup_transform("odom", "robot_footprint", rospy.Time()).transform
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("exception occurred")
                continue
            current_position = self.currentPose.transform.translation
            currentRotationQuat = self.currentPose.transform.rotation
            # errorInPositon 
            msg = Twist()
            msg.linear.x = min(0.2, 0.5* math.sqrt(current_position))
            # linear = 
            

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(rospy.get_caller_id() + "I heard %s", data.data)

obtect_to_pick = {
    "x": 0.128064,
    "y": 0.115884,
    "z": 0.258202
}
object_to_pick = np.array([0.128064, 0.115884, 0.258202])

def listener():

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

    place_target = np.array([2.0, -1.0, 0])
    place_target_theta = 0

    # estimate pick target
    pick_target = np.array([0., 0., 0.])
    while True or not rospy.is_shutdown():
        try:
            object_pose = tfBuffer.lookup_transform("world", "unit_box", rospy.Time())
            palm_to_chassis = tfBuffer.lookup_transform("chassis", "palm", rospy.Time())
        except:
            continue
        object_pose_trans = object_pose.transform.translation
        print(object_pose_trans)
        pick_target[0] = object_pose_trans.x - object_to_pick[0] - palm_to_chassis.transform.translation.x
        pick_target[1] = object_pose_trans.y
        pick_target[2] = object_pose_trans.z
        print(pick_target)
        break

    trajectory = np.array([
        pick_target - np.array([0.2, 0, 0]),
        pick_target,
        place_target
    ])
    pick_i = 1
    place_i = 2
    
    # open gripper
    print("opening gripper")
    gripper_msg = Float64MultiArray()
    gripper_msg.data = [0, 0.5, 0.5]
    gripper_controller.publish(gripper_msg)
    gripper_controller.publish(gripper_msg)
    time.sleep(2)
    i = 0
    while not rospy.is_shutdown():
        rate = rospy.Rate(10)
        current_target = trajectory[i]
        try:
            currentPose = tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time())
            # print(currentPose.transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("exception occurred")
            rate.sleep()
            continue
        current_position = np.array([0.,0.,0.])
        current_position[0] = currentPose.transform.translation.x
        current_position[1] = currentPose.transform.translation.y
        current_position[2] = currentPose.transform.translation.z

        currentRotationQuat = currentPose.transform.rotation
        orientation_list = [currentRotationQuat.x, currentRotationQuat.y, currentRotationQuat.z, currentRotationQuat.w]
        _, _, theta = euler_from_quaternion(orientation_list)
        # print(f"current position: {current_position}. current orientation: {theta}")
        msg, reached_target = calculate_twist_msg(current_position, theta, current_target, 0)
        queenie_driver.publish(msg)
        if reached_target:
            if i == len(trajectory) - 1:
                for _ in range(10):
                    queenie_driver.publish(msg)
                    rate.sleep()
                gripper_msg.data = [0.2, 0.5, 0.5]
                gripper_controller.publish(gripper_msg)
                time.sleep(2)
                return
            # time.sleep(2)
            if i == pick_i:
                print("closing gripper")
                gripper_msg.data = [0, 0, 0]
                gripper_controller.publish(gripper_msg)
                time.sleep(2)
                gripper_msg.data = [0.2, 0, 0]
                gripper_controller.publish(gripper_msg)
                time.sleep(2)
            # elif i == place_i:
            #     gripper_msg.data = [0.2, 0.5, 0.5]
            #     gripper_controller.publish(gripper_msg)
            #     time.sleep(2)
            i += 1

        # print(msg)
        rate.sleep()
def calculate_twist_msg(current_position, theta, target_position, target_theta):
    msg = geometry_msgs.msg.Twist()
    if abs(current_position[0] - target_position[0]) < 0.01:
        msg.linear.x = 0
        # print(f"target theta: {theta}, current theta: {target_theta}. error in theta: {target_theta - theta}")
        if target_theta - theta > 0.02:
            msg.angular.z = 0.1
            return msg, False
        elif target_theta - theta < -0.02:
            msg.angular.z = -0.1
            return msg, False
        else:
            msg.angular.z = 0.0
        return msg, True
    # error in theta
    errorInTheta = math.atan2(target_position[1] - current_position[1], target_position[0] - current_position[0]) - theta
    # print(f"error in theta{errorInTheta}")
    if errorInTheta > 0.02 and errorInTheta > 0 and not abs(current_position[0] - target_position[0]) < 0.01:
        msg.angular.z = 0.1
    elif errorInTheta < -0.02 and errorInTheta < 0:
        msg.angular.z = -0.1
    else:
        msg.angular.z = 0.0
    # error in position
    if msg.angular.z == 0 and math.sqrt((target_position[0] - current_position[0])**2 + (target_position[1] - current_position[1])**2) > 0.01:
        msg.linear.x = 0.1
    else:
        msg.linear.x = 0
    return msg, False
    

if __name__ == '__main__':
    print("node started")
    listener()