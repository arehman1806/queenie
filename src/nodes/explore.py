#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from queenie.msg import ExtremePoints
from std_msgs.msg import Float64, Float32
import time
import math
import PyKDL
import copy
import numpy as np

class ExploreObject(object):

    def __init__(self) -> None:

        self.distance_threshold_explore = 3
        self.go_around_distance_threshold = 3
        self.min_distance_to_handle_threshold = 5

        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.extreme_points_sub = rospy.Subscriber("/extreme_points", ExtremePoints, self.extreme_points_cb)
        self.min_distance_sub = rospy.Subscriber("/min_distance_to_object", Float64, self.min_distance_cb)
        self.queenie_pose_sub = rospy.Subscriber("robot_pose", Pose, self.queenie_pose_cb)
        self.min_distance_to_handle_sub = rospy.Subscriber("/min_distance_to_handle", Float32, self.min_distance_to_handle_cb)
        self.angle_to_handle_sub = rospy.Subscriber("/angle_to_handle", Float32, self.angle_to_handle_cb)

        self.extreme_points = ExtremePoints()
        self.min_distance = Float64().data
        self.queenie_pose = [0,0,0]
        self.min_distance_to_handle = 0
        self.angle_to_handle = 0

        self.rate = rospy.Rate(50)
    
    def min_distance_to_handle_cb(self, data):
        self.min_distance_to_handle = data.data
    
    def angle_to_handle_cb(self, data):
        self.angle_to_handle = data.data
    
    def extreme_points_cb(self, data:ExtremePoints):
        self.extreme_points = data

    def min_distance_cb(self, data:Float64):
        self.min_distance = data.data

    def queenie_pose_cb(self, data):
        x = data.position.x
        y = data.position.y

        orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                data.orientation.y,
                                                data.orientation.z,
                                                data.orientation.w)

        euler_orientation = orientation.GetRPY()
        yaw = euler_orientation[2]


        # Update internal Pose variable
        self.queenie_pose = np.array([x, y, yaw])
    
    def explore(self):

        while True:
            
            # what to do when handle is in sight
            if self.min_distance_to_handle < self.min_distance_to_handle_threshold and self.min_distance/self.min_distance_to_handle > 0.5:
                print("handle is in sight: angle: {}, distance: {}".format(self.angle_to_handle - np.pi / 2, self.min_distance_to_handle))
                opposite = -math.cos(self.angle_to_handle) * self.min_distance_to_handle
                # print("handle IS. the robot should move {} to left/right. ratio: {}".format(opposite, self.min_distance/self.min_distance_to_handle))
                success = self.approach_handle()
                if success:
                    break

            elif self.min_distance < self.distance_threshold_explore:
                self.reverse()
            elif True:
                
                goal = self.extrapolate_goal()
                print("robot should go to {}, {}, {}".format(goal[0], goal[1], goal[2]))
                success = self.move(goal)
                
            else:

                print("at distance: {} from object".format(self.min_distance))
            self.rate.sleep()
    
    def approach_handle(self):
        target_theta = self.queenie_pose[2] + self.angle_to_handle -np.pi/2
        reached = False
        while not reached:
            msg, reached = self._calculate_twist_theta_correction(target_theta)
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()
            
        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()

        while self.min_distance_to_handle > 1.7:
            msg = Twist()
            msg.linear.x = 0.1
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()
        
        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()
        return True


    
    def extrapolate_goal(self):
        leftmost = self.extreme_points.leftmost
        rightmost = self.extreme_points.rightmost

        # Calculate the distance between the two points
        # d = math.sqrt((rightmost.point.x - leftmost.point.x)**2 + (rightmost.point.y - leftmost.point.y)**2)

        # Calculate the direction of the line segment
        theta = math.atan2(rightmost.point.y - leftmost.point.y, rightmost.point.x - leftmost.point.x)

        # Calculate the endpoint that is 3 units away from the rightmost point in the direction of the line segment
        x_end = rightmost.point.x + self.go_around_distance_threshold*math.cos(theta)
        y_end = rightmost.point.y + self.go_around_distance_threshold*math.sin(theta)
        yaw = math.atan2(rightmost.point.y - y_end, rightmost.point.x - x_end)

        return np.array([x_end, y_end, yaw])

    def move(self, goal):
        while True:
            msg, reached =  self._calculate_twist_msg(goal)
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()
            if reached:
                reached = False
                while not reached:
                    msg, reached = self._calculate_twist_theta_correction(goal[2])
                    self.cmd_vel_publisher.publish(msg)
                    self.rate.sleep()
                break
           
        
        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()
        return True
    
    def _calculate_twist_msg(self, target_position):
        msg = Twist()
        
        # error in theta
        errorInTheta = math.atan2(target_position[1] - self.queenie_pose[1], target_position[0] - self.queenie_pose[0]) - self.queenie_pose[2]
        print(f"error in theta{errorInTheta}")
        if errorInTheta > 0.04 and errorInTheta > 0 and not abs(self.queenie_pose[0] - target_position[0]) < 0.01:
            msg.angular.z = 0.1
        elif errorInTheta < -0.02 and errorInTheta < 0:
            msg.angular.z = -0.1
        else:
            msg.angular.z = 0.0
        # error in position
        if msg.angular.z == 0 and math.sqrt((target_position[0] - self.queenie_pose[0])**2 + (target_position[1] - self.queenie_pose[1])**2) > 0.1:
            msg.linear.x = 0.1
        else:
            msg.linear.x = 0
        if msg.linear.x == 0 and msg.angular.z == 0:
            return msg, True
        return msg, False

    def _calculate_twist_theta_correction(self, target_theta):
        msg = Twist()
        msg.linear.x = 0
        print(f"current theta: {self.queenie_pose[2]}, target theta: {target_theta}. error in theta: {target_theta - self.queenie_pose[2]}")
        if target_theta - self.queenie_pose[2] > 0.02:
            msg.angular.z = 0.1
            return msg, False
        elif target_theta - self.queenie_pose[2] < -0.02:
            msg.angular.z = -0.1
            return msg, False
        else:
            msg.angular.z = 0.0
        return msg, True
    

    def reverse(self):
        # reverse until the closest point on the object is at least distance threshold away
        while self.min_distance < self.distance_threshold_explore:
            msg = Twist()
            msg.linear.x = -0.1
            msg.angular.z = 0
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()

        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()




    def _move_to_goal(self):
        pass

if __name__ == "__main__":
    rospy.init_node("explore")
    ex = ExploreObject()
    time.sleep(2)
    ex.explore()
    rospy.spin()