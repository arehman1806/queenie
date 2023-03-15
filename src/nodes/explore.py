#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, PointStamped, Point
from queenie.msg import ExtremePoints
from std_msgs.msg import Float64, Float32
import time
import math
import PyKDL
import copy
import numpy as np

class ExploreObject(object):

    def __init__(self) -> None:

        self.distance_threshold_explore = 2
        self.go_around_distance_threshold = 2
        self.min_distance_to_handle_threshold = 5

        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.handle_centroid_sub = rospy.Subscriber("/handle_centroid_transformed", PointStamped, self.handle_centroid_cb)
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
        self.handle_centroid = PointStamped()

        self.approach_attempted = False

        self.rate = rospy.Rate(50)

    def handle_centroid_cb(self, data: PointStamped):
        self.handle_centroid = data
    
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
            print(self.check_alignment_with_handle())
            
            # what to do when handle is in sight
            if self.min_distance_to_handle < self.min_distance_to_handle_threshold and not self.approach_attempted:
                # print("handle is in sight: angle: {}, distance: {}".format(self.angle_to_handle - np.pi / 2, self.min_distance_to_handle))
                opposite = -math.cos(self.angle_to_handle) * self.min_distance_to_handle
                # print("handle IS. the robot should move {} to left/right. ratio: {}".format(opposite, self.min_distance/self.min_distance_to_handle))
                success = self.approach_handle()
                self.approach_attempted = True
                if success:
                    self.approach_attempted = False
                    break

            elif self.min_distance < self.distance_threshold_explore:
                self.reverse()
            elif True:
                if self.approach_attempted:
                    goal = self.extrapolate_goal(True)
                    self.approach_attempted = False
                    print("extrapolating bw pc centroid and handle")
                else:
                    goal = self.extrapolate_goal()
                    print("extrapolating bw leftmost and rightmost")
                print("robot should go to {}, {}, {}".format(goal[0], goal[1], goal[2]))
                success = self.move(goal)
                
            else:

                print("at distance: {} from object".format(self.min_distance))
            self.rate.sleep()
    
    def approach_handle(self):
        target_theta = (self.queenie_pose[2] + self.angle_to_handle -np.pi/2)
        if target_theta < -np.pi or target_theta > np.pi:
            target_theta = target_theta % np.pi
        reached = False
        while not reached:
            msg, reached = self._calculate_twist_theta_correction(target_theta)
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()
            
        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()
        if not self.check_alignment_with_handle():
            return False

        while self.min_distance_to_handle > 1.1:
            msg = Twist()
            msg.linear.x = 0.3
            self.cmd_vel_publisher.publish(msg)
            self.rate.sleep()
        
        for _ in range(5):
            self.cmd_vel_publisher.publish(Twist())
            self.rate.sleep()
        return True


    def check_alignment_with_handle(self):
        # Calculate the vectors for the lines connecting the points
        vector_handle_centroid_to_point_centroid = np.array([self.extreme_points.point_centroid.point.x - self.handle_centroid.point.x,
                                                              self.extreme_points.point_centroid.point.y - self.handle_centroid.point.y])
        vector_queenie_pose_to_handle_centroid = np.array([self.queenie_pose[0] - self.handle_centroid.point.x,
                                                            self.queenie_pose[1] - self.handle_centroid.point.y])

        # Calculate the angle between the vectors using atan2
        angle_point_centroid = np.arctan2(vector_handle_centroid_to_point_centroid[1], vector_handle_centroid_to_point_centroid[0])
        angle_queenie_pose = np.arctan2(vector_queenie_pose_to_handle_centroid[1], vector_queenie_pose_to_handle_centroid[0])

        # Calculate the angle difference
        angle_difference = angle_point_centroid - angle_queenie_pose

        # Ensure the angle difference is within the range [-pi, pi]
        angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

        # Convert the angle difference to degrees
        angle_degrees = np.degrees(angle_difference)

        if 180 - abs(angle_degrees) < 40:
            return True

        return False



    
    def extrapolate_goal(self, center_to_handle=False):

        if center_to_handle:
            leftmost = self.extreme_points.point_centroid
            rightmost = self.handle_centroid
        else:
            leftmost = self.extreme_points.leftmost
            rightmost = self.extreme_points.rightmost

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
        # print(f"error in theta{errorInTheta}")
        if errorInTheta > 0.04 and errorInTheta > 0 and not abs(self.queenie_pose[0] - target_position[0]) < 0.01:
            msg.angular.z = 0.5
        elif errorInTheta < -0.02 and errorInTheta < 0:
            msg.angular.z = -0.5
        else:
            msg.angular.z = 0.0
        # error in position
        if msg.angular.z == 0 and math.sqrt((target_position[0] - self.queenie_pose[0])**2 + (target_position[1] - self.queenie_pose[1])**2) > 0.1:
            msg.linear.x = 0.4
        else:
            msg.linear.x = 0
        if msg.linear.x == 0 and msg.angular.z == 0:
            return msg, True
        return msg, False

    def _calculate_twist_theta_correction(self, target_theta):
        msg = Twist()
        msg.linear.x = 0
        # print(f"current theta: {self.queenie_pose[2]}, target theta: {target_theta}. error in theta: {target_theta - self.queenie_pose[2]}")
        if target_theta - self.queenie_pose[2] > 0.02:
            msg.angular.z = 0.5
            return msg, False
        elif target_theta - self.queenie_pose[2] < -0.02:
            msg.angular.z = -0.5
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
    time.sleep(1)
    # while not rospy.is_shutdown():
    #     print(ex.check_alignment_with_handle())
    ex.explore()
    # rospy.spin()