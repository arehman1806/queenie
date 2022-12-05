#!/usr/bin/env python3
import rospy
from rospy import Subscriber
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from actionlib import SimpleActionServer
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class moveToTCPPose():

    def __init__(self) -> None:
        # self.current_pose_subscriber = Subscriber("current_pose_suscriber", )


        # self.moveServer = SimpleActionServer("move_to_TCP_pose", Twist, self.start_moving)
        # self.moveServer.start()
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
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('queenie_tf_listener', anonymous=True)
    target_x = -4
    target_y = -4
    target_theta = 0
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    queenie_driver = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    while not rospy.is_shutdown():
        rate = rospy.Rate(10)
        try:
            # print(tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time(0)))
            currentPose = tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time())
            # print(currentPose.transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("exception occurred")
            rate.sleep()
            continue
        current_position = currentPose.transform.translation
        currentRotationQuat = currentPose.transform.rotation
        orientation_list = [currentRotationQuat.x, currentRotationQuat.y, currentRotationQuat.z, currentRotationQuat.w]
        _, _, theta = euler_from_quaternion(orientation_list)
        # print(f"current position: {current_position}. current orientation: {theta}")
        msg = geometry_msgs.msg.Twist()
        if abs(current_position.x - target_x) < 0.01:
            msg.linear.x = 0
            msg.angular.z = 0
            for _ in range(10):
                queenie_driver.publish(msg)
                rate.sleep()
            print("reached target")
            print(current_position)
            break
        # error in theta
        errorInTheta = math.atan2(target_y - current_position.y, target_x - current_position.x) - theta
        if errorInTheta > 0.01 and errorInTheta > 0:
            msg.angular.z = 0.1
        elif errorInTheta < -0.01 and errorInTheta < 0:
            msg.angular.z = -0.1
        else:
            msg.angular.z = 0
        # error in position
        if msg.angular.z == 0 and math.sqrt((target_x - current_position.x)**2 + (target_y - current_position.y)**2) > 0.01:
            msg.linear.x = 0.2
        else:
            msg.linear.x = 0
        

        queenie_driver.publish(msg)
        # print(msg)
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    print("node started")
    listener()