#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

def mir_pose_publisher():
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('robot_pose', Pose, queue_size=10)
    pub_object = rospy.Publisher('object_pose', Pose, queue_size=10)
    r = rospy.Rate(10.0)
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    while not rospy.is_shutdown():
        try:
            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_coordinates = model_state('queenie', '')
            pose = Pose()
            pose.position.x = model_coordinates.pose.position.x
            pose.position.y = model_coordinates.pose.position.y
            pose.position.z = model_coordinates.pose.position.z
            pose.orientation.x = model_coordinates.pose.orientation.x
            pose.orientation.y = model_coordinates.pose.orientation.y
            pose.orientation.z = model_coordinates.pose.orientation.z
            pose.orientation.w = model_coordinates.pose.orientation.w

        except rospy.ServiceException as e:
            print('Service call failed:' + e)
        pub.publish(pose)

        # try:
        #     model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #     model_coordinates = model_state('large_cuboid', '')
        #     pose = Pose()
        #     pose.position.x = model_coordinates.pose.position.x
        #     pose.position.y = model_coordinates.pose.position.y
        #     pose.position.z = model_coordinates.pose.position.z
        #     pose.orientation.x = model_coordinates.pose.orientation.x
        #     pose.orientation.y = model_coordinates.pose.orientation.y
        #     pose.orientation.z = model_coordinates.pose.orientation.z
        #     pose.orientation.w = model_coordinates.pose.orientation.w

        # except rospy.ServiceException as e:
        #     print('Service call failed:' + e)
        # pub_object.publish(pose)
        r.sleep()


if __name__ == '__main__':
    try:
        mir_pose_publisher()
    except rospy.ROSInterruptException:
        pass
