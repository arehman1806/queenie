import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import tf.transformations as tf

rospy.init_node('get_shortest_distance')
distance_pub = rospy.Publisher('shortest_distance', Float64, queue_size=10)

# set the dimensions of the cuboid
width = 1.0
height = 4.074220
depth = 1

# define global variables to store the object position and quaternion
object_pos = None
object_quaternion = None

# define a callback function to handle the object pose messages
def object_pose_callback(msg):
    global object_pos, object_quaternion
    object_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
    object_quaternion = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    # Get the rotation of the cuboid from the message
    cuboid_rotation_matrix = tf.quaternion_matrix(object_quaternion)[:3, :3]

    # define a callback function to handle the robot pose messages
    def robot_pose_callback(msg):
        robot_pos = np.array([msg.position.x, msg.position.y, msg.position.z])

        # Transform the robot position to the cuboid's coordinate system
        robot_pos_transformed = np.dot(robot_pos - object_pos, cuboid_rotation_matrix)

        # Calculate the shortest distance between any vertex or center of cuboid and center of robot
        dx = max(-width / 2, min(robot_pos_transformed[0], width / 2)) - robot_pos_transformed[0]
        dy = max(-height / 2, min(robot_pos_transformed[1], height / 2)) - robot_pos_transformed[1]
        dz = max(-depth / 2, min(robot_pos_transformed[2], depth / 2)) - robot_pos_transformed[2]

        distance_transformed = np.array([dx, dy, dz])
        distance = np.linalg.norm(np.dot(distance_transformed, cuboid_rotation_matrix.T))

        rospy.loginfo("Shortest distance between cuboid and robot is {:.2f} meters".format(distance))
        distance_pub.publish(distance)

    # subscribe to the robot pose topic
    rospy.Subscriber('robot_pose', Pose, robot_pose_callback)

# subscribe to the object pose topic
rospy.Subscriber('object_pose', Pose, object_pose_callback)

rospy.spin()
