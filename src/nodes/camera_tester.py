#!/usr/bin/env python3
from cv_bridge import CvBridge
import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

rospy.init_node('camera_tester')
bridge = CvBridge()

# define the names of the two contact topics
contact_topic1 = '/camera/color/image_raw'

# define publishers for each contact topic
# pub1 = rospy.Publisher('/left_finger_contact', Bool, queue_size=1)
# pub2 = rospy.Publisher('/right_finger_contact', Bool, queue_size=1)

# define callback functions to handle the contact messages
def contact1_callback(msg: Image):
    x = msg.data
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    print("data recieved")

# subscribe to the contact topics
rospy.Subscriber(contact_topic1, Image, contact1_callback)
# rospy.Subscriber(contact_topic2, ContactsState, contact2_callback)

# start the node
rospy.spin()
