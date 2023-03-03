#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool

rospy.init_node('contact_listener')

# define the names of the two contact topics
left_finger_contact_topic = 'left_finger_contact_states'
right_finger_contact_topic = 'right_finger_contact_states'
palm_contact_topic = 'palm_contact_states'

# define publishers for each contact topic
pub1 = rospy.Publisher('/left_finger_contact', Bool, queue_size=1)
pub2 = rospy.Publisher('/right_finger_contact', Bool, queue_size=1)
palm_pub = rospy.Publisher('/palm_contact', Bool, queue_size=1)

# define callback functions to handle the contact messages
def contact1_callback(msg):
    if len(msg.states) > 0:
        pub1.publish(True)
    else:
        pub1.publish(False)

def contact2_callback(msg):
    if len(msg.states) > 0:
        pub2.publish(True)
    else:
        pub2.publish(False)

def palm_contact_callback(msg):
    if len(msg.states) > 0:
        palm_pub.publish(True)
    else:
        palm_pub.publish(False)

# subscribe to the contact topics
rospy.Subscriber(left_finger_contact_topic, ContactsState, contact1_callback)
rospy.Subscriber(right_finger_contact_topic, ContactsState, contact2_callback)
rospy.Subscriber(palm_contact_topic, ContactsState, palm_contact_callback)

# start the node
rospy.spin()
