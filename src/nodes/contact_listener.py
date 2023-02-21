#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool

rospy.init_node('contact_listener')

# define the names of the two contact topics
contact_topic1 = 'left_finger_contact_states'
contact_topic2 = 'right_finger_contact_states'

# define publishers for each contact topic
pub1 = rospy.Publisher('/left_finger_contact', Bool, queue_size=1)
pub2 = rospy.Publisher('/right_finger_contact', Bool, queue_size=1)

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

# subscribe to the contact topics
rospy.Subscriber(contact_topic1, ContactsState, contact1_callback)
rospy.Subscriber(contact_topic2, ContactsState, contact2_callback)

# start the node
rospy.spin()
