#!/usr/bin/env python3 
import rospy
from scipy import signal
from geometry_msgs.msg import WrenchStamped

if __name__ == '__main__':
    rospy.init_node('ft_low_pass_filter')
    filtered_wrench_pub = rospy.Publisher("filtered_ft", WrenchStamped, queue_size=10)
    
    def callback(data):

        # time = data.header.stamp.secs  +  data.header.stamp.nsecs * 1e-9

        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        tx = data.wrench.torque.x
        ty = data.wrench.torque.y
        tz = data.wrench.torque.z

        FX = []
        FY = []
        FZ = []

        TX = []
        TY = []
        TZ = []

        FX.append(fx)
        FY.append(fy)
        FZ.append(fz)

        TX.append(tx)
        TY.append(ty)
        TZ.append(tz)

        fs = 1000 #sampling frequency

        fc = 200 # Cut-off frequency of the filter
        w = fc / (fs / 2) # Normalize the frequency

        b, a = signal.butter(5, w, 'low')


        fx_low = signal.lfilter(b, a, FX) #Forward filter
        fy_low = signal.lfilter(b, a, FY)
        fz_low = signal.lfilter(b, a, FZ)

        tx_low = signal.lfilter(b, a, TX)
        ty_low = signal.lfilter(b, a, TY)
        tz_low = signal.lfilter(b, a, TZ)

        filtered_msg = WrenchStamped()
        filtered_msg.header = data.header
        filtered_msg.wrench.torque.y = ty_low
        filtered_msg.wrench.torque.z = tz_low
        filtered_wrench_pub.publish(filtered_msg)
    rospy.Subscriber("/ft_sensor_topic", WrenchStamped, callback)
    rospy.spin()
    