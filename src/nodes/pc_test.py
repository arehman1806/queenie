#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from skimage.metrics import structural_similarity
from scipy.special import rel_entr
from std_msgs.msg import Float32

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_image = None
        # self.pub = rospy.Publisher('/reward_signal', Float32, queue_size=10)
        self.sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.callback)
        self.count = 0

    def callback(self, data):
        for point in data.data:
            print("lololol")

def main():
    rospy.init_node('image_processor', anonymous=True)
    image_processor = ImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
