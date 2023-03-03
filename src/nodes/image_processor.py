#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.metrics import structural_similarity
from scipy.special import rel_entr
from std_msgs.msg import Float32

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_image = None
        self.pub = rospy.Publisher('/reward_signal', Float32, queue_size=10)
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.count = 0

    def callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Preprocess image
        # cv_image = cv2.resize(cv_image, (84, 84))  # Resize to 84x84 for faster processing
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        cv_image = cv_image.astype(np.float32) / 255.0  # Normalize to [0, 1]

        # Calculate MSE between current and previous images
        if self.prev_image is not None and self.count > 5:
            mse = np.mean((cv_image - self.prev_image) ** 2)
            # (score, diff) = structural_similarity(cv_image, self.prev_image, full=True)
            # reward = 1.0 / (mse + 1e-6)  # Add small constant to avoid division by zero
            pdf_old = cv2.calcHist([self.prev_image],[0],None,[16],[0,1])
            pdf_old = pdf_old / pdf_old.sum()
            pdf_new = cv2.calcHist([cv_image],[0],None,[16],[0,1])
            pdf_new = pdf_new / pdf_new.sum()
            kl_div = rel_entr(pdf_old, pdf_new)
            kl_div[np.isinf(kl_div)] = 0
            score = kl_div.sum()

            self.pub.publish(score)
            print("{}".format(score))
            self.prev_image = cv_image
            self.count = 0

        # Store current image as previous image
        if self.prev_image is None:
            self.prev_image = cv_image
        self.count += 1

def main():
    rospy.init_node('image_processor', anonymous=True)
    image_processor = ImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
