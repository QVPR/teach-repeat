#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage, Image

import cv2
import cv_bridge
import numpy as np

class miro_ball_detection:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # subscribe to the touch sensor message
        self.sub_image = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.got_image_data, queue_size=1)
        # publish motor commands
        self.pub_image = rospy.Publisher("/miro/filtered_image", Image, queue_size=0)

    def got_image_data(self, msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
            return

        image = self.filter_image(image)
        
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "mono8"))
        except cv_bridge.CvBridgeError as e:
            print(e)

    def filter_image(self, image):
        image = np.float32(image)

        # gamma correction
        image = image * 1/255
        cv2.pow(image, 2.2, image)

        blue, green, red = cv2.split(image)
        combined = cv2.add(blue, cv2.add(green, red))

        # chromacity
        red = cv2.divide(red, combined)
        green = cv2.divide(green, combined)
        blue = cv2.divide(blue, combined)

        yellow = cv2.add(green, red)

        _, yellowT = cv2.threshold(yellow, 0.9, 1.0, cv2.THRESH_BINARY)
        yellowT = cv2.morphologyEx(yellowT, cv2.MORPH_CLOSE,np.ones((3,3),np.float))
        yellowT = cv2.morphologyEx(yellowT, cv2.MORPH_OPEN,np.ones((5,5),np.float))

        return np.uint8(yellowT * 255)




if __name__ == "__main__":
    # intialise this program as a ROS node
    rospy.init_node("miro_ball_detection")
    # start the demo program
    demo = miro_ball_detection()
    # spin makes the program run while ROS is running
    rospy.spin()