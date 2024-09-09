#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import time

class WristCameraListener:

    def __init__(self) -> None:
        self.subscriber = rospy.Subscriber(
            "/left_arm_camera/color/image_rect_color", 
            Image, 
            self.callback,
            queue_size=1
            )
        self.bridge = CvBridge()

        # To get the publish rate
        self.last_time = None
        self.time_diff = []


    def callback(self, image_data):
        rospy.loginfo(f"Received an image of size {image_data.width}x{image_data.height}")

        try:
            self.image = self.bridge.imgmsg_to_cv2(image_data)

            cv2.imshow("Left Arm Camera Image", self.image)
            cv2.waitKey(1)  # Display image for 1ms and process any OpenCV window events

            # get the rate of message
            if self.last_time is not None:
                current_time = time.time()
                self.time_diff.append(current_time - self.last_time)
                self.last_time = current_time
            else:
                self.last_time = time.time()
            rate = 1 / np.average(self.time_diff)
            rospy.loginfo(f"Message rate: {rate} Hz")

        except CvBridgeError as e:
            rospy.loginfo(f"Error converting ROS Image to OpenCV: {e}")



def main():
    listener = WristCameraListener()
    rospy.init_node("wrist_camera_image_color_rect")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Wrist Camera Data-logging Module")


if __name__ == "__main__":
    main()