#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class WristCameraListener:

    def __init__(self) -> None:
        self.subscriber = rospy.Subscriber(
            "/left_arm_camera/color/image_rect_color", 
            Image, 
            self.callback,
            queue_size=1
            )


    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def main():
    listener = WristCameraListener()
    rospy.init_node("wrist_camera_image_color_rect")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Wrist Camera Data-logging Module")


if __name__ == "__main__":
    main()