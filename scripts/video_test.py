#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path

def play_video_from_bag(bag_path, topic_name):
    """
    Reads a ROS bag file and plays back the video recorded on a specific topic.

    Args:
        bag_path (str): Path to the ROS bag file.
        topic_name (str): The topic name where the video (Image messages) is recorded.
    """
    # Initialize CvBridge
    bridge = CvBridge()

    # Open the ROS bag file
    bag = rosbag.Bag(bag_path, 'r')

    # Iterate through the messages in the bag file
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if topic == topic_name:
            try:
                # Convert the ROS Image message to an OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Display the image
                cv2.imshow("Video Playback", cv_image)

                # Wait for 1ms to allow OpenCV to process the window events
                key = cv2.waitKey(1)
                if key == 27:  # ESC key to exit early
                    break

            except Exception as e:
                rospy.logerr(f"Error converting image: {e}")
    
    # Close the bag file and OpenCV windows
    bag.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('rosbag_video_player', anonymous=True)

        # Path to the ROS bag file
        bag_path = Path(__file__).parent / "../rosbag/2024-09-10-16-51-40.bag"  # Replace with the actual path to your bag file
        
        # Topic name where the video (Image messages) is recorded
        topic_name = "/left_arm_camera/color/image_rect_color"  # Replace with the actual topic name in the bag file

        # Play the video from the ROS bag
        play_video_from_bag(bag_path, topic_name)

    except rospy.ROSInterruptException:
        pass