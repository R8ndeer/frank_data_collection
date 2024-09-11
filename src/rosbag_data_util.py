#!/usr/bin/env python

import os
import copy

import rospy
import bisect
import cv2
from cv_bridge import CvBridge
import numpy as np
import pandas as pd
from tqdm import tqdm
from rosbag import Bag
from pathlib import Path

# The following import is temporary for coding purpose, change if there is error
from rosbag_read_utils.Rosbag_Reader import RosbagReader


class FormatConverter:
    def __init__(self) -> None:
        pass


def read_observations(obs_topic: str, rosbag: Bag) -> pd.DataFrame:
    obs = pd.DataFrame(columns=["timestamp"] + [f"joint_{i}" for i in range(7)])

    # initialise progress bar
    pbar = tqdm(total=rosbag.get_message_count(obs_topic))

    for topic, msg, ts in rosbag.read_messages(obs_topic):
        pbar.update(1)
        timestamp = pd.to_datetime(ts.to_sec(), unit="s")
        obs.loc[len(obs)] = [timestamp] + [msg.position[0],
                                           msg.position[1],
                                           msg.position[2],
                                           msg.position[3],
                                           msg.position[4],
                                           msg.position[5],
                                           msg.position[6]] 
    # return the first 5 rows
    obs.head()

    return obs


def read_images(img_topic: str, rosbag: Bag) -> pd.DataFrame:
    images = pd.DataFrame(columns=["timestamp", "cv::Mat"])

    bridge = CvBridge()

    pbar = tqdm(total=rosbag.get_message_count(img_topic))

    for topic, msg, ts in rosbag.read_messages(img_topic):
        pbar.update(1)
        timestamp = pd.to_datetime(ts.to_sec(), unit='s')
        cv_image_msg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        images.loc[len(images)] = [timestamp, cv_image_msg]

    return images


def find_closest_image(obs_row: pd.Series, images: pd.DataFrame, output_dir: str):
    """
    find the closest image to an observation, assuming observation messages are at a higher freq.

    Args:
        obs (pd.Series): one (row) of the observations obtained by read_observations() function
        images (pd.DataFrame): images obtained by read_images() function
    """

    def find_closest_image_index(obs_timestamp: float, image_timestamps: list) -> int:
        pos = bisect.bisect_left(image_timestamps, obs_timestamp)

        # check boundary conditions
        if pos == 0:
            closest_image_idx = 0
        elif pos == len(image_timestamps):
            closest_image_idx = pos - 1
        else:
            # check whether before or after is a closer timestamp
            before = image_timestamps[pos - 1]  # due to bisect_left()
            after = image_timestamps[pos]
            if abs(after - obs_timestamp) < abs(before - obs_timestamp):
                closest_image_idx = pos
            else:
                closest_image_idx = pos - 1
            
        return closest_image_idx

    # Create output directory if it does not exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    closest_image = None

    obs_timestamp = obs_row.loc["timestamp"]
    image_timestamps = images.loc[:, "timestamp"].to_list()

    closest_image_index = find_closest_image_index(obs_timestamp, image_timestamps)
    closest_image = images.loc[closest_image_index, "cv::Mat"]
    
    if closest_image is not None:
        # Construct image file name
        image_file = os.path.join(output_dir, 'closest_image.png')

        # Save image
        cv2.imwrite(image_file, closest_image)
        rospy.loginfo(f"Saved closest image: {image_file}")
    else:
        rospy.loginfo("No images found in the specified topic.")
    
    return closest_image
    

def sync_image(obs: pd.DataFrame, images: pd.DataFrame, output_dir: str) -> list: 
    cv_image = []

    pbar = tqdm(range(obs.shape[0]))
    for idx, row in obs.iterrows():
        pbar.update(1)
        image = find_closest_image(row, images, output_dir)
        cv_image.append(image)
    
    return cv_image



def main():
    ROS_BAG_PATH = Path(__file__).parent / "../rosbag/2024-09-10-16-51-40.bag"
    IMG_DIR = Path(__file__).parent / "../image_data"
    OBS_TOPIC = "/left_arm/joint_states"
    IMG_TOPIC = "/left_arm_camera/color/image_rect_color"

    # Initialise bag reader
    bag_reader = RosbagReader()
    bag = bag_reader.get_rosbag(path_to_bag=ROS_BAG_PATH)

    # Read and sync robot obs and image data
    obs = read_observations(OBS_TOPIC, bag)
    obs = bag_reader.resample(obs, freq=50.0)  # note this sets the timestamp column as row index
    obs = obs.reset_index()
    images = read_images(IMG_TOPIC, bag)
    cv_image = []

    pbar = tqdm(range(obs.shape[0]))
    for idx, row in obs.iterrows():
        pbar.update(1)
        image = find_closest_image(row, images, IMG_DIR)
        cv_image.append(image)

#    cv_image = read_and_sync_image(obs, IMG_DIR, IMG_TOPIC, bag, bag_reader)
#
    # image playback
    for image in cv_image:
        cv2.imshow("Left Arm Camera Image", image)
        cv2.waitKey(1)

    print(obs.tail())
    print(images.tail())

if __name__ == "__main__":
    main()
