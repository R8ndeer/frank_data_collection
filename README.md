# frank_data_collection
In src:

data_collection.py is just subscribing to the topics and read data directly.

rosbag_data_util leverages on Alex's rosbag reader, resample the observations and syncing the image data to the observations (joint angles).
