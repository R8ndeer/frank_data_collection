import rosbag
from pathlib import Path

bag_path = Path(__file__).parent / "../rosbag/2024-09-10-15-28-22.bag"
bag = rosbag.Bag(bag_path)
msg_count = bag.get_message_count()
print(f"message count: {msg_count}")

for topic, msg, ts in bag.read_messages():
    print(msg.position)
    print(msg.velocity)
    break