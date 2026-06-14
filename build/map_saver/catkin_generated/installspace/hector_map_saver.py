#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image
import numpy as np
import os

def save_map(msg, filename_prefix):
    filename_prefix = os.path.expanduser(filename_prefix)
    output_dir = os.path.dirname(filename_prefix)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    # ROS map: -1=unknown, 0=free, 100=occupied
    img = np.zeros(data.shape, dtype=np.uint8)
    img[data==0] = 254        # free -> white
    img[data==100] = 0        # occupied -> black
    img[data==-1] = 205       # unknown -> gray

    img = Image.fromarray(img, mode='L')
    img.save(filename_prefix + '.pgm')

    map_yaml = {
        'image': os.path.basename(filename_prefix + '.pgm'),
        'resolution': msg.info.resolution,
        'origin': [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    with open(filename_prefix + '.yaml', 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=False)
    rospy.loginfo("Map saved to {}.pgm + {}.yaml".format(filename_prefix, filename_prefix))

if __name__ == '__main__':
    rospy.init_node('hector_map_saver')
    topic = rospy.get_param('~map_topic', '/map')
    filename = rospy.get_param('~filename', os.path.expanduser('~/robot_graduation/maps/hector_map'))
    msg = rospy.wait_for_message(topic, OccupancyGrid)
    save_map(msg, filename)
