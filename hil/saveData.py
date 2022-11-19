#!/usr/bin/env python3
from hil import *
from hil.saveSensorDataNode import saveDataNode
import numpy as np

# INPUT TOPICS
RGB_TOPIC = "/rgb"
LIDAR_PC_TOPIC = "/cloud"
CAMERA_INFO_TOPIC = "/camera_info"
GPS_TOPIC = "/gps"
# OUTPUT TOPICS
PROJECTED_TOPIC = "/projected"
T_depth_rgb = np.array([
    [0, -1, 0, 0.2],
    [0, 0, -1, -0.19],
    [1, 0, 0, -0.15],
    [0, 0, 0, 1]
])

def main():
    save_data_node = saveDataNode(
        CAMERA_INFO_TOPIC,
        LIDAR_PC_TOPIC,
        RGB_TOPIC,
        GPS_TOPIC
    )
    save_data_node.run()

if __name__ == '__main__':
    main()