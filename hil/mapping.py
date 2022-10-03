#!/usr/bin/env python3
from hil import *
import numpy as np
from hil.mapNode import mapNode
RGB_TOPIC = "/rgb"
LIDAR_PC_TOPIC = "/cloud"
CAMERA_INFO_TOPIC = "/camera_info"
GPS_TOPIC = "/gps"
PROJECTED_TOPIC = "/projected"

T_depth_rgb = np.array([
    [0, -1, 0, 0.2],
    [0, 0, -1, -0.19],
    [1, 0, 0, -0.15],
    [0, 0, 0, 1]
])

def main():
    map_node = mapNode(
        CAMERA_INFO_TOPIC,
        LIDAR_PC_TOPIC,
        RGB_TOPIC,
        GPS_TOPIC,
        PROJECTED_TOPIC,
        T_depth_rgb
    )
    map_node.run()

if __name__ == '__main__':
    main()