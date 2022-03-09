#!/usr/bin/env python3
from HIL import *
from HIL.lidar2camNode import lidar2camNode
import numpy as np

RGB_TOPIC = "/rgb"
LIDAR_PC_TOPIC = "/cloud"
CAMERA_INFO_TOPIC = "/camera_info"
T_depth_rgb = np.array([
    [1, 0, 0, -0.059076450765132904],
    [0, 1, 0, -0.0003348544123582542],
    [0, 0, 1, -0.00001691722536634188],
    [0, 0, 0, 1]
])

def main():
    lidar2cam_node = lidar2camNode(
        CAMERA_INFO_TOPIC,
        LIDAR_PC_TOPIC,
        RGB_TOPIC,
        T_depth_rgb
    )
    lidar2cam_node.run()

if __name__ == '__main__':
    main()