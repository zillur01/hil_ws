#!/usr/bin/env python3
from hil import *
from hil.calibrationNodes import arucoMappingNode
import numpy as np

RGB_TOPIC = "/rgb"
CAMERA_INFO_TOPIC = "/camera_info"

def main():
    desired_ids = [5, 11]
    marker_length = .145
    arucoMapping_node = arucoMappingNode(
        CAMERA_INFO_TOPIC,
        RGB_TOPIC,
        desired_ids,
        marker_length
    )
    arucoMapping_node.run()

if __name__ == '__main__':
    main()