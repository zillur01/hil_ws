#!/usr/bin/env python3
from calibration.calibrationNodes import arucoMappingNode
import numpy as np
from hil.globals import * 

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