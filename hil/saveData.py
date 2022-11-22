#!/usr/bin/env python3
from hil import *
from hil.saveSensorDataNode import saveDataNode
import numpy as np
from hil.globals import *

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