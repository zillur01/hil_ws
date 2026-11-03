#!/usr/bin/env python3
"""Extracts a recorded drive into files: time-synced RGB images (img/im_N.png), depth maps
(depth_img/depth_imN.npy, metres) and the GNSS/INS state at that moment (raw_gps/raw_gps_N.npy:
lat, lon, height, roll, pitch, compass heading deg, stamp, vn, ve), plus camera_info.npz.

Parameter `input_bag_path`: the ros2 bag being played (only used to name the output directory,
<bag dir>_extracted/<bag name>/). Normally started by save_data.launch.py, which also plays the bag."""
import os

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hil_common import origin
from hil_common.globals import CAMERA_INFO_TOPIC, DEPTH_IM_TOPIC, RGB_TOPIC
from hil_common.gnss import GnssState
from hil_common.ros_image import depth_msg_to_meters
from hil_common.utils import get_camera_info, get_dist_matrix, get_img_size, get_projection_matrix, stamp_to_sec

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync


def output_dir_for_bag(bag_path):
    bag_path = os.path.normpath(bag_path)  # ros2 bags are directories: drop a trailing '/'
    bag_filename = os.path.splitext(os.path.basename(bag_path))[0]
    return os.path.join(os.path.dirname(bag_path) + "_extracted", bag_filename)


class SaveDataNode(Node):
    def __init__(self):
        super().__init__('save_data_node')
        bag_path = self.declare_parameter('input_bag_path', '').value
        if not bag_path:
            raise RuntimeError("Parameter 'input_bag_path' is required")
        self.fileDir = output_dir_for_bag(bag_path)
        self.get_logger().info(f"Saving to {self.fileDir}")
        self.img_path = os.path.join(self.fileDir, 'img')
        self.raw_gps_path = os.path.join(self.fileDir, 'raw_gps')
        self.depth_path = os.path.join(self.fileDir, 'depth_img')
        for _path in [self.fileDir, self.img_path, self.depth_path, self.raw_gps_path]:
            os.makedirs(_path, exist_ok=True)

        self.camera_info = get_camera_info(self, CAMERA_INFO_TOPIC)
        if self.camera_info is None:
            raise RuntimeError(f"No camera info received on {CAMERA_INFO_TOPIC}")
        self.P = get_projection_matrix(self.camera_info)
        self.D = get_dist_matrix(self.camera_info)
        np.savez(os.path.join(self.fileDir, 'camera_info.npz'), P=self.P, D=self.D)
        self.height, self.width = get_img_size(self.camera_info)

        self.bridge = CvBridge()
        self.gnss = GnssState(self)
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_IM_TOPIC, qos_profile=qos_profile_sensor_data)
        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            k_time_sync_queue_size,
            k_time_sync_slop
            )
        self.im_depth_sub.registerCallback(self.im_depth_callback)
        self.iter = 0
        self.diff = 0
        self.gps_last = np.zeros(9)

    def im_depth_callback(self, img_msg: Image, depth_msg: Image):
        cur_gps = self.gnss.snapshot()
        if cur_gps is None:
            return
        self.diff += abs(cur_gps.stamp - stamp_to_sec(img_msg.header.stamp))
        gps_out = np.array([
            cur_gps.lat,
            cur_gps.lon,
            cur_gps.height,
            cur_gps.roll,
            cur_gps.pitch,
            float(origin.compass_heading_deg(cur_gps.heading)), # azimuth, compass deg
            cur_gps.stamp,
            cur_gps.vn,
            cur_gps.ve
        ])
        if self.gps_last[0] - gps_out[0] == 0 and self.gps_last[1]-gps_out[1] == 0:
            print(f"Same coord at {self.iter}")
        self.gps_last = gps_out
        curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth_map = depth_msg_to_meters(depth_msg)

        np.save(os.path.join(self.depth_path, f'depth_im{self.iter}.npy'), depth_map)
        cv2.imwrite(os.path.join(self.img_path, f'im_{self.iter}.png'), curImage)
        np.save(os.path.join(self.raw_gps_path, f'raw_gps_{self.iter}.npy'), gps_out)
        self.iter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SaveDataNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.iter:
            print(f"Done, iters:{node.iter}, avg |gps - image| stamp: {node.diff/node.iter:.4f} s")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
