#!/usr/bin/env python3
"""Aggregates a global 3D colored pointcloud map (OUTPUT_MAP_TOPIC, WORLD_FRAME) from lidar + RGB
as the car moves around. Needs the WORLD_FRAME -> GPS_FRAME tf from gps_tf_pub_node."""
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
import tf2_ros

import hil_common.utils as ut
from hil_common.globals import (CAMERA_INFO_TOPIC, GPS_FRAME, LIDAR_PC_TOPIC, OUTPUT_MAP_TOPIC, RGB_TOPIC,
                                T_lidar_rgb, WORLD_FRAME)
from hil_common.tf_funcs import get_tf

k_time_sync_queue_size = 10
k_time_sync_slop = 0.01 # allowed difference in seconds between messages to sync


class MapNode(Node):
    """ Aggregates a global 3D colored pointcloud map
        as the car moves around the environment"""
    def __init__(self):
        super().__init__('mapping')
        self.camera_info = ut.get_camera_info(self, CAMERA_INFO_TOPIC)
        if self.camera_info is None:
            raise RuntimeError(f"No camera info received on {CAMERA_INFO_TOPIC}")
        self.P = ut.get_projection_matrix(self.camera_info)
        self.height, self.width = ut.get_img_size(self.camera_info)

        self.bridge = CvBridge()
        self.cur_image = None

        self.projected_pub = self.create_publisher(PointCloud2, OUTPUT_MAP_TOPIC, 10)

        self.rgb_pointcloud = np.zeros(0, dtype=[('x', float), ('y', float), ('z', float),
                                    ('rgb', np.float32)])

        # spin_thread: the buffer keeps filling while the image callback waits for a transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pc_sub = message_filters.Subscriber(self, PointCloud2, LIDAR_PC_TOPIC, qos_profile=qos_profile_sensor_data)
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.all_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub],
            k_time_sync_queue_size,
            k_time_sync_slop
            )
        self.all_sub.registerCallback(self.pc_im_callback)

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image):
        # Grab car's relative transform from our World frame origin
        try:
            T_gps = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_buffer, convertToTransform=True)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"No {WORLD_FRAME} -> {GPS_FRAME} transform yet: {e}",
                                   throttle_duration_sec=2.0)
            return
        T_gps = T_gps @ np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0,0,1,0],
            [0,0,0,1]
        ])
        # Get current rgb pointcloud(in world coord) from image + lidar + transform inputs
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        pts_depth = ut.pointcloud2_to_xyz(pc_msg)

        imgfov_pts = ut.color_pointcloud(
                    pts_depth, self.cur_image,
                    T_lidar_rgb, self.P,
                    transform_pc = T_gps
        )

        # Stack current 3d points view onto current set of points and publish
        self.rgb_pointcloud = np.hstack((self.rgb_pointcloud, imgfov_pts))
        pc2_res = ut.xyzrgb_to_pointcloud2(self.rgb_pointcloud, self.get_clock().now().to_msg(), WORLD_FRAME)
        self.projected_pub.publish(pc2_res)


def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
