#!/usr/bin/env python3
"""Visualizes lidar 3D points projected onto the image (/projected_vis) and publishes the
lidar points inside the camera view, colored from the image, on OUTPUT_MAP_TOPIC."""
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

import hil_common.utils as ut
from hil_common.globals import CAMERA_INFO_TOPIC, LIDAR_PC_TOPIC, OUTPUT_MAP_TOPIC, RGB_TOPIC, T_lidar_rgb, WORLD_FRAME

k_time_sync_queue_size = 10
k_time_sync_slop = 0.1 # allowed difference in seconds between messages to sync


class Lidar2CamNode(Node):
    """ Visualizes lidar 3D points projected onto an image
        and publishes a colored lidar pointcloud to OUTPUT_MAP_TOPIC"""
    def __init__(self):
        super().__init__('lidar2cam')
        self.camera_info = ut.get_camera_info(self, CAMERA_INFO_TOPIC)
        if self.camera_info is None:
            raise RuntimeError(f"No camera info received on {CAMERA_INFO_TOPIC}")
        self.P = ut.get_projection_matrix(self.camera_info)
        self.height, self.width = ut.get_img_size(self.camera_info)

        self.bridge = CvBridge()
        self.cur_image = None

        self.pc_sub = message_filters.Subscriber(self, PointCloud2, LIDAR_PC_TOPIC, qos_profile=qos_profile_sensor_data)
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.pc_im_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub],
            k_time_sync_queue_size,
            k_time_sync_slop
            )
        self.pc_im_sub.registerCallback(self.pc_im_callback)

        self.projected_pub = self.create_publisher(PointCloud2, OUTPUT_MAP_TOPIC, 10)
        self.im_vis_pub = self.create_publisher(Image, "/projected_vis", 10)

        self.visualized_img = None
        self.timer = self.create_timer(0.1, self.publish_vis)  # 10 Hz

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image):
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        pts_depth = ut.pointcloud2_to_xyz(pc_msg)
        imgfov_pts, self.visualized_img = ut.color_pointcloud(
                    pts_depth, self.cur_image,
                    T_lidar_rgb, self.P,
                    visualize_projection=True
        )

        pc2_res = ut.xyzrgb_to_pointcloud2(imgfov_pts, self.get_clock().now().to_msg(), WORLD_FRAME)
        self.projected_pub.publish(pc2_res)

    def publish_vis(self):
        if self.visualized_img is not None:
            vis_img_msg = self.bridge.cv2_to_imgmsg(self.visualized_img.astype('uint8'), encoding='bgr8')
            self.im_vis_pub.publish(vis_img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar2CamNode()
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
