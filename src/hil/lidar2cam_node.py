#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy
from cv_bridge import CvBridge
import hil.utils as ut
from hil.globals import *

k_time_sync_queue_size = 10
k_time_sync_slop = 0.1 # allowed difference in seconds between messages to sync

class lidar2camNode:
    """ Visualizes lidar 3D points projected onto an image 
        and publishes a colored lidar pointcloud to OUTPUT_MAP_TOPIC"""
    def __init__(self):
        rospy.init_node('lidar2cam')
        self.camera_info = ut.get_camera_info(CAMERA_INFO_TOPIC)
        self.P = ut.get_projection_matrix(self.camera_info)
        print(self.P)
        self.height, self.width = ut.get_img_size(self.camera_info)
        # listener = tf.TransformListener()
        # self.T_depth_rgb = get_tf('velodyne', 'front_single_camera_link', listener, convertToTransform=True)

        self.bridge = CvBridge()
        self.curImage = None
        
        self.pc_sub = message_filters.Subscriber(LIDAR_PC_TOPIC, PointCloud2)
        self.image_sub = message_filters.Subscriber(RGB_TOPIC,Image)
        self.pc_im_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.pc_im_sub.registerCallback(self.pc_im_callback)

        self.projected_pub = rospy.Publisher(OUTPUT_MAP_TOPIC, PointCloud2)

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image, ):
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        orig_pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)
        pts_depth = ros_numpy.point_cloud2.get_xyz_points(orig_pc, remove_nans=True)
        imgfov_pts = ut.color_pointcloud(
                    pts_depth, self.cur_image, 
                    T_lidar_rgb, self.P,
                    visualize_projection=True
        )

        pc2_res = ros_numpy.point_cloud2.array_to_pointcloud2(imgfov_pts, stamp=rospy.Time.now(), frame_id=WORLD_FRAME)
        self.projected_pub.publish(pc2_res)

    def run(self):
        rospy.spin()

def main():
    node = lidar2camNode()
    node.run()

if __name__ == '__main__':
    main()