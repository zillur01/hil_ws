#!/usr/bin/env python3
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
from hil.utils import *
from hil.tf_funcs import *
from hil.globals import *
import rospkg
import os

rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class saveDataNode:
    def __init__(self, extracted_bag_name):
        rospy.init_node('saveData', anonymous=True)
        self.fileDir = os.path.join(rospack.get_path('hil'), 'data', 'extracted_bags', extracted_bag_name)
        self.img_path = os.path.join(self.fileDir, 'img')
        self.pc_path = os.path.join(self.fileDir, 'pc')
        self.pose_path = os.path.join(self.fileDir, 'pose')
        if not os.path.exists(self.fileDir):
            os.mkdir(self.fileDir)
        if not os.path.exists(self.img_path):
            os.mkdir(self.img_path)
        if not os.path.exists(self.pc_path):
            os.mkdir(self.pc_path)
        if not os.path.exists(self.pose_path):
            os.mkdir(self.pose_path)
        self.camera_info = get_camera_info(CAMERA_INFO_TOPIC)
        self.P = get_projection_matrix(self.camera_info)
        self.D = get_dist_matrix(self.camera_info)
        np.savez(os.path.join(self.fileDir, 'camera_info.npz'), P=self.P, D=self.D)
        self.height, self.width = get_img_size(self.camera_info)

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
        self.iter = 0

        self.tf_listener = tf.TransformListener()

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image ):
        T_gps = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_listener, convertToTransform=True)

        self.curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        pts_depth = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)

        cv2.imwrite(os.path.join(self.img_path, f'im_{self.iter}.png'), self.curImage)
        np.save(os.path.join(self.pc_path, f'pc_{self.iter}.npy'), pts_depth)
        np.save(os.path.join(self.pose_path, f'gps_{self.iter}.npy'), T_gps)
        self.iter += 1
    def run(self):
        rospy.spin()

def main():
    node_name = "save_data_node"
    param_name = "rosbag_filename"
    bag_filename = rospy.get_param(f'{node_name}/{param_name}')
    bag_filename = bag_filename.split('.')[0]
    node = saveDataNode(bag_filename)
    node.run()
    print("DOne")

if __name__ == '__main__':
    main()
