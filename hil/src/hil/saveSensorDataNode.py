import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from hil.utils import *
from hil.tf_funcs import *
import rospkg
import os
from novatel_gps_msgs.msg import Inspva
import utm

rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.01 # allowed difference in seconds between messages to sync

class saveDataNode:
    def __init__(self, CAMERA_INFO_TOPIC, LIDAR_PC_TOPIC, RGB_TOPIC, GPS_TOPIC):
        rospy.init_node('saveData', anonymous=True)
        self.fileDir = os.path.join(rospack.get_path('hil'), 'experimentation', 'outdoor_bag')
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
        self.gps_sub = message_filters.Subscriber(GPS_TOPIC, Inspva)
        self.pc_im_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub, self.gps_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.pc_im_sub.registerCallback(self.pc_im_callback)
        self.iter = 0
        self.first_msg = False

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image, gps_msg: Inspva ):
        T_gps = np.eye(4)
        if not self.first_msg:
            self.start_gps = gps_msg
            self.first_msg = True

        u1 = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
        u2 = utm.from_latlon(self.start_gps.latitude, self.start_gps.longitude)
        dy = u1[0]-u2[0]
        dx = u1[1]-u2[1]
        dz = gps_msg.height-self.start_gps.height
        delTheta = -np.deg2rad(gps_msg.azimuth-self.start_gps.azimuth)
        T_gps[:3,:3] = np.array([
            [np.cos(delTheta), -np.sin(delTheta), 0],
            [np.sin(delTheta), np.cos(delTheta), 0],
            [0, 0, 1]
        ])
        T_gps[:3,3] = [-dx,dy,dz]
        self.curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        pts_depth = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)

        cv2.imwrite(os.path.join(self.img_path, f'im_{self.iter}.png'), self.curImage)
        np.save(os.path.join(self.pc_path, f'pc_{self.iter}.npy'), pts_depth)
        np.save(os.path.join(self.pose_path, f'gps_{self.iter}.npy'), T_gps)
        self.iter += 1
    def run(self):
        rospy.spin()
