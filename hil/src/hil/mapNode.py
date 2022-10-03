from concurrent.futures.process import _threads_wakeups
from operator import truediv
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from hil.utils import *
from hil.tf_funcs import *
from novatel_gps_msgs.msg import Inspva
import math
import utm

k_time_sync_queue_size = 10
k_time_sync_slop = 1 # allowed difference in seconds between messages to sync

class mapNode:
    def __init__(self, CAMERA_INFO_TOPIC, LIDAR_PC_TOPIC, RGB_TOPIC, GPS_TOPIC, PROJECTED_TOPIC, T_depth_rgb):
        rospy.init_node('mapping', anonymous=True)
        self.camera_info = get_camera_info(CAMERA_INFO_TOPIC)
        self.P = get_projection_matrix(self.camera_info)
        self.height, self.width = get_img_size(self.camera_info)
        self.T_depth_rgb = T_depth_rgb

        self.bridge = CvBridge()
        self.curImage = None
        
        self.pc_sub = message_filters.Subscriber(LIDAR_PC_TOPIC, PointCloud2)
        self.image_sub = message_filters.Subscriber(RGB_TOPIC, Image)
        self.gps_sub = message_filters.Subscriber(GPS_TOPIC, Inspva)
        self.all_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub, self.gps_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.all_sub.registerCallback(self.pc_im_callback)

        self.projected_pub = rospy.Publisher(PROJECTED_TOPIC, PointCloud2)
        self.rgb_pointcloud = np.zeros(0, dtype=[('x', float), ('y', float), ('z', float),
                                    ('rgb', np.float32)])

        self.first_msg = False
        self.start_gps = None

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image, gps_msg: Inspva):
        T_gps = np.eye(4)
        if not self.first_msg:
            self.start_gps = gps_msg
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
        T_gps[:3,3] = [-dx,-dy,-dz]
        self.curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        orig_pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)
        pts_depth = ros_numpy.point_cloud2.get_xyz_points(orig_pc, remove_nans=True)
        a = pts_depth.dtype.names
        print("Names", pts_depth.dtype.names)
        # print(pts_depth[:20])
        print(np.min(pts_depth), np.max(pts_depth), pts_depth[1500:1510])
        pts_2d, depths = project_to_image(pts_depth, self.P, self.T_depth_rgb)
        print(np.min(pts_2d), np.max(pts_2d), pts_2d[:, 1500:1510])
        print(pts_2d.shape)

        # Filter lidar points to be within image FOV
        inds = np.where((pts_2d[0, :] < self.width) & (pts_2d[0, :] >= 0) &
                        (pts_2d[1, :] < self.height) & (pts_2d[1, :] >= 0) &
                        (depths > 0)
                        )[0]
        print(inds.shape)
        # Filter out pixels points
        imgfov_pc_pixel = pts_2d[:, inds]
        imgfov_pc_depth = depths[inds]

        max_depth = np.max(imgfov_pc_depth)
        filtered_pts_depth = applyTransform(pts_depth[inds], T_gps)

        print(imgfov_pc_pixel.shape)
        imgfov_pts = np.zeros(filtered_pts_depth.shape[0], 
                        dtype=[('x', float), ('y', float), ('z', float),
                                ('b', int), ('g', int), ('r', int)])
        imgfov_pts['x'] = filtered_pts_depth[:, 0]
        imgfov_pts['y'] = filtered_pts_depth[:, 1]
        imgfov_pts['z'] = filtered_pts_depth[:, 2]

        x_pixels = imgfov_pc_pixel[0].astype(int)
        y_pixels = imgfov_pc_pixel[1].astype(int)
        rgb_pts = self.curImage[y_pixels, x_pixels]
        imgfov_pts['r'] = rgb_pts[:, 0]
        imgfov_pts['g'] = rgb_pts[:, 1]
        imgfov_pts['b'] = rgb_pts[:, 2]
        imgfov_pts = ros_numpy.point_cloud2.merge_rgb_fields(imgfov_pts)
        self.rgb_pointcloud = np.hstack((self.rgb_pointcloud, imgfov_pts))
        pc2_res = ros_numpy.point_cloud2.array_to_pointcloud2(self.rgb_pointcloud, stamp=rospy.Time.now(), frame_id='lidar2')
        self.projected_pub.publish(pc2_res)

        self.first_msg = True

    def run(self):
        rospy.spin()
