import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from hil.utils import *
from hil.tf_funcs import *

k_time_sync_queue_size = 10
k_time_sync_slop = 1 # allowed difference in seconds between messages to sync

class lidar2camNode:
    def __init__(self, CAMERA_INFO_TOPIC, LIDAR_PC_TOPIC, RGB_TOPIC, T_depth_rgb):
        rospy.init_node('lidar2cam', anonymous=True)
        self.camera_info = get_camera_info(CAMERA_INFO_TOPIC)
        self.P = get_projection_matrix(self.camera_info)
        self.height, self.width = get_img_size(self.camera_info)
        self.T_depth_rgb = T_depth_rgb
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

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image, ):
        self.curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        pts_depth = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
        # print(pts_depth[:20])
        print(np.min(pts_depth), np.max(pts_depth), pts_depth[1500:1510])
        pts_2d, depths = project_to_image(pts_depth, self.P, self.T_depth_rgb)
        print(np.min(pts_2d), np.max(pts_2d), pts_2d[:, 1500:1510])
        print(pts_2d.shape)

        # Filter lidar points to be within image FOV
        inds = np.where((pts_2d[0, :] < self.width) & (pts_2d[0, :] >= 0) &
                        (pts_2d[1, :] < self.height) & (pts_2d[1, :] >= 0) &
                        (depths > 1) & (depths < 3)
                        )[0]
        print(inds.shape)
        # Filter out pixels points
        imgfov_pc_pixel = pts_2d[:, inds]
        imgfov_pc_depth = depths[inds]
        max_depth = np.max(imgfov_pc_depth)
        print(imgfov_pc_pixel.shape)

        # Create color map and paint circles with color corresponding to depth
        cmap = plt.cm.get_cmap('hsv', 256)
        cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
        print("Depth Mean:", np.mean(imgfov_pc_depth))
        depth_map = np.clip(255* (imgfov_pc_depth/max_depth), 0, 255).astype(int)
        print(cmap[depth_map].shape)
        # Paint lidar points onto the image 
        depth_image = self.curImage.copy()
        print(depth_image.shape)
        x_pixels = imgfov_pc_pixel[0].astype(int)
        y_pixels = imgfov_pc_pixel[1].astype(int)
        depth_image[y_pixels, x_pixels] = cmap[depth_map]
        
        cv2.imshow("Depth Image window", depth_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
