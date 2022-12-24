#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
import hil.utils as ut
from hil.tf_funcs import *
from hil.globals import *
import tf
import rospkg

rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.01 # allowed difference in seconds between messages to sync

class mapNode:
    """ Aggregates a global 3D colored pointcloud map
        as the car moves around the environment"""
    def __init__(self):
        rospy.init_node('mapping', anonymous=True)
        self.camera_info = ut.get_camera_info(CAMERA_INFO_TOPIC)
        self.P = ut.get_projection_matrix(self.camera_info)
        self.height, self.width = ut.get_img_size(self.camera_info)

        self.bridge = CvBridge()
        self.cur_image = None
        
        self.projected_pub = rospy.Publisher(OUTPUT_MAP_TOPIC, PointCloud2)
        self.aug_im_publisher = rospy.Publisher('augmented_image', Image)

        self.rgb_pointcloud = np.zeros(0, dtype=[('x', float), ('y', float), ('z', float),
                                    ('rgb', np.float32)])

        self.tf_listener = tf.TransformListener()

        self.pc_sub = message_filters.Subscriber(LIDAR_PC_TOPIC, PointCloud2)
        self.image_sub = message_filters.Subscriber(RGB_TOPIC, Image)
        self.all_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.all_sub.registerCallback(self.pc_im_callback)

    def pc_im_callback(self, pc_msg: PointCloud2, img_msg: Image):
        # Grab car's relative transform from our World frame origin
        T_gps = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_listener, convertToTransform=True)
        # Get current rgb pointcloud(in world coord) from image + lidar + transform inputs
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        orig_pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)
        pts_depth = ros_numpy.point_cloud2.get_xyz_points(orig_pc, remove_nans=True)

        imgfov_pts = ut.color_pointcloud(
                    pts_depth, self.cur_image, 
                    T_lidar_rgb, self.P,
                    transform_pc = T_gps
        )

        # Stack current 3d points view onto current set of points and publish
        self.rgb_pointcloud = np.hstack((self.rgb_pointcloud, imgfov_pts))
        pc2_res = ros_numpy.point_cloud2.array_to_pointcloud2(self.rgb_pointcloud, stamp=rospy.Time.now(), frame_id=WORLD_FRAME)
        self.projected_pub.publish(pc2_res)

    def run(self):
        rospy.spin()

def main():
    node = mapNode()
    node.run()

if __name__ == '__main__':
    main()