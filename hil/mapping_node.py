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
import os
import open3d as o3d
import rospkg

rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

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

        # Pointcloud Insertion
        self.model_path = os.path.join(rospack.get_path('hil'), 'data', 'models', "dummy.obj")
        tri_mesh = o3d.io.read_triangle_mesh(self.model_path)
        nPoints = 5000
        pc = tri_mesh.sample_points_poisson_disk(nPoints)
        model_pc = np.asarray(pc.points)
        model_mat = np.array([
            [0,0.02,0,5.77],
            [0.02,0,0,-21],
            [0,0,0.02,-1.7],
            [0,0,0,1]
        ])
        scale_mat = np.diag([1,1,1,1])
        model_pc = ut.applyTransform(model_pc, model_mat @ scale_mat)
        model_pts = np.zeros(model_pc.shape[0], 
                        dtype=[('x', float), ('y', float), ('z', float),
                                ('b', int), ('g', int), ('r', int)])
        model_pts['x'] = model_pc[:, 0]
        model_pts['y'] = model_pc[:, 1]
        model_pts['z'] = model_pc[:, 2]

        model_pts['r'] = np.full_like(model_pc[:,0], 255)
        model_pts['g'] = np.full_like(model_pc[:,0], 0)
        model_pts['b'] = np.full_like(model_pc[:,0], 0)
        model_pts = ros_numpy.point_cloud2.merge_rgb_fields(model_pts)

        self.rgb_pointcloud = np.hstack((self.rgb_pointcloud, model_pts))

        self.tf_listener = tf.TransformListener()

        # Image Insertion
        # model_mat2 = np.array([
        #     [0,0.02,0,9],
        #     [0.02,0,0,0],
        #     [0,0,0.02,-1.7],
        #     [0,0,0,1]
        # ]).T
        # self.iEngine = insertionEngine()
        # Subscribers
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

        # T_cam = T_gps
        # T_cam[2, 3] += 1.7
        # cur_cam_world_pose = T_cam[:3,3]
        # in_front_of_cam = cur_cam_world_pose
        # in_front_of_cam[0] += 1
        # in_front_of_cam = np.append(in_front_of_cam, 1)
        # look_at_world_pose = T_cam @ in_front_of_cam
        # res_img = self.iEngine.render(self.curImage, cur_cam_world_pose[:3], look_at_world_pose[:3])

        # aug_img_msg = self.bridge.cv2_to_imgmsg(res_img)
        # self.aug_im_publisher.publish(aug_img_msg)


    # def insertion_im_callback(self, img_msg: Image, gps_msg: Inspva):
    #     print("Received")
    #     T_gps = np.eye(4)
    #     if not self.first_msg:
    #         self.start_gps = gps_msg
    #     u1 = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
    #     u2 = utm.from_latlon(self.start_gps.latitude, self.start_gps.longitude)
    #     dy = u1[0]-u2[0]
    #     dx = u1[1]-u2[1]
    #     dz = gps_msg.height-self.start_gps.height
    #     delTheta = -np.deg2rad(gps_msg.azimuth-self.start_gps.azimuth)
    #     T_gps[:3,:3] = np.array([
    #         [np.cos(delTheta), -np.sin(delTheta), 0],
    #         [np.sin(delTheta), np.cos(delTheta), 0],
    #         [0, 0, 1]
    #     ])
    #     T_gps[:3,3] = [-dx,dy,dz]
    #     data_path = os.path.join(rospack.get_path('hil'), 'experimentation', 'outdoor_bag_data', 'img')
    #     img = cv2.imread(os.path.join(data_path, 'im_100.jpg'))
    #     res_img = self.iEngine.render(img, (-5,0,0), (0,0,0))
    #     # rospy.sleep(5)
    #     cv2.imshow('res', res_img)
    #     cv2.waitKey(0)
    #     self.first_msg = True
    def run(self):
        rospy.spin()

def main():
    node = mapNode()
    node.run()

if __name__ == '__main__':
    main()