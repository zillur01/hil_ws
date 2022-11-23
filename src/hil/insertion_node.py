#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import hil.utils as ut
from hil.tf_funcs import *
from hil.globals import *
import tf
import os
import rospkg
from hil.insertionEngine import insertionEngine

rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class insertionNode:
    """ Aggregates a global 3D colored pointcloud map
        as the car moves around the environment"""
    def __init__(self):
        rospy.init_node('insertion', anonymous=True)
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

        model_path = os.path.join(rospack.get_path('hil'), 'data', 'models')
        obj_path = os.path.join(model_path, 'dummy.obj')
        texture_path = os.path.join(model_path,'dummy.jpg')

        self.iEngine = insertionEngine(
            obj_path,
            texture_path,
            is_online=False
        )
        # This matrix defines the global pose where the obejct is placed in meters
        self.model_mat= np.array([
            [0,0.02,0,5.77],
            [0.02,0,0,-21],
            [0,0,0.02,-1.7],
            [0,0,0,1]
        ]).T    
        # Subscribers
        self.image_sub = rospy.Subscriber(RGB_TOPIC, Image, callback=self.im_callback)

    def im_callback(self, img_msg: Image):
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    def img_insertion(self):
        # data_path = os.path.join(rospack.get_path('hil'), 'data', 'extracted_bags', 'outdoor_bag', 'img')
        # img = cv2.imread(os.path.join(data_path, 'im_100.jpg'))
        # cv2.imshow('orig', img)
        # res_img = self.iEngine.render(img, (-5,0,0), (0,0,0), self.model_mat)
        # # rospy.sleep(5)
        # cv2.imshow('res', res_img)
        # cv2.waitKey(0)
        T_gps = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_listener, convertToTransform=True)
        T_cam = T_gps
        T_cam[2, 3] += 1.7  # Estimated z-val adjustment since camera is higher than gps unit 
        cur_cam_world_pose = T_cam[:3,3]
        in_front_of_cam = np.array([1,0,0,1])
        look_at_world_pose = T_cam @ in_front_of_cam

        res_img = self.iEngine.render(self.cur_image, cur_cam_world_pose, look_at_world_pose[:3], self.model_mat)

        aug_img_msg = self.bridge.cv2_to_imgmsg(res_img)
        self.aug_im_publisher.publish(aug_img_msg)

    # def pc_insertion(self):
    #     # Pointcloud Insertion
    #     self.model_path = os.path.join(rospack.get_path('hil'), 'data', 'models', "dummy.obj")
    #     tri_mesh = o3d.io.read_triangle_mesh(self.model_path)
    #     nPoints = 5000
    #     pc = tri_mesh.sample_points_poisson_disk(nPoints)
    #     model_pc = np.asarray(pc.points)
    #     model_mat = np.array([
    #         [0,0.02,0,5.77],
    #         [0.02,0,0,-21],
    #         [0,0,0.02,-1.7],
    #         [0,0,0,1]
    #     ])
    #     scale_mat = np.diag([1,1,1,1])
    #     model_pc = ut.applyTransform(model_pc, model_mat @ scale_mat)
    #     model_pts = np.zeros(model_pc.shape[0], 
    #                     dtype=[('x', float), ('y', float), ('z', float),
    #                             ('b', int), ('g', int), ('r', int)])
    #     model_pts['x'] = model_pc[:, 0]
    #     model_pts['y'] = model_pc[:, 1]
    #     model_pts['z'] = model_pc[:, 2]

    #     model_pts['r'] = np.full_like(model_pc[:,0], 255)
    #     model_pts['g'] = np.full_like(model_pc[:,0], 0)
    #     model_pts['b'] = np.full_like(model_pc[:,0], 0)
    #     model_pts = ros_numpy.point_cloud2.merge_rgb_fields(model_pts)

    #     self.rgb_pointcloud = np.hstack((self.rgb_pointcloud, model_pts))

    def run(self):
        r = rospy.Rate(10) # 10hz 
        while not rospy.is_shutdown():
            if self.cur_image is not None:
                self.img_insertion()
            r.sleep()

def main():
    node = insertionNode()
    node.run()

if __name__ == '__main__':
    main()