#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import hil.utils as ut
from hil.tf_funcs import *
from hil.globals import *
import tf
import os, time
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
        self.cur_image_timestamp = None
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
        scale = 0.01
        # shift = [2, 3, -1.7]
        shift = [-25, -2, -1.7]
        # shift = [15, -2, -1.7]
        self.model_mat= np.array([
            [  scale,   0.0,   0.0,    shift[0]],
            [  0.0,     scale, 0.0,    shift[1]],
            [  0.0,     0.0,   scale,  shift[2]],
            [  0.0,     0.0,   0.0,    1.0]
        ]).T

        permute = np.array([
            [0, 1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.model_mat = permute @ self.model_mat  
        # Subscribers
        self.image_sub = rospy.Subscriber(RGB_TOPIC, Image, callback=self.im_callback)
        self.n = 0
        self.total_time = 0
    def im_callback(self, img_msg: Image):
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.cur_image_timestamp = img_msg.header.stamp
    def img_insertion(self):
        cam2world = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_listener, common_t = self.cur_image_timestamp, convertToTransform=True)
        world2cam_cv = np.linalg.inv(cam2world)

        cv2gl = np.array([
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])
        world2cam_gl = cv2gl @ world2cam_cv
        world2cam_gl_left_hand = world2cam_gl.T
        start_time = time.time()

        res_img = self.iEngine.render(
            self.cur_image, 
            self.iEngine.perspective, 
            world2cam_gl_left_hand, 
            self.model_mat)
        self.total_time += time.time()-start_time

        aug_img_msg = self.bridge.cv2_to_imgmsg(res_img)
        aug_img_msg.header.stamp = rospy.Time.now()
        aug_img_msg.header.frame_id = "zed2_left_camera_optical_frame"
        self.aug_im_publisher.publish(aug_img_msg)
        self.n += 1
        rospy.loginfo(f"avg time at {self.n}: {self.total_time/self.n} seconds")
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