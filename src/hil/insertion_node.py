#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as pil_img
import hil.utils as ut
from hil.tf_funcs import *
from hil.globals import *
import tf, message_filters
import os, time, sys
import rospkg
rospack = rospkg.RosPack()
rendering_lib_path = os.path.join(rospack.get_path('hil'), 'src', 'HIL_realtime_rendering')
sys.path.append(rendering_lib_path)
os.chdir(rendering_lib_path)
from hil.insertionEngine import insertionEngine
from hil.tf_funcs import get_gps_tf
from hil.utils import tf_to_transform
from HIL_realtime_rendering.main import HIL_rendering, setup_window_config, cleanup_window_config, DEFAULT_ARGS
from HIL_realtime_rendering.constants import T_blender2opengl
import moderngl_window as mglw


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
        self.T_gps = np.eye(4)
        self.projected_pub = rospy.Publisher(OUTPUT_MAP_TOPIC, PointCloud2)
        self.aug_im_publisher = rospy.Publisher('augmented_image', Image, queue_size=10)

        self.rgb_pointcloud = np.zeros(0, dtype=[('x', float), ('y', float), ('z', float),
                                    ('rgb', np.float32)])


        self.tf_listener = tf.TransformListener()

        
        # Subscribers
        # self.image_sub = rospy.Subscriber(RGB_TOPIC, Image, callback=self.im_callback)
        self.image_sub = message_filters.Subscriber(RGB_TOPIC,Image)
        self.depth_sub = message_filters.Subscriber(DEPTH_IM_TOPIC, Image)
        self.inspva_sub = message_filters.Subscriber(GPS_TOPIC, Inspva)

        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.inspva_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.im_depth_sub.registerCallback(self.im_depth_callback)
        self.n = 0
        self.total_time = 0

        # Rendering setup

        self.window, self.config, self.timer = setup_window_config(HIL_rendering, DEFAULT_ARGS, True)


    def im_depth_callback(self, img_msg: Image, depth_msg: Image, gps_msg: Inspva):
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        self.cur_image_timestamp = img_msg.header.stamp
        mode_depth='F'
        self.depth_map = np.asarray(pil_img.frombytes(mode_depth, (depth_msg.width, depth_msg.height), depth_msg.data))
        (t,Q) = get_gps_tf(gps_msg, gps_msg)
        self.T_gps = tf_to_transform(t, Q)

    def img_insertion(self):
        # cam2world = get_tf(GPS_FRAME, WORLD_FRAME, self.tf_listener, common_t = self.cur_image_timestamp, convertToTransform=True)
        cam2world = self.T_gps
        world2cam_cv = np.linalg.inv(cam2world)

        cv2gl = np.array([
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])
        world2cam_gl = cv2gl @ world2cam_cv
        sensor_data = {
            "rgb": self.cur_image[..., :3].astype(np.float32) / 255,
            "pose": world2cam_gl @ T_blender2opengl,
            "depth": self.depth_map
        }

        start_time = time.time()

        # Rendering iteration
        if self.config.clear_color is not None:
            self.window.clear(*self.config.clear_color)

        # Always bind the window framebuffer before calling render
        self.window.use()
        res_img = self.window.config.render_single_frame(
            sensor_data
        )
        
        res_img = cv2.cvtColor((res_img[..., :3] * 255.).astype(np.uint8), cv2.COLOR_RGB2BGR)
        # End render iter

        self.total_time += time.time()-start_time
        # cv2.imshow("res", res_img)
        # cv2.waitKey(1)
        # publish message
        aug_img_msg = self.bridge.cv2_to_imgmsg(res_img)
        aug_img_msg.header.stamp = rospy.Time.now()
        aug_img_msg.header.frame_id = "zed2_left_camera_optical_frame"
        self.aug_im_publisher.publish(aug_img_msg)

        self.n += 1
        rospy.loginfo(f"avg time at {self.n}: {self.n/self.total_time} fps")

        if not self.window.is_closing:
            self.window.swap_buffers()

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
        r = rospy.Rate(5) # 10hz 
        while not rospy.is_shutdown():
            if self.cur_image is not None:
                print("Calling")
                self.img_insertion()
            r.sleep()

def main():
    node = insertionNode()
    node.run()

if __name__ == '__main__':
    main()