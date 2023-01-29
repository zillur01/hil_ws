#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseArray, Pose, PointStamped
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
from hil.tf_funcs import get_gps_tf
from hil.utils import tf_to_transform
from HIL_realtime_rendering.main import HIL_rendering, setup_window_config, cleanup_window_config, DEFAULT_ARGS
from HIL_realtime_rendering.constants import T_blender2opengl
import moderngl_window as mglw
import torch.nn.functional as F
import torch
from omegaconf import OmegaConf
DEBUG = True
FRAME_RATE = 10

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class insertionNode:
    """ Aggregates a global 3D colored pointcloud map
        as the car moves around the environment"""
    def __init__(self, omega_config):
        rospy.init_node('insertion', anonymous=True)
        self.config = omega_config
        self.window_size = omega_config.window_size

        self.bridge = CvBridge()
        self.cur_image = None
        self.depth_map = None
        self.cur_image_timestamp = None
        self.T_gps = None
        self.projected_pub = rospy.Publisher(OUTPUT_MAP_TOPIC, PointCloud2)
        self.latency_pub = rospy.Publisher(SIM_LATENCY, Float64, queue_size=10)
        self.sim_im_publisher = rospy.Publisher(SIM_IMAGE, Image, queue_size=10)
        self.sim_depth_publisher = rospy.Publisher(SIM_DEPTH, Image, queue_size=10)
        self.sim_obst_publisher = rospy.Publisher(SIM_OBSTACLE_POSES, PoseArray, queue_size=10)
        self.rgb_pointcloud = np.zeros(0, dtype=[('x', float), ('y', float), ('z', float),
                                    ('rgb', np.float32)])
        
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
        self.n_insertion = 0 # number of times img_insertion is called
        self.n_callback = 0 # numer of times im_depth_callback is called
        # Rendering setup

        self.window, self.config_obj, self.timer = setup_window_config(HIL_rendering, DEFAULT_ARGS, True, self.config)

    def im_depth_callback(self, img_msg: Image, depth_msg: Image, gps_msg: Inspva):
        """Sensor data(rgb, depth, pose) updated as synced messages are received"""
        # print("received data")
        cv2_rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        # cv2.imshow("res2", cv2_rgb_image)
        # cv2.waitKey(1)
        self.cur_image = np.array(pil_img.fromarray(cv2_rgb_image).resize(size=self.window_size, resample=pil_img.Resampling.BILINEAR))
        self.cur_image_timestamp = img_msg.header.stamp
        mode_depth='F'
        orig_depth_map = np.asarray(pil_img.frombytes(mode_depth, (depth_msg.width, depth_msg.height), depth_msg.data))
        self.depth_map = F.interpolate(torch.from_numpy(np.nan_to_num(orig_depth_map, nan=np.inf, posinf=np.inf, neginf=np.inf)[None, None]), scale_factor=0.5).numpy()[0][0]
        (t,Q) = get_gps_tf(gps_msg)
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
        input_img = self.cur_image
        input_img_stamp = self.cur_image_timestamp
        sensor_data = {
            "rgb": input_img[..., :3].astype(np.float32) / 255,
            "pose": world2cam_gl @ T_blender2opengl,
            "depth": self.depth_map
        }

        # Rendering iteration
        if self.config_obj.clear_color is not None:
            self.window.clear(*self.config_obj.clear_color)

        # Always bind the window framebuffer before calling render
        self.window.use()
        res_img, res_depth, primary_obstacle_poses = self.window.config.render_single_frame(
            sensor_data
        )

        res_img = cv2.cvtColor((res_img[..., :3] * 255.).astype(np.uint8), cv2.COLOR_RGB2BGR)
        res_depth = np.array(res_depth)
        if DEBUG:
            cv2.imshow("Original", cv2.cvtColor(self.cur_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
            cv2.imshow("Inserted", res_img)
            cv2.waitKey(1)
        # End render iter

        # publish message
        sim_img_msg = self.bridge.cv2_to_imgmsg(res_img, encoding="bgr8")
        sim_img_msg.header.frame_id = "zed2_left_camera_optical_frame"
        sim_depth_msg = self.bridge.cv2_to_imgmsg(res_depth, encoding="32FC1")
        sim_depth_msg.header.frame_id = "zed2_left_camera_optical_frame"

        stamp = rospy.Time.now()
        sim_img_msg.header.stamp = stamp
        sim_depth_msg.header.stamp = stamp
        self.sim_im_publisher.publish(sim_img_msg)
        self.sim_depth_publisher.publish(sim_depth_msg)
        # obstacle point in local coords
        pose_array = []
        for pose in primary_obstacle_poses:
            T = np.eye(4)
            obs_x, obs_z, obs_y  = pose
            obs_x *= -1
            T[:3,3] = obs_x, obs_y, obs_z
            pose_array.append(T)
        self.publish_pose_array(pose_array)
        # sim_obst_point = PointStamped()
        
        # sim_obst_point.header.stamp = rospy.Time.now()

        # sim_obst_point.point.x = obs_x
        # sim_obst_point.point.y = obs_y
        # sim_obst_point.point.z = obs_z

        # self.sim_obst_publisher.publish(sim_obst_point)
        latency = stamp.to_sec() - input_img_stamp.to_sec()
        print("Latency:", latency)
        self.latency_pub.publish(data=latency)
        self.n_insertion += 1
        current_time, delta = self.timer.next_frame()
        print('frame {}, FPS = {}'.format(self.n_insertion, self.n_insertion / current_time))

        # rospy.loginfo(f"avg time at {self.n}: {self.n/self.total_time} fps")

        if not self.window.is_closing:
            self.window.swap_buffers()
    def publish_pose_array(self, arr):
        ros_pose = []
        for T in arr:
            (t,Q) = ut.transform_to_tf(T)
            p = Pose()
            p.position.x = t[0]
            p.position.y = t[1]
            p.position.z = t[2]
            p.orientation.x = Q[0]
            p.orientation.y = Q[1]
            p.orientation.z = Q[2]
            p.orientation.w = Q[3]
            ros_pose.append(p)
        out_msg = PoseArray()
        out_msg.header.stamp = rospy.Time.now()
        out_msg.poses = ros_pose
        self.sim_obst_publisher.publish(out_msg)
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
    def on_shutdown(self):
        cleanup_window_config(self.window, self.timer)

    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        print("Starting main insertion loop")
        r = rospy.Rate(FRAME_RATE) # 10hz 
        self.timer.start()

        while not rospy.is_shutdown():
            if  self.cur_image is not None and \
                self.depth_map is not None and \
                self.T_gps is not None:
                print("inserting")
                self.img_insertion()
            r.sleep()

def main():
    node_name = "insertion"
    param_name = "config_path"
    config_path = rospy.get_param(f'{node_name}/{param_name}')
    custom_config = OmegaConf.load(config_path)
    node = insertionNode(custom_config)
    node.run()

if __name__ == '__main__':
    main()