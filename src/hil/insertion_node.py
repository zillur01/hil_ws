#!/usr/bin/env python3
"""ROS 2 node: inserts virtual objects (HIL_realtime_rendering) into the camera image.

Inputs : RGB + depth (time-synced), /navsatfix + /insnavgeod (camera pose in the local ENU frame,
         see hil/origin.py)
Outputs: /insertion/rendered_image (rgb8), /insertion/rendered_depth (32FC1, depth of the virtual
         objects only, 100 m where there are none), /insertion/obstacle_poses (PoseArray in the
         local ENU frame), /insertion/latency

Run with `colcon build --symlink-install` (the renderer and its assets are loaded from the source tree).
"""
import os
import sys
import time

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import torch
import torch.nn.functional as F
from geometry_msgs.msg import Pose, PoseArray
from omegaconf import OmegaConf
from PIL import Image as pil_img
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from hil import origin
from hil.globals import (RGB_TOPIC, DEPTH_IM_TOPIC, SIM_IMAGE, SIM_DEPTH,
                         SIM_OBSTACLE_POSES, SIM_LATENCY)
from hil.gnss import GnssState
from hil.pose import gnss_to_cam2world
from hil.ros_image import imgmsg_to_rgb, depth_msg_to_meters, rgb_to_imgmsg, depth_to_imgmsg

FRAME_RATE = 50  # how often the render timer checks for a new frame

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03  # allowed difference in seconds between messages to sync

# Renderer world frame (x = -east, y = up, z = north; this follows from T_blender2opengl, which is
# applied to the camera pose in img_insertion) -> local ENU (x east, y north, z up)
P_GL_TO_ENU = np.array([[-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]], dtype=float)
# camera frame of the car (x right, y forward, z up) -> OpenGL camera frame (x right, y up, -z forward)
CV2GL = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, -1, 0, 0],
                  [0, 0, 0, 1]], dtype=float)


def default_renderer_dir():
    # realpath resolves the symlink created by `colcon build --symlink-install`
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(here, 'src', 'HIL_realtime_rendering')


def resolve_config_path(path, renderer_dir):
    """Accepts an absolute path, a path relative to the cwd, or one relative to the renderer's
    configs/ folder (e.g. 'gem/static_cone.yaml' or 'exp_1/scen_1/path_1.yaml')."""
    candidates = [path, os.path.join(renderer_dir, 'configs', path), os.path.join(renderer_dir, path)]
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError(f"Config '{path}' not found (tried: {', '.join(candidates)})")


def to_numpy_matrix(T):
    """Renderer transform (glm.mat4 or ndarray) -> row-major 4x4 numpy array."""
    M = np.array(T, dtype=float)
    try:
        import glm
        if isinstance(T, glm.mat4):
            M = M.T  # glm indexes [column][row]
    except ImportError:
        pass
    return M


class InsertionNode(Node):
    """Renders virtual objects into the latest image at the car's current pose."""

    def __init__(self):
        super().__init__('insertion')
        self.declare_parameter('config_path', '')
        self.declare_parameter('renderer_dir', '')
        self.declare_parameter('show_debug', False)
        self.declare_parameter('auto_origin_alt', False)
        renderer_dir = self.get_parameter('renderer_dir').value or default_renderer_dir()
        config_path = self.get_parameter('config_path').value
        self.show_debug = self.get_parameter('show_debug').value
        self.auto_origin_alt = self.get_parameter('auto_origin_alt').value
        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required, e.g. gem/static_cone.yaml")
        if not os.path.isfile(os.path.join(renderer_dir, 'main.py')):
            raise RuntimeError(
                f"Renderer not found in {renderer_dir}. Build with `colcon build --symlink-install` "
                "or pass the 'renderer_dir' parameter.")
        config_path = resolve_config_path(config_path, renderer_dir)
        self.config = OmegaConf.load(config_path)
        self.window_size = tuple(self.config.window_size)
        self.get_logger().info(f"Config: {config_path}, render size {self.window_size}")

        # The renderer loads assets/shaders relative to its own directory.
        sys.path.insert(0, renderer_dir)
        sys.path.insert(0, os.path.dirname(renderer_dir))
        os.chdir(renderer_dir)
        from HIL_realtime_rendering.main import HIL_rendering, setup_window_config, cleanup_window_config, DEFAULT_ARGS
        from HIL_realtime_rendering.constants import T_blender2opengl
        self._cleanup_window_config = cleanup_window_config
        self.T_blender2opengl = T_blender2opengl

        self.cur_image = None
        self.depth_map = None
        self.cur_image_timestamp = None
        self.T_gps = None
        self.call_insert = False
        self.origin_alt_set = False
        self.n_insertion = 0  # number of times img_insertion is called
        self.n_callback = 0   # number of times im_depth_callback is called

        self.latency_pub = self.create_publisher(Float64, SIM_LATENCY, 10)
        self.sim_im_publisher = self.create_publisher(Image, SIM_IMAGE, 10)
        self.sim_depth_publisher = self.create_publisher(Image, SIM_DEPTH, 10)
        self.sim_obst_publisher = self.create_publisher(PoseArray, SIM_OBSTACLE_POSES, 10)

        # Pose comes from the latest /navsatfix + /insnavgeod (speed is not needed for rendering)
        self.gnss = GnssState(self)
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_IM_TOPIC, qos_profile=qos_profile_sensor_data)
        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], k_time_sync_queue_size, k_time_sync_slop)
        self.im_depth_sub.registerCallback(self.im_depth_callback)

        # Rendering setup. The GL context belongs to this (main) thread, so use a single-threaded
        # executor and render from a timer instead of from the subscription callback.
        self.window, self.config_obj, self.timer = setup_window_config(HIL_rendering, DEFAULT_ARGS, True, self.config)
        self.timer.start()
        self.render_timer = self.create_timer(1.0 / FRAME_RATE, self.step)
        self.get_logger().info("Insertion ready, waiting for image, depth and GNSS")

    def im_depth_callback(self, img_msg: Image, depth_msg: Image):
        """Sensor data (rgb, depth, pose) updated as synced messages are received"""
        gnss = self.gnss.snapshot(need_speed=False)
        if gnss is None:
            self.get_logger().warn("No (or stale) /navsatfix + /insnavgeod yet", throttle_duration_sec=2.0)
            return
        if self.auto_origin_alt and not self.origin_alt_set:
            # first test aid: make the car's starting height the local z = 0, so the ground plane of
            # the config (z = -1.8 m) sits under the car
            origin.ORIGIN_ALT = float(gnss.height)
            self.origin_alt_set = True
            self.get_logger().info(f"ORIGIN_ALT set to {origin.ORIGIN_ALT:.3f} m (auto_origin_alt)")
        rgb = imgmsg_to_rgb(img_msg)
        self.cur_image = np.array(pil_img.fromarray(rgb).resize(
            size=self.window_size, resample=pil_img.Resampling.BILINEAR))
        depth = np.nan_to_num(depth_msg_to_meters(depth_msg), nan=np.inf, posinf=np.inf, neginf=np.inf)
        self.depth_map = F.interpolate(
            torch.from_numpy(depth)[None, None], size=(self.window_size[1], self.window_size[0]),
            mode='nearest').numpy()[0][0]
        self.cur_image_timestamp = img_msg.header.stamp
        self.T_gps = gnss_to_cam2world(gnss)
        self.n_callback += 1
        self.call_insert = True

    def step(self):
        if self.call_insert:
            self.img_insertion()

    def img_insertion(self):
        cam2world = self.T_gps
        world2cam_cv = np.linalg.inv(cam2world)
        world2cam_gl = CV2GL @ world2cam_cv
        input_img = self.cur_image
        input_stamp_s = self.cur_image_timestamp.sec + self.cur_image_timestamp.nanosec * 1e-9
        sensor_data = {
            "rgb": input_img[..., :3].astype(np.float32) / 255,
            "pose": world2cam_gl @ self.T_blender2opengl,
            "depth": self.depth_map,
        }

        # Rendering iteration
        if self.config_obj.clear_color is not None:
            self.window.clear(*self.config_obj.clear_color)
        # Always bind the window framebuffer before calling render
        self.window.use()
        res_img, res_depth, primary_obstacle_poses = self.window.config.render_single_frame(sensor_data)
        res_img = (np.clip(res_img[..., :3], 0, 1) * 255.).astype(np.uint8)  # rgb
        res_depth = np.array(res_depth.cpu())

        if self.show_debug:
            cv2.imshow("Original", cv2.cvtColor(input_img, cv2.COLOR_RGB2BGR))
            cv2.imshow("Inserted", cv2.cvtColor(res_img, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

        stamp = self.get_clock().now().to_msg()
        self.sim_im_publisher.publish(rgb_to_imgmsg(res_img, stamp, "zed2_left_camera_optical_frame"))
        self.sim_depth_publisher.publish(depth_to_imgmsg(res_depth, stamp, "zed2_left_camera_optical_frame"))
        self.publish_pose_array(primary_obstacle_poses, stamp)

        latency = self.get_clock().now().nanoseconds * 1e-9 - input_stamp_s
        self.latency_pub.publish(Float64(data=latency))
        self.n_insertion += 1
        current_time, _ = self.timer.next_frame()
        self.get_logger().info(
            f"frame {self.n_insertion}, {self.n_insertion / current_time:.1f} FPS, latency {latency:.3f} s, "
            f"camera ENU ({cam2world[0, 3]:.2f}, {cam2world[1, 3]:.2f}, {cam2world[2, 3]:.2f})",
            throttle_duration_sec=2.0)
        if not self.window.is_closing:
            self.window.swap_buffers()
        self.call_insert = False

    def publish_pose_array(self, transforms, stamp):
        """Publish obstacle poses in the local ENU frame (x east, y north, z up), the same frame
        as the car pose and the waypoints."""
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        for T_gl in transforms:
            T = P_GL_TO_ENU @ to_numpy_matrix(T_gl)
            q = R.from_matrix(T[:3, :3] / np.cbrt(abs(np.linalg.det(T[:3, :3])))).as_quat()
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
            msg.poses.append(p)
        self.sim_obst_publisher.publish(msg)
        if msg.poses:
            self.get_logger().info(
                "obstacles (ENU east, north, up): " +
                "; ".join(f"({p.position.x:.1f}, {p.position.y:.1f}, {p.position.z:.1f})" for p in msg.poses),
                throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self._cleanup_window_config(self.window, self.timer)
        except Exception as e:  # window may already be gone
            self.get_logger().warn(f"renderer cleanup: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InsertionNode()
    try:
        rclpy.spin(node)  # single-threaded: all GL calls stay on this thread
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
