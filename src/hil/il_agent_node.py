#!/usr/bin/env python3
"""ROS 2 node running the imitation-learning agent (HIL_imitation_learning, a git submodule:
`git submodule update --init src/hil/src/HIL_imitation_learning`).

Inputs : RGB image, /navsatfix + /insnavgeod + /pacmod/vehicle_speed_rpt, /pacmod/brake_cmd (human brake)
Output : IL_OUTPUT (/il/pred_brake, pacmod2_msgs/SystemCmdFloat, predicted brake in `command`),
         /il/desired_speed (pacmod2_msgs/VehicleSpeedRpt), IL_LATENCY

Run with `colcon build --symlink-install` (the agent is loaded from the source tree), or pass `il_dir`.
"""
import os
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from pacmod2_msgs.msg import SystemCmdFloat, VehicleSpeedRpt
from PIL import Image as pil_img
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from hil_common.globals import IL_LATENCY, IL_OUTPUT, RGB_TOPIC
from hil_common.gnss import GnssState
from hil_common.tf_funcs import get_gps_tf
from hil_common.utils import stamp_to_sec, tf_to_transform

FRAME_RATE = 10
BRAKE_THRESHOLD = 0.05
CRUISE_SPEED = 3.0 # 1.5
k_brake_gain = 1
PACMOD_BRAKE_CMD_TOPIC = "/pacmod/brake_cmd"


def default_il_dir():
    # realpath resolves the --symlink-install link, so this is src/HIL_imitation_learning in the checkout
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), 'src', 'HIL_imitation_learning')


class ILAgentNode(Node):
    def __init__(self):
        super().__init__('il_agent')
        il_lib_path = self.declare_parameter('il_dir', '').value or default_il_dir()
        il_lib_path = os.path.abspath(il_lib_path)
        if not os.path.isfile(os.path.join(il_lib_path, 'ros_main.py')):
            raise RuntimeError(
                f"ros_main.py not found in {il_lib_path}. Run "
                "`git submodule update --init src/hil/src/HIL_imitation_learning`, build with "
                "`colcon build --symlink-install`, or pass the 'il_dir' parameter.")
        # As under catkin: the parent is on the path (`import HIL_imitation_learning.ros_main`) and so is
        # the directory itself (the agent's own top-level imports). It loads files relative to its directory.
        sys.path.append(os.path.dirname(il_lib_path))
        sys.path.append(il_lib_path)
        os.chdir(il_lib_path)
        from HIL_imitation_learning.ros_main import IL_Agent, save_frames_as_gif, visualize_data
        self.save_frames_as_gif, self.visualize_data = save_frames_as_gif, visualize_data

        # IL Agent
        self.agent = IL_Agent(True)
        self.gif_frames = []
        self.start_time = None
        self.frame_rate_start = None
        self.last_time = None

        self.bridge = CvBridge()
        self.gnss = GnssState(self)
        self.image_sub = self.create_subscription(Image, RGB_TOPIC, self.im_gps_cb, qos_profile_sensor_data)
        self.gps_timer = self.create_timer(0.05, self.gps_cb)
        # human brake command, used as the agent's brake history
        self.pacmod_brake_sub = self.create_subscription(
            SystemCmdFloat, PACMOD_BRAKE_CMD_TOPIC, self.brake_cb, qos_profile_sensor_data)

        # Predicted brake / speed publishers
        self.pred_speed_pb = self.create_publisher(SystemCmdFloat, IL_OUTPUT, 10)
        self.latency_pub = self.create_publisher(Float64, IL_LATENCY, 10)
        self.desired_speed_pb = self.create_publisher(VehicleSpeedRpt, "/il/desired_speed", 10)

        self.count = 0
        self.is_running_prediction = False
        self.cur_speed = 0
        # The prediction runs in its own callback group so it never blocks the sensor callbacks
        # when used with a MultiThreadedExecutor.
        self.timer = self.create_timer(
            1.0 / FRAME_RATE, self.step, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Node is Ready")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def brake_cb(self, cmd_msg: SystemCmdFloat):
        if not self.is_running_prediction and self.start_time:
            stamp = np.float32(self.now_s() - self.start_time)
            self.agent.add_brake_data(stamp, cmd_msg.command)

    def gps_cb(self):
        gps_msg = self.gnss.snapshot()
        if gps_msg is None:
            return
        if not self.is_running_prediction and self.start_time:
            gps_stamp = np.float32(gps_msg.stamp - self.start_time)
            if len(self.agent.R_timestamp_cache)==0 or gps_stamp > self.agent.R_timestamp_cache[-1]:
                tot_speed = float(gps_msg.speed) # PACMod vehicle speed
                self.cur_speed = tot_speed
                (t,Q) = get_gps_tf(gps_msg)
                pose = tf_to_transform(t, Q)

                self.agent.add_speed_data(gps_stamp, tot_speed)
                self.agent.add_pose_data(gps_stamp, pose)

    def im_gps_cb(self, img_msg: Image):
        if not self.is_running_prediction and self.start_time:
            stamp = np.float32(stamp_to_sec(img_msg.header.stamp) - self.start_time)
            cv2_rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
            cur_image = pil_img.fromarray(cv2_rgb_image)
            self.agent.add_img_data(stamp, cur_image)

    def publish_pred_brake(self, brake_val):
        brake_msg = SystemCmdFloat()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.command = float(brake_val)
        self.pred_speed_pb.publish(brake_msg)

        desired_speed = CRUISE_SPEED
        if brake_val > BRAKE_THRESHOLD:
            # desired_speed = max(self.cur_speed - k_brake_gain * brake_val, 0)
            desired_speed = 0.0
        speed_msg = VehicleSpeedRpt()
        speed_msg.header.stamp = self.get_clock().now().to_msg()
        speed_msg.vehicle_speed = float(desired_speed)
        speed_msg.vehicle_speed_valid = True
        self.desired_speed_pb.publish(speed_msg)

    def step(self):
        if self.start_time is None:
            # with use_sim_time the clock reads 0 until the first /clock message
            now = self.now_s()
            if now > 0.0:
                self.start_time = now
                self.frame_rate_start = time.time()
                self.get_logger().info(f"Ros Start_Time {self.start_time}")
            return
        self.is_running_prediction = True
        cache_time = self.agent.img_timestamp_cache[-1] if len(self.agent.img_timestamp_cache) > 0 else None
        try:
            output_data = self.agent.run_prediction()
        finally:
            self.is_running_prediction = False
        if output_data is not None:
            curr_time = self.now_s() - self.start_time
            self.last_time = curr_time
            if cache_time is not None:
                latency = abs(curr_time - cache_time)
                print("Cur_time:", curr_time, "Cur Latency", latency)
                self.latency_pub.publish(Float64(data=float(latency)))
            self.publish_pred_brake(output_data['pred'])
            self.count += 1
            fps = self.count / (time.time() - self.frame_rate_start)
            print("fps", fps, "Frame_Count:", self.count, "Predicted Brake:", output_data['pred'])
            curr_gif_frame = self.visualize_data(output_data)
            self.gif_frames.append(curr_gif_frame)

    def on_shutdown(self):
        print(len(self.gif_frames))
        if len(self.gif_frames) > 0:
            self.save_frames_as_gif(self.gif_frames, 'rosbag_run')
            print("Saved Frames as GIF")


def main(args=None):
    rclpy.init(args=args)
    node = ILAgentNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.on_shutdown()
        print("Shutting Down")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
