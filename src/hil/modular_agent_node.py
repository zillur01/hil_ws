#!/usr/bin/env python3
"""ROS 2 node running the modular longitudinal-control agent (HIL_modular_autonomy).

Inputs : RGB + depth (ZED, time-synced), /navsatfix, /insnavgeod, /pacmod/vehicle_speed_rpt
Output : /modular/pred_speed (pacmod2_msgs/VehicleSpeedRpt) with the desired speed, /modular/latency

Run with `colcon build --symlink-install` (the agent and its weights are loaded from the source tree).
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from pacmod2_msgs.msg import VehicleSpeedRpt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import message_filters

from hil_common.globals import RGB_TOPIC, DEPTH_IM_TOPIC, MODULAR_OUTPUT, MODULAR_LATENCY
from hil_common.gnss import GnssState
from hil_common.origin import compass_heading_deg, latlon_to_local, load_local_waypoints

FRAME_RATE = 8

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03  # allowed difference in seconds between messages to sync


def dist2d(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def default_agent_dir():
    # realpath resolves the symlink created by `colcon build --symlink-install`, so this points
    # at src/HIL_modular_autonomy in the source checkout (not a copy under install/).
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(here, 'src', 'HIL_modular_autonomy')


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ModularAgentNode(Node):
    """Runs ModularAgent on the latest synced RGB+depth and GNSS/speed, at FRAME_RATE Hz."""

    def __init__(self):
        super().__init__('modular_agent')
        self.declare_parameter('waypoint_csv_path', '')
        self.declare_parameter('agent_dir', '')
        waypoint_path = self.get_parameter('waypoint_csv_path').value
        agent_dir = self.get_parameter('agent_dir').value or default_agent_dir()
        if not waypoint_path:
            raise RuntimeError("Parameter 'waypoint_csv_path' is required (local x,y,heading_deg CSV)")
        waypoint_path = os.path.abspath(waypoint_path)  # before chdir below
        if not os.path.isfile(os.path.join(agent_dir, 'agent.py')):
            raise RuntimeError(
                f"agent.py not found in {agent_dir}. Build with `colcon build --symlink-install` "
                "or pass the 'agent_dir' parameter.")

        # Planned xza waypoint trajectory, recorded in the local ENU frame (x, y, heading_deg,
        # header-less CSV, see hil_common/origin.py). Same format as gem_gnss_control's waypoints.
        self.xzas = load_local_waypoints(waypoint_path)
        self.get_logger().info(f"Loaded {len(self.xzas)} waypoints from {waypoint_path}")
        # Closest point in trajectory to our current position
        self.cur_xza_ind = 0

        # The agent loads its weights with paths relative to its own directory.
        sys.path.insert(0, agent_dir)
        os.chdir(agent_dir)
        from agent import ModularAgent  # noqa: E402 (needs the path set up above)
        self.agent = ModularAgent()

        self.bridge = CvBridge()
        self.cur_image = None
        self.cur_depth_map = None
        self.cur_image_timestamp_s = None
        self.cur_xza_pose = None
        self.cur_spd = None

        self.pred_speed_pb = self.create_publisher(VehicleSpeedRpt, MODULAR_OUTPUT, 10)
        self.latency_pub = self.create_publisher(Float64, MODULAR_LATENCY, 10)

        # Pose + speed come from the latest /navsatfix, /insnavgeod, /pacmod/vehicle_speed_rpt
        self.gnss = GnssState(self)

        # Best-effort QoS receives from both reliable and best-effort camera publishers.
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_IM_TOPIC, qos_profile=qos_profile_sensor_data)
        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], k_time_sync_queue_size, k_time_sync_slop)
        self.im_depth_sub.registerCallback(self.im_depth_callback)

        self.start_time = 0
        self.n_prediction = 0  # number of times update_pred_step is called
        self.n_callback = 0    # number of times im_depth_callback is called

        # The prediction (GPU inference) runs in its own callback group so it never blocks the
        # image/GNSS subscriptions when used with a MultiThreadedExecutor.
        self.timer = self.create_timer(
            1.0 / FRAME_RATE, self.step, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Modular agent ready, waiting for image, depth, GNSS and speed")

    def im_depth_callback(self, img_msg: Image, depth_msg: Image):
        """Sensor data (rgb, depth) updated as synced messages are received"""
        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))
        if depth_msg.encoding == '16UC1':  # depth in millimetres
            depth = depth.astype(np.float32) / 1000.0
        depth = depth.astype(np.float32)
        # nan_to_num returns a new (writable) array; the agent edits the depth map in place
        depth = np.nan_to_num(depth, nan=np.inf, posinf=np.inf, neginf=np.inf)
        self.cur_image_timestamp_s = stamp_to_sec(img_msg.header.stamp)
        self.cur_image = image
        self.cur_depth_map = depth
        self.n_callback += 1

    def update_gnss(self):
        """Refresh ego pose (local ENU) and speed (PACMod) from the latest GNSS values.
        Returns False if GNSS/speed data is missing or stale."""
        s = self.gnss.snapshot()
        if s is None:
            return False
        x, y = latlon_to_local(s.lat, s.lon)
        # xza = [x east (m), y north (m), compass heading (rad)]
        self.cur_xza_pose = np.array([x, y, math.radians(float(compass_heading_deg(s.heading)))])
        self.cur_spd = float(s.speed)
        while (self.cur_xza_ind < len(self.xzas) - 1 and
               dist2d(self.cur_xza_pose[:2], self.xzas[self.cur_xza_ind]) >
               dist2d(self.cur_xza_pose[:2], self.xzas[self.cur_xza_ind + 1])):
            self.cur_xza_ind += 1
        return True

    def step(self):
        if self.n_prediction == 2:
            self.start_time = time.perf_counter()
        if self.cur_image is None or self.cur_depth_map is None or not self.update_gnss():
            return
        self.update_pred_step()

    def update_pred_step(self):
        xza_ind = self.cur_xza_ind
        # read the shared fields once; the subscription thread may replace them at any time
        image, depth, input_img_stamp = self.cur_image, self.cur_depth_map, self.cur_image_timestamp_s
        self.agent.update(image, depth, input_img_stamp, self.cur_xza_pose, self.cur_spd)
        dspd, dacc = self.agent.act(self.xzas[xza_ind + 1:])
        self.get_logger().info('frame %d | desired spd: %.4f, desired acc: %.4f' % (self.n_prediction, dspd, dacc))

        latency = self.get_clock().now().nanoseconds * 1e-9 - input_img_stamp
        self.latency_pub.publish(Float64(data=latency))

        self.publish_pred_speed(dspd)
        self.n_prediction += 1
        if self.n_prediction > 2:
            cur_avg_runtime = (time.perf_counter() - self.start_time) / (self.n_prediction - 2)
            self.get_logger().info('%.3f s/frame ==> %.1f fps' % (cur_avg_runtime, 1 / cur_avg_runtime))

    def publish_pred_speed(self, speed_val):
        speed_msg = VehicleSpeedRpt()
        speed_msg.header.stamp = self.get_clock().now().to_msg()
        speed_msg.vehicle_speed_valid = True
        speed_msg.vehicle_speed = float(speed_val)
        self.pred_speed_pb.publish(speed_msg)

    def print_summary(self):
        if self.n_prediction > 2:
            avg_runtime = (time.perf_counter() - self.start_time) / (self.n_prediction - 2)
            self.get_logger().info('avg %.3f s/frame ==> %.1f fps' % (avg_runtime, 1 / avg_runtime))
        self.get_logger().info('Sensor Data n callback: %d' % self.n_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ModularAgentNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
