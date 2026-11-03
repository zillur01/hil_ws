#!/usr/bin/env python3
"""Writes <bag dir>_extracted/<bag name>/synced.csv for imitation learning: for every time-synced
RGB + depth pair, the image / GNSS / depth stamps (ns) and the index of the latest PACMod accel,
brake, steering and speed message received at that moment.

ROS 2 headers have no `seq`, so the *_seqnum columns are per-topic receive counters (0 = first
message of that topic in the bag), not the ROS 1 header.seq values.

The node exits (and writes the CSV) 5 s after the last synced frame, i.e. when the bag has ended.
Parameter `input_bag_path`: the ros2 bag being played. Started by save_data_IL.launch.py."""
import csv
import os
import time

import message_filters
import rclpy
from pacmod2_msgs.msg import SystemCmdFloat, SystemRptFloat, VehicleSpeedRpt
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hil_common.globals import DEPTH_IM_TOPIC, PACMOD_SPEED_TOPIC, RGB_TOPIC
from hil_common.gnss import GnssState
from hil_common.utils import stamp_to_nsec

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync
IDLE_EXIT_S = 5.0

# PACMod topics of the ROS 2 driver (pacmod2)
PACMOD_ACCEL_CMD_TOPIC = "/pacmod/accel_cmd"
PACMOD_BRAKE_CMD_TOPIC = "/pacmod/brake_cmd"
PACMOD_STEER_RPT_TOPIC = "/pacmod/steering_rpt"


class SaveDataILNode(Node):
    def __init__(self):
        super().__init__('save_data_node')
        bag_filepath = self.declare_parameter('input_bag_path', '').value
        if not bag_filepath:
            raise RuntimeError("Parameter 'input_bag_path' is required")
        bag_filepath = os.path.normpath(bag_filepath)  # ros2 bags are directories: drop a trailing '/'
        extracted_bag_name = os.path.splitext(os.path.basename(bag_filepath))[0]
        self.fileDir = os.path.join(os.path.dirname(bag_filepath) + "_extracted", extracted_bag_name)
        self.get_logger().info(f"FileDir: {self.fileDir}")
        self.csv_path = os.path.join(self.fileDir, 'synced.csv')
        os.makedirs(self.fileDir, exist_ok=True)

        # Synced topics
        self.image_sub = message_filters.Subscriber(self, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_IM_TOPIC, qos_profile=qos_profile_sensor_data)
        self.gnss = GnssState(self)

        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            k_time_sync_queue_size,
            k_time_sync_slop
            )
        self.im_depth_sub.registerCallback(self.im_depth_callback)
        self.iter = 0

        # PACMod topics: count the messages of each (see module docstring)
        self.counts = {'accel': None, 'brake': None, 'steer': None, 'speed': None}
        for key, msg_type, topic in [('accel', SystemCmdFloat, PACMOD_ACCEL_CMD_TOPIC),
                                     ('brake', SystemCmdFloat, PACMOD_BRAKE_CMD_TOPIC),
                                     ('steer', SystemRptFloat, PACMOD_STEER_RPT_TOPIC),
                                     ('speed', VehicleSpeedRpt, PACMOD_SPEED_TOPIC)]:
            self.create_subscription(msg_type, topic, lambda _msg, k=key: self.count_cb(k), qos_profile_sensor_data)
        self.last_save_time = None

        self.fields = ["image_stamp", "gps_stamp", "depth_stamp",
                        "accel_seqnum", "brake_seqnum", "steer_seqnum", "speed_seqnum"]
        self.rows = []
        # wall clock: with use_sim_time the node clock stops when the bag ends
        self.timer = self.create_timer(1.0, self.check_done, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def count_cb(self, key):
        self.counts[key] = 0 if self.counts[key] is None else self.counts[key] + 1

    def im_depth_callback(self, img_msg: Image, depth_msg: Image):
        gps_msg = self.gnss.snapshot()
        if gps_msg is None:
            return
        if self.counts['brake'] is not None and self.counts['accel'] is not None:
            synced_row = [
                stamp_to_nsec(img_msg.header.stamp),
                int(gps_msg.stamp * 1e9),
                stamp_to_nsec(depth_msg.header.stamp),
                self.counts['accel'],
                self.counts['brake'],
                self.counts['steer'],
                self.counts['speed'],
            ]
            self.rows.append(synced_row)
            self.last_save_time = time.time()
            self.iter += 1

    def check_done(self):
        if self.last_save_time is not None and time.time() - self.last_save_time > IDLE_EXIT_S:
            raise SystemExit  # leaves spin(); main() writes the CSV

    def write_csv(self):
        print(f"Writing {len(self.rows)} rows to {self.csv_path}")
        with open(self.csv_path, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(self.fields)
            csvwriter.writerows(self.rows)


def main(args=None):
    rclpy.init(args=args)
    node = SaveDataILNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.write_csv()
        print("Shutting Down")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
