#!/usr/bin/env python3
"""Records waypoints in the local ENU frame (hil_common/origin.py), same CSV format as
gem_gnss_control's record_waypoints.py: header-less rows of `x, y, heading_deg`.

Drive the route manually (or play a bag of it) with this node running. The output
file is what modular_agent_node / metrics_node read via `waypoint_csv_path`.

Parameters: `filename` (default recorded_track[.csv]), `output_dir` (default data/waypoints
in the source tree, needs `colcon build --symlink-install`).
"""
import csv
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hil_common.gnss import GnssState
from hil_common.origin import latlon_to_local

MIN_DISTANCE_M = 0.5  # only append a point after moving this far from the last one
RATE_HZ = 5.0


def default_output_dir():
    # realpath resolves the --symlink-install link, so this is src/hil/data/waypoints
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'waypoints')


class SaveWaypointNode(Node):
    def __init__(self):
        super().__init__('save_waypoint_node')
        filename = self.declare_parameter('filename', 'recorded_track').value
        out_dir = self.declare_parameter('output_dir', '').value or default_output_dir()
        os.makedirs(out_dir, exist_ok=True)
        self.out_path = os.path.join(out_dir, filename if filename.endswith('.csv') else filename + '.csv')

        self.gnss = GnssState(self)
        self.last_xy = None
        self.n = 0
        self.file = open(self.out_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.get_logger().info(f"Recording waypoints to {self.out_path}")
        self.timer = self.create_timer(1.0 / RATE_HZ, self.step)

    def step(self):
        s = self.gnss.snapshot(need_speed=False)  # pose only, PACMod speed not needed
        if s is None:
            return
        x, y = latlon_to_local(s.lat, s.lon)
        if self.last_xy is None or math.hypot(x - self.last_xy[0], y - self.last_xy[1]) >= MIN_DISTANCE_M:
            # raw INSNavGeod heading, like gem_gnss_control; origin.compass_heading_deg()
            # is applied when the file is loaded.
            self.writer.writerow([x, y, s.heading])
            self.file.flush()
            self.last_xy = (x, y)
            self.n += 1
            self.get_logger().info(f"[{self.n}] x={x:.3f} y={y:.3f} heading={s.heading:.2f}")

    def close(self):
        self.file.close()
        print(f"Saved {self.n} waypoints to {self.out_path}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveWaypointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
