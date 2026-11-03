#!/usr/bin/env python3
"""Reads the car's current pose and prints the insertion command that puts a pedestrian at a place given
relative to the car, e.g. 25 m ahead and 4 m to its left, walking across to the right:

  ros2 run hil plan_pedestrian.py --ahead 25 --left 4

Run it with the car parked at the start of the test (the same spot where the node will be started).
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node

from hil_common.gnss import GnssState
from hil_common.origin import compass_heading_deg, latlon_to_local


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ahead', type=float, default=25.0, help='metres ahead of the car (default 25)')
    ap.add_argument('--left', type=float, default=4.0, help='metres to the left of the car line (default 4)')
    ap.add_argument('--walk', choices=['right', 'left'], default='right',
                    help='direction the pedestrian walks, seen from the car (default right)')
    ap.add_argument('--trigger', type=float, default=15.0,
                    help='the pedestrian starts walking when the car is this close (default 15 m)')
    ap.add_argument('--config', default='gem/jaywalker.yaml', help='base config (needs a jaywalking actor_0)')
    ap.add_argument('--timeout', type=float, default=10.0, help='seconds to wait for GNSS (default 10)')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node('plan_pedestrian')
    gnss = GnssState(node)
    sample = None
    end = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while rclpy.ok() and sample is None and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        sample = gnss.snapshot(need_speed=False)
    node.destroy_node()
    rclpy.shutdown()
    if sample is None:
        sys.exit('No /navsatfix + /insnavgeod received. Are the Septentrio driver and topics running?')

    x0, y0 = latlon_to_local(sample.lat, sample.lon)
    hdg = float(compass_heading_deg(sample.heading))
    t = math.radians(hdg)
    east = x0 + args.ahead * math.sin(t) - args.left * math.cos(t)
    north = y0 + args.ahead * math.cos(t) + args.left * math.sin(t)
    walk = (hdg + (90.0 if args.walk == 'right' else -90.0)) % 360.0

    print(f"Car: ENU ({x0:.2f}, {y0:.2f}), heading {hdg:.1f} deg (compass)")
    print(f"Pedestrian: ENU ({east:.2f}, {north:.2f}), {args.ahead:g} m ahead, {args.left:g} m left, "
          f"walks towards compass {walk:.1f} deg ({args.walk}), starts when the car is within {args.trigger:g} m\n")
    print("ros2 launch hil insertion.launch.py "
          f"config_path:={args.config} auto_origin_alt:=true "
          f"actor_east:={east:.2f} actor_north:={north:.2f} actor_walk_heading_deg:={walk:.1f} "
          f"trigger_distance:={args.trigger:g}")


if __name__ == '__main__':
    main()
