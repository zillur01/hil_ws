#!/usr/bin/env python3
"""Test helper: publishes a camera image + depth, /navsatfix, /insnavgeod and a PACMod speed so the
insertion node (or the modular agent) can be tried without the car.

The "car" either stands still at (x, y) facing `heading_deg` (compass degrees, 0 = north, 90 = east),
or, if `waypoint_csv_path` is given, drives along that path at `speed` m/s.
Positions are in the local ENU frame (metres east / north of the origin in hil_common/origin.py).

  ros2 run hil fake_sensor_node.py --ros-args -p x:=0.0 -p y:=-13.0 -p heading_deg:=90.0
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from pacmod2_msgs.msg import VehicleSpeedRpt
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod

from hil_common import origin
from hil_common.ros_image import rgb_to_imgmsg, depth_to_imgmsg


def make_background(width, height):
    """Sky above a horizon at mid-height, asphalt-grey ground below, so objects stand out."""
    img = np.zeros((height, width, 3), np.uint8)
    img[:height // 2] = (150, 190, 235)
    img[height // 2:] = (95, 95, 95)
    return img


class FakeSensorNode(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        # dynamic typing so that `-p x:=30` (integer) is accepted as well as `-p x:=30.0`
        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('x', 0.0, num)
        self.declare_parameter('y', 0.0, num)
        self.declare_parameter('heading_deg', 90.0, num)
        self.declare_parameter('height', origin.ORIGIN_ALT, num)  # ellipsoidal height reported by the INS
        self.declare_parameter('waypoint_csv_path', '')
        self.declare_parameter('speed', 2.0, num)
        self.declare_parameter('rate_hz', 10.0, num)
        self.declare_parameter('image_path', '')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('rgb_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.x, self.y, self.heading = float(g('x')), float(g('y')), float(g('heading_deg'))
        self.height, self.speed, self.dt = float(g('height')), float(g('speed')), 1.0 / float(g('rate_hz'))
        w, h = int(g('image_width')), int(g('image_height'))
        if g('image_path'):
            import cv2
            bgr = cv2.imread(g('image_path'))
            if bgr is None:
                raise RuntimeError(f"cannot read image {g('image_path')}")
            self.rgb = cv2.cvtColor(cv2.resize(bgr, (w, h)), cv2.COLOR_BGR2RGB)
        else:
            self.rgb = make_background(w, h)
        self.depth = np.full((h, w), 100.0, np.float32)

        self.path = None
        if g('waypoint_csv_path'):
            wp = np.loadtxt(g('waypoint_csv_path'), delimiter=',', ndmin=2)[:, :2]
            seg = np.hypot(*np.diff(wp, axis=0).T)
            self.path, self.cum = wp, np.concatenate([[0.0], np.cumsum(seg)])
            self.s = 0.0
            self.get_logger().info(f"Driving along {len(wp)} waypoints ({self.cum[-1]:.1f} m) at {self.speed} m/s")
        else:
            self.get_logger().info(
                f"Standing at ENU ({self.x:.2f}, {self.y:.2f}) facing {self.heading:.1f} deg (compass)")

        self.rgb_pub = self.create_publisher(Image, g('rgb_topic'), qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, g('depth_topic'), qos_profile_sensor_data)
        self.fix_pub = self.create_publisher(NavSatFix, '/navsatfix', 10)
        self.ins_pub = self.create_publisher(INSNavGeod, '/insnavgeod', 10)
        self.speed_pub = self.create_publisher(VehicleSpeedRpt, '/pacmod/vehicle_speed_rpt', 10)
        self.create_timer(self.dt, self.tick)

    def advance_along_path(self):
        self.s = min(self.s + self.speed * self.dt, self.cum[-1])
        i = min(int(np.searchsorted(self.cum, self.s, side='right')) - 1, len(self.path) - 2)
        f = (self.s - self.cum[i]) / max(self.cum[i + 1] - self.cum[i], 1e-9)
        (x0, y0), (x1, y1) = self.path[i], self.path[i + 1]
        self.x, self.y = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        self.heading = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0  # compass: east = 90

    def tick(self):
        if self.path is not None:
            self.advance_along_path()
        now = self.get_clock().now().to_msg()
        lat, lon = origin.local_to_latlon(self.x, self.y)

        fix = NavSatFix()
        fix.header.stamp, fix.header.frame_id = now, 'gnss'
        fix.latitude, fix.longitude, fix.altitude = float(lat), float(lon), self.height
        self.fix_pub.publish(fix)

        ins = INSNavGeod()
        ins.header.stamp = now
        ins.latitude, ins.longitude, ins.height = math.radians(lat), math.radians(lon), self.height
        # raw INSNavGeod heading, so that origin.compass_heading_deg(raw) == the requested heading
        ins.heading = float((self.heading - origin.HEADING_OFFSET_DEG) % 360.0)
        ins.ve = self.speed * math.sin(math.radians(self.heading)) if self.path is not None else 0.0
        ins.vn = self.speed * math.cos(math.radians(self.heading)) if self.path is not None else 0.0
        self.ins_pub.publish(ins)

        spd = VehicleSpeedRpt()
        spd.header.stamp = now
        spd.vehicle_speed = self.speed if self.path is not None else 0.0
        spd.vehicle_speed_valid = True
        self.speed_pub.publish(spd)

        self.rgb_pub.publish(rgb_to_imgmsg(self.rgb, now, 'zed2_left_camera_optical_frame'))
        self.depth_pub.publish(depth_to_imgmsg(self.depth, now, 'zed2_left_camera_optical_frame'))


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
