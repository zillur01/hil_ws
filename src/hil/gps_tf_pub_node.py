#!/usr/bin/env python3
"""Publishes the car pose from GNSS/INS (/navsatfix + /insnavgeod) as the tf WORLD_FRAME -> GPS_FRAME
(local ENU frame of hil_common/origin.py), and an axis marker on VIS_MARKER_TOPIC for RViz."""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from hil_common import rviz_tools, utils
from hil_common.globals import GPS_FRAME, VIS_MARKER_TOPIC, WORLD_FRAME
from hil_common.gnss import GnssState
from hil_common.tf_funcs import get_gps_tf


class GpsToTfNode(Node):
    """Converts the latest GNSS/INS state (NavSatFix + INSNavGeod) to a ros transform
        Publishes transform message on /tf and visualization marker on VIS_MARKER_TOPIC"""
    def __init__(self):
        super().__init__('gps_to_tf')
        self.br = TransformBroadcaster(self)
        self.markers = rviz_tools.RvizMarkers(self, WORLD_FRAME, VIS_MARKER_TOPIC)
        self.gnss = GnssState(self)
        self.last_pub_time = None
        self.gps_timer = self.create_timer(0.05, self.process_gps_callback)

    def process_gps_callback(self):
        cur_gps = self.gnss.snapshot(need_speed=False)  # pose only, PACMod speed not needed
        if cur_gps is None:
            return
        now = self.get_clock().now()
        # with sim time the clock can stand still; don't send the same stamp twice
        if self.last_pub_time is not None and now <= self.last_pub_time:
            return
        (t, Q) = get_gps_tf(cur_gps)
        msg = TransformStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = WORLD_FRAME
        msg.child_frame_id = GPS_FRAME
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(v) for v in t)
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = (float(v) for v in Q)
        self.br.sendTransform(msg)
        self.markers.publishAxis(utils.tf_to_transform(t, Q), 1.0, 0.1)
        self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GpsToTfNode()
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
