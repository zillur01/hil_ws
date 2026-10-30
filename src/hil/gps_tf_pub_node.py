#!/usr/bin/env python3
import tf, tf2_ros
import numpy as np
import rospy
from hil_common import rviz_tools, tf_funcs, utils
from hil_common.globals import *
from hil_common.gnss import GnssState
from geometry_msgs.msg import TransformStamped, Transform

class gps_to_tf_node:
    """ Converts the latest GNSS/INS state (NavSatFix + INSNavGeod) to a ros transform 
        Publishes tansform message on /tf and visualization marker on VIS_MARKER_TOPIC """
    def __init__(self):
        rospy.init_node('gps_to_tf', anonymous=True)
        self.br = tf.TransformBroadcaster()

        # self.static_br = tf2_ros.StaticTransformBroadcaster()
        # self.pub_static_transform(T_lidar_rgb, LIDAR_FRAME, CAMERA_FRAME)

        self.markers = rviz_tools.RvizMarkers(WORLD_FRAME, VIS_MARKER_TOPIC)
        self.gnss = GnssState()
        self.gps_timer = rospy.Timer(rospy.Duration(0.05), self.process_gps_callback)
        self.tf_listener = tf.TransformListener()
        self.last_pub_time = rospy.Time.now()

    def process_gps_callback(self, event=None):
        cur_gps = self.gnss.snapshot()
        if cur_gps is None:
            return
        if self.last_pub_time < rospy.Time.now():
            (t,Q) = tf_funcs.get_gps_tf(cur_gps)
            self.br.sendTransform(
                t, Q, rospy.Time.now(), GPS_FRAME, WORLD_FRAME
            )
            T_gps = utils.tf_to_transform(t, Q)
            self.markers.publishAxis(T_gps, 1, 0.1)
            self.last_pub_time = rospy.Time.now()

    def pub_static_transform(self, T, parent_frame, child_frame):
        """Publishes the given matrix transform T from parent_frame 
            to child_frame on the /tf_static topic"""
        (t,Q) = utils.transform_to_tf(T)

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = child_frame

        static_transformStamped.transform.translation.x = float(t[0])
        static_transformStamped.transform.translation.y = float(t[1])
        static_transformStamped.transform.translation.z = float(t[2])

        static_transformStamped.transform.rotation.x = Q[0]
        static_transformStamped.transform.rotation.y = Q[1]
        static_transformStamped.transform.rotation.z = Q[2]
        static_transformStamped.transform.rotation.w = Q[3]
        self.static_br.sendTransform(static_transformStamped)

    def run(self):
        rospy.spin()

def main():
    node = gps_to_tf_node()
    node.run()

if __name__ == '__main__':
    main()