import rospy
import tf
import tf.transformations as tftrans

import numpy as np
from scipy.spatial.transform import Rotation as R

from hil.utils import *
k_tf_timeout = rospy.Duration(1) # timeout for waiting for a tf msg

# get_gps_tf now lives in hil.pose (ROS-free); re-exported here for the ROS 1 nodes.
from hil.pose import get_gps_tf  # noqa: E402,F401

def get_tf(src_frame: str, dst_frame: str, tf_listener: tf.TransformListener, common_t = None, convertToTransform=True) -> 'tuple[np.ndarray, np.ndarray]':
    "Gets the transform from src to dst based on the /tf or /tf_static topic data"
    (t,Q) = None, None
    if common_t == None:
        common_t = tf_listener.getLatestCommonTime(src_frame, dst_frame)
        print(common_t)
    tf_listener.waitForTransform(dst_frame, src_frame, common_t, k_tf_timeout)  

    # common_t = rospy.Time(0)

    (t, Q) = tf_listener.lookupTransform(dst_frame, src_frame, common_t)
    if convertToTransform:
        return tf_to_transform(t, Q)
    return (t, Q)

def get_gem_tf():
    """DOES NOT WORK! Returns Transform from depth to rgb frame for gem vehicle"""
    listener = tf.TransformListener()
    T_w_rgb = get_tf('base_link', 'front_camera_link', listener) #@ get_tf('front_camera_link', 'front_single_camera_link', listener)

    T_toprack_depth = get_tf('top_rack_link', 'VLP-16_base_link', listener)
    T_w_toprack = get_tf('base_link', 'top_rack_link', listener)
    T_w_depth = T_w_toprack @ T_toprack_depth
    T_depth_rgb = inverse_transform(T_w_depth) @ T_w_rgb 
    T_depth_rgb = get_tf('lidar1', 'front_camera_link', listener)
    # T_depth_rgb[:3,:3] = np.eye(3)
    print(f"T_world^depth: {T_w_depth}")
    print(f"T_world^rgb: {T_w_rgb} done ")
    print(f"T_depth^rgb: {T_depth_rgb}")
    return T_depth_rgb