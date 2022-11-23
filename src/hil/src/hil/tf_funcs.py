import rospy
import tf
import tf.transformations as tftrans

import utm
import numpy as np
from novatel_gps_msgs.msg import Inspva

from hil.utils import *
k_tf_timeout = rospy.Duration(1) # timeout for waiting for a tf msg

def get_gps_tf(start_gps:Inspva, cur_gps: Inspva) -> 'tuple[np.ndarray, np.ndarray]':
    """ Returns the (t,Q)=(translation, Quaternion) between start_gps and cur_gps
        This transform describes the car's pose in our world coordinates
    Args: 
        start_gps: starting gps message of the car(treated as origin of our world coordinate space)
        cur_gps: current gps message position
    """
    T_gps = np.eye(4)
    u1 = utm.from_latlon(cur_gps.latitude, cur_gps.longitude)
    u2 = utm.from_latlon(start_gps.latitude, start_gps.longitude)
    dy = u1[0]-u2[0]
    dx = u1[1]-u2[1]
    dz = cur_gps.height-start_gps.height
    delTheta = -np.deg2rad(cur_gps.azimuth-start_gps.azimuth)
    # T_gps[:3,:3] = np.array([
    #     [np.cos(delTheta), -np.sin(delTheta), 0],
    #     [np.sin(delTheta), np.cos(delTheta), 0],
    #     [0, 0, 1]
    # ])
    # T_gps[:3,3] = [-dx,dy,dz]
    Q = tftrans.quaternion_about_axis(delTheta, (0,0,1))
    t = (-dx, dy, dz)
    return (t,Q)

def get_tf(src_frame: str, dst_frame: str, tf_listener: tf.TransformListener, convertToTransform=True) -> 'tuple[np.ndarray, np.ndarray]':
    "Gets the transform from src to dst based on the /tf or /tf_static topic data"
    (t,Q) = None, None
    tf_listener.waitForTransform(dst_frame, src_frame, rospy.Time(), k_tf_timeout)  
    common_t = tf_listener.getLatestCommonTime(src_frame, dst_frame)
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