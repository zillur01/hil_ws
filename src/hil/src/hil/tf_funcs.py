import rospy
import tf
import tf.transformations as tftrans

import utm
import numpy as np
from novatel_gps_msgs.msg import Inspva
from scipy.spatial.transform import Rotation as R

from hil.utils import *
k_tf_timeout = rospy.Duration(1) # timeout for waiting for a tf msg

def get_gps_tf(start_gps:Inspva, cur_gps: Inspva) -> 'tuple[np.ndarray, np.ndarray]':
    """ Returns the (t,Q)=(translation, Quaternion) between start_gps and cur_gps
        This transform describes the car's pose in our world coordinates
    Args: 
        start_gps: starting gps message of the car(treated as origin of our world coordinate space)
        cur_gps: current gps message position
    """
    east_origin, north_origin, height_origin = 394654.1397068981, 4438790.165831781, 199.849746366553
    # origin_utm = utm.from_latlon(40.092887,-88.235184)
    # east_origin, north_origin, height_origin = origin_utm[0], origin_utm[1], 200
    utm_data = utm.from_latlon(cur_gps.latitude, cur_gps.longitude)
    east, north = utm_data[0], utm_data[1]
    height = cur_gps.height
    # -------------------------------------------------
    # normalize positions, center at (0, 0, 0)
    east -= east_origin
    north -= north_origin
    height -= height_origin
    pos = np.array([east, north, height]).T # position (3)
    pitch, roll, azimuth = cur_gps.pitch, cur_gps.roll, cur_gps.azimuth
    euler = np.stack([pitch, roll, -azimuth]).T
    r = R.from_euler('xyz', euler, degrees=True)
    rot = r.as_matrix() # (3, 3)

    cam2world = np.zeros((4, 4))
    cam2world[-1, -1] = 1
    cam2world[:3, :3] = rot 
    cam2world[:3, 3] = pos

    world2cam = cam2world 
    # @ np.array([
    #         [0, -1, 0, 0],
    #         [1, 0, 0, 0],
    #         [0,0,1,0],
    #         [0,0,0,1]
    #     ])
    Q = tftrans.quaternion_from_matrix(world2cam)
    t = world2cam[:3,3]    
    # # Q = tftrans.quaternion_from_matrix(cam2world)
    # t = cam2world[:3,3]
    return (t,Q)

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