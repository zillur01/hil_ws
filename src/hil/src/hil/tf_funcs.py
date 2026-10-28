import rospy
import tf
import tf.transformations as tftrans

import numpy as np
from scipy.spatial.transform import Rotation as R

from hil.utils import *
from hil import origin
k_tf_timeout = rospy.Duration(1) # timeout for waiting for a tf msg

def get_gps_tf(gnss: 'GnssSample') -> 'tuple[np.ndarray, np.ndarray]':
    """ Returns the (t,Q)=(translation, Quaternion) between the local origin and the GNSS pose.
        This transform describes the car's pose in our local ENU world coordinates
        (east, north, up in metres from hil.origin.ORIGIN_LAT/LON).
    Args:
        gnss: hil.gnss.GnssSample (lat/lon from NavSatFix, rest from INSNavGeod)
    """
    east, north, up = origin.geodetic_to_local(gnss.lat, gnss.lon, gnss.height)
    pos = np.array([east, north, up]).T # position (3)
    # TODO(USER): roll/pitch sign conventions of INSNavGeod vs the old Novatel Inspva
    # are unverified. They only affect the rendered camera pose (insertion), not the
    # modular agent. Tilt the car and check, then flip signs here if needed.
    pitch, roll, azimuth = gnss.pitch, gnss.roll, float(origin.compass_heading_deg(gnss.heading))
    euler = np.stack([pitch, roll, -azimuth]).T
    r = R.from_euler('xyz', euler, degrees=True)
    rot = r.as_matrix() # (3, 3)

    cam2world = np.zeros((4, 4))
    cam2world[-1, -1] = 1
    cam2world[:3, :3] = rot
    cam2world[:3, 3] = pos

    Q = tftrans.quaternion_from_matrix(cam2world)
    t = cam2world[:3,3]
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