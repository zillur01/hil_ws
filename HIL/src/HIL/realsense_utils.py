import rospy
import tf

from HIL.utils import *

def get_realsense_tf():
    """Returns Transform from depth to rgb frame for realsense camera"""
    listener = tf.TransformListener()
    (t_depth,Q_depth)  = None, None
    (t_rgb,Q_rgb) = None, None

    while t_depth is None:
        try:
            # print(listener.lookupTransform('/camera_color_optical_frame', '/camera_depth_optical_frame', rospy.Time(0)))
            (t_depth,Q_depth) = listener.lookupTransform('/camera_depth_optical_frame', '/camera_link', rospy.Time(0))
            (t_rgb,Q_rgb) = listener.lookupTransform('/camera_color_optical_frame', '/camera_link', rospy.Time(0))
            # (T_,Q_rgb) = listener.lookupTransform('/camera_link', '/camera_color_optical_frame', rospy.Time(0))
            # print(f"Depth: {(t_depth,Q_depth)}")
            # print(f"RGB: {(t_rgb,Q_rgb)}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    # World to depth
    T_w_depth = tf_to_transform(t_depth, Q_depth)
    # World to rgb
    T_w_rgb = tf_to_transform(t_rgb, Q_rgb)
    T_depth_rgb = inverse_transform(T_w_depth) @ T_w_rgb 
    # self.T_depth_rgb = np.eye(4)
    # self.T_depth_rgb[0:3, 3] = np.array([0, -0.059, 0])
    print(f"T_world^depth: {T_w_depth}")
    print(f"T_world^rgb: {T_w_rgb}")
    print(f"T_depth^rgb: {T_depth_rgb}")
    return T_depth_rgb