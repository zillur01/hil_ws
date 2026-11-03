import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from hil_common.utils import tf_to_transform
# get_gps_tf lives in hil_common.pose (ROS-free); re-exported here for the nodes.
from hil_common.pose import get_gps_tf  # noqa: F401

k_tf_timeout = Duration(seconds=1)  # timeout for waiting for a tf msg


def get_tf(src_frame: str, dst_frame: str, tf_buffer: tf2_ros.Buffer, common_t: Time = None,
           convertToTransform=True) -> 'np.ndarray | tuple[np.ndarray, np.ndarray]':
    """Gets the transform from src to dst based on the /tf or /tf_static topic data.

    tf_buffer: a tf2_ros.Buffer filled by a tf2_ros.TransformListener of a spinning node.
    common_t: time of the transform; None = latest available.
    Raises tf2_ros.LookupException / ExtrapolationException / ConnectivityException if not available."""
    when = Time() if common_t is None else common_t
    tf_msg = tf_buffer.lookup_transform(dst_frame, src_frame, when, timeout=k_tf_timeout)
    tr, rot = tf_msg.transform.translation, tf_msg.transform.rotation
    t = np.array([tr.x, tr.y, tr.z])
    Q = np.array([rot.x, rot.y, rot.z, rot.w])
    if convertToTransform:
        return tf_to_transform(t, Q)
    return (t, Q)
