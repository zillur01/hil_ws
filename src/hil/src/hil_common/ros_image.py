"""sensor_msgs/Image <-> numpy without cv_bridge (so the insertion node has no cv_bridge/numpy-ABI dependency)."""
import numpy as np
from sensor_msgs.msg import Image

_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}


def imgmsg_to_rgb(msg: Image) -> np.ndarray:
    """uint8 (H, W, 3) RGB array from rgb8 / bgr8 / rgba8 / bgra8 / mono8."""
    if msg.encoding not in _CHANNELS:
        raise ValueError(f"unsupported image encoding '{msg.encoding}'")
    c = _CHANNELS[msg.encoding]
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * c]
    arr = arr.reshape(msg.height, msg.width, c)
    if msg.encoding in ('bgr8', 'bgra8'):
        arr = arr[..., [2, 1, 0]]
    elif msg.encoding == 'mono8':
        arr = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(arr[..., :3])


def depth_msg_to_meters(msg: Image) -> np.ndarray:
    """float32 (H, W) depth in metres from 32FC1 (metres) or 16UC1 (millimetres)."""
    if msg.encoding == '32FC1':
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return d.copy()
    if msg.encoding == '16UC1':
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return d.astype(np.float32) / 1000.0
    raise ValueError(f"unsupported depth encoding '{msg.encoding}'")


def rgb_to_imgmsg(rgb: np.ndarray, stamp, frame_id: str = '') -> Image:
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def depth_to_imgmsg(depth: np.ndarray, stamp, frame_id: str = '') -> Image:
    depth = np.ascontiguousarray(depth, dtype=np.float32)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = depth.shape[:2]
    msg.encoding = '32FC1'
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    msg.data = depth.tobytes()
    return msg
