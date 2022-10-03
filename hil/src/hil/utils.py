import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from tf.transformations import quaternion_matrix

k_wait_msg_timeout = 10

def inverse_transform(T):
    R, t = T[0:3, 0:3], T[0:3, 3]
    print(f"R:{R}, t: {t}")
    inv_T = np.eye(4)
    inv_T[0:3,0:3] = np.transpose(R)
    inv_T[0:3, 3] = -R @ t 
    return inv_T

def tf_to_transform(trans, Q):
    T = quaternion_matrix(Q)
    T[0:3, 3] = trans
    return T

def project_to_image(pts_3d, proj_mat, T_depth_rgb):
    """
    Apply the perspective projection
    Args:
        pts_3d:     3D points in camera coordinate [3, npoints]
        proj_mat:   Projection matrix [3, 4]
    """
    num_pts = pts_3d.shape[0]
    # Change to homogenous coordinates
    pts_3d = np.vstack((pts_3d.T, np.ones(num_pts)))
    points = proj_mat @ T_depth_rgb @ pts_3d
    points[:2, :] /= points[2, :]
    return points[:2, :], points[2, :]
def applyTransform(pts_3d, T):
    """
    Apply the given transform to a set of points
    Args:
        pts_3d:     3D points [3, npoints]
        T:  Transformation Matrix [4, 4]
    """
    num_pts = pts_3d.shape[0]
    # Change to homogenous coordinates
    pts_3d = np.vstack((pts_3d.T, np.ones(num_pts)))
    points = T @ pts_3d
    return points[:3].T
def get_camera_info(topic_name: str)-> CameraInfo:
    cam_info = None
    try:
        cam_info = rospy.wait_for_message(topic_name, CameraInfo, timeout=k_wait_msg_timeout)
    except rospy.ROSException:
        print("Exception while getting camera info")
    return cam_info

def get_dist_matrix(info_msg: CameraInfo):
    return info_msg.D

def get_camera_matrix(info_msg: CameraInfo):
    K = np.array(info_msg.K).reshape(3,3)
    return K

def get_projection_matrix(info_msg: CameraInfo):
    P = np.array(info_msg.P).reshape(3,4)
    print(f"Projection Matrix:{P}")
    return P

def get_img_size(info_msg: CameraInfo):
    print(f"Camera Dims: ({info_msg.height, info_msg.width})")
    return info_msg.height, info_msg.width