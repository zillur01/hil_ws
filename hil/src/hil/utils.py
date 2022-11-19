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
    return np.array(info_msg.D)

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


def fit_rigid(src, tgt, normals=None, point_to_plane = False):
  """
  Args: 
    src: source points ndarray of size (3, n)
    tgt: target points ndarray of size (3,n)
  """
  if point_to_plane:
    p = src
    q = tgt
    n = normals
    c = np.cross(p.T,n.T).T
    cx, cy, cz = c
    nx, ny, nz = n
    vals = np.zeros(p.shape[1])
    for i in range(p.shape[1]):
        vals[i] = ((p[:,i]-q[:,i]) @ n[:,i])
    b = -np.array([
        np.sum(cx*vals),
        np.sum(cy*vals),
        np.sum(cz*vals),
        np.sum(nx*vals),
        np.sum(ny*vals),
        np.sum(nz*vals)
    ])
    print(b)
    A_upper = np.hstack((c@c.T, c@n.T))
    A_lower = np.hstack((n@c.T, n@n.T))
    A = np.vstack((A_upper,A_lower))
    print(A.shape)
    L = np.linalg.cholesky(A)
    y = np.linalg.solve(L,b)
    alph, beta, gam, tx, ty, tz  = np.linalg.solve(L.T,y)
    T = np.array([
        [1, -gam, beta, tx],
        [gam, 1, -alph, ty],
        [-beta, alph, 1, tz],
        [0, 0, 0, 1]
    ])
  else:
    T = np.identity(4)
    src_centroid = np.mean(src, axis=1)
    tgt_centroid = np.mean(tgt, axis=1)
    centered_src = (src.T - src_centroid).T
    centered_tgt = (tgt.T - tgt_centroid).T
    # print(centered_src, centered_tgt)
    H = centered_src @ centered_tgt.T
    # print(H)
    U, S, Vh = np.linalg.svd(H)
    R = Vh.T @ np.diag([1,1,np.linalg.det(Vh.T@U.T)]) @ U.T
    # print(R)
    t = tgt_centroid - R @ src_centroid
    T[:3,:3] = R
    T[:3, 3] = t
  # -------------------------
  return T