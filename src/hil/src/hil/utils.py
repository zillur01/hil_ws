import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy, ros_numpy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import tf.transformations as tftrans

k_wait_msg_timeout = 10

def inverse_transform(T):
    R, t = T[0:3, 0:3], T[0:3, 3]
    print(f"R:{R}, t: {t}")
    inv_T = np.eye(4)
    inv_T[0:3,0:3] = np.transpose(R)
    inv_T[0:3, 3] = -R @ t 
    return inv_T

def transform_to_tf(T):
    t = T[:3,3]
    Q = tftrans.quaternion_from_matrix(T)
    return (t, Q)

def tf_to_transform(trans, Q):
    T = tftrans.quaternion_matrix(Q)
    T[0:3, 3] = trans
    return T

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

def color_pointcloud(pts_depth: np.ndarray, cur_image: np.ndarray, 
                    T_depth_rgb: np.ndarray, proj_mat: np.ndarray, 
                    transform_pc=np.eye(4), visualize_projection=False):
    """Adds rgb from cur_image to the pts_depth pointcloud within the fov of the camera
    Args: 
        pts_depth: 3D points in lidar pointcloud coordinate [3, npoints]
        cur_image: input image in RGB byte order
        T_depth_rgb: Homogenous transform between 3D depth points and camera frame [4,4]
        proj_mat: Camera Projection matrix [3,4]
        transform_pc (optional): Additional transformation to apply to the output 3D pointcloud
        visualize_projection (optional): True if 3D point projected onto image should be visualized
    Returns:
        imgfov_pts[4,...] if visualize_projection=False
        imgfov_pts[4,...], visualized_im if visualization=True
    """
    height, width, _ = cur_image.shape

    pts_2d, depths = project_to_image(pts_depth, proj_mat, T_depth_rgb)
    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < height) & (pts_2d[1, :] >= 0) &
                    (depths > 0)
                    )[0]
    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]
    imgfov_pc_depth = depths[inds]

    max_depth = np.max(imgfov_pc_depth)
    filtered_pts_depth = applyTransform(pts_depth[inds], transform_pc)

    # Build output colored pointcloud
    imgfov_pts = np.zeros(filtered_pts_depth.shape[0], 
                    dtype=[('x', float), ('y', float), ('z', float),
                            ('b', int), ('g', int), ('r', int)])
    imgfov_pts['x'] = filtered_pts_depth[:, 0]
    imgfov_pts['y'] = filtered_pts_depth[:, 1]
    imgfov_pts['z'] = filtered_pts_depth[:, 2]

    x_pixels = imgfov_pc_pixel[0].astype(int)
    y_pixels = imgfov_pc_pixel[1].astype(int)
    rgb_pts = cur_image[y_pixels, x_pixels]
    imgfov_pts['r'] = rgb_pts[:, 0]
    imgfov_pts['g'] = rgb_pts[:, 1]
    imgfov_pts['b'] = rgb_pts[:, 2]
    imgfov_pts = ros_numpy.point_cloud2.merge_rgb_fields(imgfov_pts)

    # Create color map and paint circles with color corresponding to depth
    if visualize_projection:
        max_depth = np.max(imgfov_pc_depth)
        cmap = plt.cm.get_cmap('hsv', 256)
        cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
        depth_map = np.clip(255* (imgfov_pc_depth/max_depth), 0, 255).astype(int)
        # Paint lidar points onto the image 
        depth_image = cur_image.copy()
        depth_image[y_pixels, x_pixels] = cmap[depth_map]
        return imgfov_pts, depth_image
        # cv2.imshow("3D projected onto Image window", depth_image)
        # cv2.waitKey(1)
    
    return imgfov_pts


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