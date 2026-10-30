import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt
import cv2
import hil_common.utils as utils
P = np.array([[527.56256104 ,  0.    ,     616.24493408  , 0.        ],
 [  0.    ,     527.56256104 ,359.1973877  ,  0.        ],
 [  0.     ,      0.      ,     1.     ,      0.        ]])
T_depth_rgb = np.eye(4)
im = cv2.imread('/home/bhargav/catkin_ws/src/HIL_Research/hil/experimentation/aruco_testing/static_aruco.png')
im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
implot = plt.imshow(im)
lidar_pts = []
cam_pts = []
with open('/home/bhargav/catkin_ws/src/HIL_Research/lidar_camera_calibration/conf/points.txt') as f:
    content = f.read().splitlines()
    n = int(content[0])
    for i in range(1,n+1):
        lidar_pts.append([float(i) for i in content[i].split()])
    for i in range(n+1, 2*n+1):
        cam_pts.append([float(i) for i in content[i].split()])
with open('/home/bhargav/catkin_ws/src/HIL_Research/lidar_camera_calibration/log/avg_values.txt') as f:
    content = f.read().splitlines()
    vals = content[-7:-1]
    T_depth_rgb[:3, 3] = [float(i) for i in vals[:3]]
    for i in range(3,6):
        T_depth_rgb[i-3, :3] = [float(j) for j in vals[i].split()]

lidar_pts = np.array(lidar_pts)
np.save('lidar_pts.npy', lidar_pts)
cam_pts = np.array(cam_pts)
np.save('cam_pts.npy', cam_pts)
cam_pts[:, 0] -= 0.1
T_depth_rgb = utils.fit_rigid(lidar_pts.T, cam_pts.T)
print(repr(T_depth_rgb))
num_pts = cam_pts.shape[0]
uv_pts = utils.applyTransform(cam_pts, P).T
uv_pts[:2, :] /= uv_pts[2, :]
lidar_uv_pts = utils.applyTransform(lidar_pts, P@T_depth_rgb).T
lidar_uv_pts[:2, :] /= lidar_uv_pts[2, :]

plt.figure(1)
plt.scatter(x=lidar_uv_pts[0], y=lidar_uv_pts[1], c='b', s=40, label="Lidar")

plt.scatter(x=uv_pts[0], y=uv_pts[1], c='r', s=40, label="Camera")
plt.legend()

plt.figure(2)
plt.imshow(im)
lidar_uv_pts = utils.applyTransform(lidar_pts, P).T
lidar_uv_pts[:2, :] /= lidar_uv_pts[2, :]
plt.scatter(x=lidar_uv_pts[0], y=lidar_uv_pts[1], c='b', s=40, label="Lidar")

plt.scatter(x=uv_pts[0], y=uv_pts[1], c='r', s=40, label="Camera")
plt.legend()
plt.show()
