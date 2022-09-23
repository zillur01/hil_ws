#!/usr/bin/env python3
from hil import *
from hil.lidar2camNode import lidar2camNode
import numpy as np

RGB_TOPIC = "/rgb"
LIDAR_PC_TOPIC = "/cloud"
CAMERA_INFO_TOPIC = "/camera_info"

T_depth_rgb = np.array([
    [0, -1, 0, 0.2],
    [0, 0, -1, -0.19],
    [1, 0, 0, -0.15],
    [0, 0, 0, 1]
])

# T_depth_rgb[:3,:3] = np.array([
#     [0.223396 , -0.972437 , 0.0667876],
#   [0.0444629, -0.0582815 ,  -0.99731],
#    [0.973713 ,  0.225764 , 0.0302176 ]
# ])
# T_depth_rgb[:3,3] = np.array([
#     -0.221484, -0.38841, 0.354662
# ])
# R_init = np.array([
#     [  0.0007963, -0.0000000, -0.9999997, 0 ],
#   [-0.9999993,  0.0007963, -0.0007963,0 ],
#   [ 0.0007963,  0.9999997,  0.0000006,0 ],
#   [0,0,0,1]])

# T_avg =  np.array([

#  [-0.944768  , -0.326835  ,-0.0243213    , 1.32351],
#  [ -0.327727 ,   0.941513 ,  0.0784035 ,  -0.143747],
# [-0.00272614  , 0.0820439  , -0.996625  ,   4.29098],
#  [        0     ,      0      ,     0       ,    1],
#  ])
# T_depth_rgb = R_init @ T_avg
# print(T_depth_rgb)
# center 4 points, RMSE 0.0446
# T_depth_rgb = np.array([
# [ 0.0247386 , -0.998504 ,-0.0487721,  -.115547],
# [-0.0620504 , 0.0471593 , -0.996958,  -.172072],
# [0.997766 , 0.0276897, -0.0607908  ,  0.05],
# [         0       ,   0       ,   0      ,    1]
# ])
# [  0.0007963, -0.0000000, -0.9999997;
#   -0.9999993,  0.0007963, -0.0007963;
#    0.0007963,  0.9999997,  0.0000006 ]
# -0.12 0 1.51 lidar pose from center link
# "0.43 0 1.19" camera pose from center link
def main():
    lidar2cam_node = lidar2camNode(
        CAMERA_INFO_TOPIC,
        LIDAR_PC_TOPIC,
        RGB_TOPIC,
        T_depth_rgb
    )
    lidar2cam_node.run()

if __name__ == '__main__':
    main()