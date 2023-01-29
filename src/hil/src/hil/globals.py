import numpy as np
# Topic Names
RGB_TOPIC = "/rgb"
DEPTH_IM_TOPIC = "/depth"
LIDAR_PC_TOPIC = "/cloud"
CAMERA_INFO_TOPIC = "/camera_info"
GPS_TOPIC = "/gps"
VIS_MARKER_TOPIC = "/gps_vis_marker"
OUTPUT_MAP_TOPIC = "/projected"

#Insertion 
SIM_IMAGE = "/insertion/rendered_image"
SIM_DEPTH = "/insertion/rendered_depth"
SIM_OBSTACLE_POSES = "/insertion/obstacle_poses"

SIM_LATENCY = "/insertion/latency"

#IL
IL_OUTPUT = "/il/pred_brake"
IL_LATENCY = "/il/latency"

#Modular
MODULAR_OUTPUT = "/modular/pred_speed"
MODULAR_LATENCY = "/modular/latency"

# Frame Names
WORLD_FRAME = "world"
GPS_FRAME = "gps"
LIDAR_FRAME = "velodyne_lidar"
CAMERA_FRAME = "zed2_camera"

# Static Transforms
T_lidar_rgb = np.array([
    [0, -1, 0, 0.2],
    [0, 0, -1, -0.19],
    [1, 0, 0, -0.15],
    [0, 0, 0, 1]
])
# base_T_depth_rgb = np.array([
#     [0, -1, 0, 0.],
#     [0, 0, -1, 0],
#     [1, 0, 0, 0],
#     [0, 0, 0, 1]
# ])
# T_depth_rgb = np.array([[ 0.99833134,  0.0419498 ,  0.03968317,  0.08546855],
#        [-0.03715544,  0.99270851, -0.11467034,  0.11303873],
#        [-0.04420422,  0.11300455,  0.99261068, -0.07585925],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]]) @ base_T_depth_rgb

