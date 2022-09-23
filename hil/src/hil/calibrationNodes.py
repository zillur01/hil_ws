import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
from hil.utils import *
from hil.tf_funcs import *
from lidar_camera_calibration.msg import marker_6dof
from scipy.spatial.transform import Rotation as rot
k_time_sync_queue_size = 10
k_time_sync_slop = 1 # allowed difference in seconds between messages to sync

MARKER_OUTPUT_TOPIC = "lidar_camera_calibration_rt"

class arucoMappingNode:
    def __init__(self, CAMERA_INFO_TOPIC: str, RGB_TOPIC: str, desired_ids, marker_length):
        rospy.init_node('aruco_mapping', anonymous=True)
        self.camera_info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo)
        self.height, self.width = get_img_size(self.camera_info)
        self.desired_ids = desired_ids
        self.marker_length = marker_length
        self.bridge = CvBridge()
        self.curImage = None
        
        self.image_sub = rospy.Subscriber(RGB_TOPIC,Image, callback=self.process_im_callback)
        self.marker6dof_pub = rospy.Publisher(MARKER_OUTPUT_TOPIC, marker_6dof)

    def process_im_callback(self, img_msg: Image):
        self.curImage = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # Marker Detection
        gray = cv2.cvtColor(self.curImage, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # Filter aruco markers by id
        valid_inds = [i for i in range(len(ids)) if ids[i] in self.desired_ids]
        corners = [np.roll(corners[i], 1,1) for i in valid_inds]
        ids = np.array([ids[i] for i in valid_inds])
        result_image = self.curImage.copy()
        if len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                                corners, 
                                self.marker_length, 
                                get_camera_matrix(self.camera_info), 
                                get_dist_matrix(self.camera_info)
                            )
            
            # Publish marker info for lidar_camera_calibration
            output_marker_6dof = marker_6dof()
            output_marker_6dof.header.stamp = rospy.Time.now()
            output_marker_6dof.dof.data.clear()
            output_marker_6dof.num_of_markers = len(ids)
            adjustmentR = np.array([
                [1,0,0],
                [0,0,-1],
                [0,1,0]
            ])
            for i in range(len(rvecs)):
                output_marker_6dof.dof.data.append(ids[i])
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                rTemp = rot.from_rotvec(rvec)
                rvec = rot.from_matrix(rTemp.as_matrix()@ adjustmentR)
                rvec = rvec.as_rotvec()
                # tvec = adjustmentR @ tvec
                for j in tvec:
                    output_marker_6dof.dof.data.append(float(j))
                for j in rvec:
                    output_marker_6dof.dof.data.append(float(j))
                result_image = aruco.drawAxis(
                                result_image, 
                                get_camera_matrix(self.camera_info), 
                                get_dist_matrix(self.camera_info),
                                rvec, tvec, self.marker_length)

            self.marker6dof_pub.publish(output_marker_6dof)
            
            # Draw axes and bounds on image
            result_image = aruco.drawDetectedMarkers(result_image, corners, ids)

        cv2.imshow("Final Image window", result_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
