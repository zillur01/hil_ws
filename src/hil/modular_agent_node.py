#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, Image
from pacmod_msgs.msg import VehicleSpeedRpt
from cv_bridge import CvBridge
import cv2
from PIL import Image as pil_img
import pandas as pd
import hil.utils as ut
from hil.tf_funcs import *
from hil.globals import *
import tf, message_filters
import os, time, sys
import rospkg
import matplotlib.pyplot as plt
rospack = rospkg.RosPack()
rendering_lib_path = os.path.join(rospack.get_path('hil'), 'src', 'HIL_modular_autonomy')
sys.path.append(rendering_lib_path)
os.chdir(rendering_lib_path)

import torch.nn.functional as F
import torch

from HIL_modular_autonomy.agent import ModularAgent, dist2d, lla_to_xza

DEBUG = True
FRAME_RATE = 8

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class modularAgentNode:
    """ Aggregates a global 3D colored pointcloud map
        as the car moves around the environment"""
    def __init__(self, waypoint_path):
        rospy.init_node('modular_agent', anonymous=True)
        
        # Setup waypoint trajectory info 
        df_gps = pd.read_csv(waypoint_path)
        gps_timestamp = df_gps["field.header.stamp"].tolist()
        gps_timestamp = np.array(gps_timestamp)
        gps_timestamp, ind = np.unique(gps_timestamp, return_index=True)
        lat = np.array(df_gps["field.latitude"].tolist())[ind]
        lon = np.array(df_gps["field.longitude"].tolist())[ind]
        azimuth = np.array(df_gps["field.azimuth"].tolist())[ind]
        
        # Planned xza waypoint trajectory
        self.xzas = np.zeros((len(ind), 3))
        for i in range(len(ind)):
            self.xzas[i] = lla_to_xza(lat[i], lon[i], azimuth[i])
        # Closest point in trajectory to our current position
        self.cur_xza_ind = 0

        self.agent = ModularAgent()

        self.bridge = CvBridge()
        self.cur_image = None
        self.cur_depth_map = None
        self.cur_image_timestamp_s = None
        self.cur_xza_pose = None
        self.cur_spd = None

        # Speed Publisher
        self.pred_speed_pb = rospy.Publisher(MODULAR_OUTPUT, VehicleSpeedRpt, queue_size=10)

        # Subscribers
        # self.image_sub = rospy.Subscriber(RGB_TOPIC, Image, callback=self.im_callback)
        self.image_sub = message_filters.Subscriber(RGB_TOPIC,Image)
        self.depth_sub = message_filters.Subscriber(DEPTH_IM_TOPIC, Image)
        self.inspva_sub = message_filters.Subscriber(GPS_TOPIC, Inspva)

        self.im_depth_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.inspva_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.im_depth_sub.registerCallback(self.im_depth_callback)

        self.latency_pub = rospy.Publisher(MODULAR_LATENCY, Float64, queue_size=10)

        self.start_time = 0
        self.n_prediction = 0 # number of times update_pred_step is called
        self.n_callback = 0 # numer of times im_depth_callback is called

    def im_depth_callback(self, img_msg: Image, depth_msg: Image, gps_msg: Inspva):
        """Sensor data(rgb, depth, pose) updated as synced messages are received"""
        # print("received data")
        self.cur_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        self.cur_image_timestamp_s = img_msg.header.stamp.to_sec()
        # cv2.imshow("res2", self.cur_image)
        # cv2.waitKey(1)
        mode_depth='F'
        orig_depth_map = np.asarray(pil_img.frombytes(mode_depth, (depth_msg.width, depth_msg.height), depth_msg.data))
        # plt.imshow(orig_depth_map)
        # plt.show()
        self.cur_depth_map = np.nan_to_num(orig_depth_map, nan=np.inf, posinf=np.inf, neginf=np.inf)
        self.cur_spd = float(np.sqrt(np.square(gps_msg.east_velocity) + np.square(gps_msg.north_velocity)))
        self.cur_xza_pose = lla_to_xza(gps_msg.latitude, gps_msg.longitude, gps_msg.azimuth)
        while   self.cur_xza_ind < len(self.xzas)-1 and \
                dist2d(self.cur_xza_pose[:2], self.xzas[self.cur_xza_ind]) > dist2d(self.cur_xza_pose[:2], self.xzas[self.cur_xza_ind+1]):
                self.cur_xza_ind += 1
        # print(self.cur_xza_ind)
        self.n_callback += 1

    def update_pred_step(self):
        xza_ind = self.cur_xza_ind
        input_img_stamp = self.cur_image_timestamp_s
        self.agent.update(self.cur_image, self.cur_depth_map, input_img_stamp, self.cur_xza_pose, self.cur_spd)
        dspd, dacc = self.agent.act(self.xzas[xza_ind+1:])
        print('frame %d | desired spd: %.4f, desired acc: %.4f' % (self.n_prediction, dspd, dacc))        

        latency = rospy.Time.now().to_sec() - input_img_stamp
        self.latency_pub.publish(data=latency)

        # publish speed message
        self.publish_pred_speed(dspd)
        self.n_prediction += 1
        if self.n_prediction > 2:
            cur_avg_runtime = (time.perf_counter() - self.start_time)/(self.n_prediction - 2)
            print(cur_avg_runtime, 's ==>', 1/cur_avg_runtime, 'fps')   

    def publish_pred_speed(self, speed_val):
        speed_msg = VehicleSpeedRpt()
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.vehicle_speed_valid = True
        speed_msg.vehicle_speed = speed_val
        self.pred_speed_pb.publish(speed_msg)

    def on_shutdown(self):
        avg_runtime = (time.perf_counter() - self.start_time)/(self.n_prediction - 2)
        print(avg_runtime, 's ==>', 1/avg_runtime, 'fps')
        print("Sensor Data n callback:", self.n_callback)

    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        print("Starting main modular autonomy loop")
        r = rospy.Rate(FRAME_RATE) # 10hz 
        while not rospy.is_shutdown():
            if self.n_prediction == 2:
                self.start_time = time.perf_counter()
            if  self.cur_image is not None and \
                self.cur_depth_map is not None and \
                self.cur_xza_pose is not None and \
                self.cur_spd is not None:
                print("predicting")
                self.update_pred_step()
            r.sleep()

def main():
    node_name = "modular_agent"
    param_name = "waypoint_csv_path"
    waypoint_csv_path = rospy.get_param(f'{node_name}/{param_name}')

    node = modularAgentNode(waypoint_csv_path)
    node.run()

if __name__ == '__main__':
    main()