#!/usr/bin/env python3
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy
from cv_bridge import CvBridge
import cv2
from hil_common.utils import *
from hil_common.globals import *
from hil_common.gnss import GnssState
import rospkg
import os, sys, time
import csv  
from pacmod_msgs.msg import PacmodCmd, SystemRptFloat, VehicleSpeedRpt
rospack = rospkg.RosPack()

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class saveDataNode:
    def __init__(self, output_dir, extracted_bag_name):
        rospy.init_node('saveData', anonymous=True)
        # self.fileDir = os.path.join(rospack.get_path('hil'), 'data', 'extracted_bags', extracted_bag_name+"3")
        print("inputs:", output_dir, extracted_bag_name)
        self.extracted_dir = output_dir+"_extracted"
        self.fileDir = os.path.join(self.extracted_dir, extracted_bag_name)
        print("FileDir:", self.fileDir)
        self.csv_path = os.path.join(self.fileDir, 'synced.csv')
        for _path in [self.fileDir]:
            if not os.path.exists(_path):
                os.makedirs(_path)

        self.bridge = CvBridge()
        
        # Synced topics
        self.pc_sub = message_filters.Subscriber(LIDAR_PC_TOPIC, PointCloud2)
        self.image_sub= message_filters.Subscriber(RGB_TOPIC,Image)
        self.depth_sub= message_filters.Subscriber(DEPTH_IM_TOPIC,Image)
        self.gnss = GnssState()

        self.pc_im_sub = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.pc_im_sub.registerCallback(self.pc_im_callback)
        self.iter = 0

        # Pacmod Topics
        # TODO(USER): IL-only. These are the old ROS1 PACMod topic names/types; the ROS2 car
        # (pacmod2_msgs) uses different ones (e.g. /pacmod/accel_cmd, /pacmod/vehicle_speed_rpt).
        self.pacmod_accel_seq_sub = rospy.Subscriber("/pacmod/as_rx/accel_cmd", PacmodCmd, callback=self.accel_seq_cb)
        self.cur_accel_seq = None
        self.pacmod_brake_seq_sub = rospy.Subscriber("/pacmod/as_rx/brake_cmd", PacmodCmd, callback=self.brake_seq_cb)
        self.cur_brake_seq = None

        self.pacmod_accel_seq_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, callback=self.steer_cb)
        self.cur_steer_seq = None
        self.pacmod_brake_seq_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, callback=self.speed_cb)
        self.cur_speed_seq = None
        self.last_save_time = None

        self.fields = ["image_stamp", "gps_stamp", "depth_stamp", 
                        "accel_seqnum", "brake_seqnum", "steer_seqnum", "speed_seqnum"]
        self.rows = []
    def accel_seq_cb(self, cmd_msg: PacmodCmd):
        self.cur_accel_seq = cmd_msg.header.seq
    def brake_seq_cb(self, cmd_msg: PacmodCmd):
        self.cur_brake_seq = cmd_msg.header.seq
    def steer_cb(self, steer_msg: SystemRptFloat):
        self.cur_steer_seq = steer_msg.header.seq
    def speed_cb(self, speed_msg: VehicleSpeedRpt):
        self.cur_speed_seq = speed_msg.header.seq

    def pc_im_callback(self,img_msg: Image, depth_msg: Image):
        gps_msg = self.gnss.snapshot()
        if gps_msg is None:
            return
        if self.cur_brake_seq != None and self.cur_accel_seq != None:
            img_t = img_msg.header.stamp.to_nsec()
            gps_t = int(gps_msg.stamp * 1e9)
            depth_t = depth_msg.header.stamp.to_nsec()
            synced_row = [
                img_t,
                gps_t,
                depth_t,
                self.cur_accel_seq,
                self.cur_brake_seq,
                self.cur_steer_seq,
                self.cur_speed_seq,                
            ]
            self.rows.append(synced_row)
            self.last_save_time = time.time()
            self.iter += 1
    def on_shutdown(self):
        # writing to csv file 
        print("Writing csv")
        with open(self.csv_path, 'w') as csvfile: 
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 
                
            # writing the fields 
            csvwriter.writerow(self.fields) 
                
            # writing the data rows 
            csvwriter.writerows(self.rows)
    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        while not rospy.is_shutdown():
            if self.last_save_time != None and time.time()-self.last_save_time > 5:
                break
            time.sleep(1)
        print("Shutting Down")
        sys.exit(0)
def main():
    node_name = "save_data_node"
    param_name = "input_bag_path"
    bag_filepath = rospy.get_param(f'{node_name}/{param_name}')
    bag_filename = os.path.splitext(os.path.basename(bag_filepath))[0]
    extracted_dir = os.path.dirname(bag_filepath)
    print(extracted_dir, bag_filename)
    node = saveDataNode(extracted_dir, bag_filename)
    node.run()

if __name__ == '__main__':
    main()
