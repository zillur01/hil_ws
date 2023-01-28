#!/usr/bin/env python3
import tf, tf2_ros
import numpy as np
import rospy
from hil import rviz_tools, tf_funcs, utils
from hil.globals import *
from std_msgs.msg import Float64
from novatel_gps_msgs.msg import Inspva
from geometry_msgs.msg import PointStamped
import message_filters 
import pandas as pd

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync
PRINT_RATE = 5

def dist2d(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

def avg(data):
    if len(data) == 0:
        return -1
    return sum(data)/len(data)
class metrics_node:
    def __init__(self, experiment_name, waypoint_csv_path):
        rospy.init_node('metrics', anonymous=True)
        # Setup waypoint trajectory info 
        df_gps = pd.read_csv(waypoint_path)
        gps_timestamp = df_gps["field.header.stamp"].tolist()
        gps_timestamp = np.array(gps_timestamp)
        gps_timestamp, ind = np.unique(gps_timestamp, return_index=True)
        lat = np.array(df_gps["field.latitude"].tolist())[ind]
        lon = np.array(df_gps["field.longitude"].tolist())[ind]
        azimuth = np.array(df_gps["field.azimuth"].tolist())[ind]

        # Subscribers
        self.inspva_sub = message_filters.Subscriber(GPS_TOPIC, Inspva)
        self.obstacle_pose_sub = message_filters.Subscriber(SIM_OBSTACLE_POINT, PointStamped)
        self.obstacle_dist_sub = message_filters.ApproximateTimeSynchronizer(
            [self.inspva_sub, self.obstacle_pose_sub], 
            k_time_sync_queue_size, 
            k_time_sync_slop
            )
        self.obstacle_dist_sub.registerCallback(self.obstacle_dist_cb)

        self.sim_latency_sub = rospy.Subscriber(SIM_LATENCY, Float64, callback=self.sim_latency_cb)
        self.il_latency_sub = rospy.Subscriber(IL_LATENCY, Float64, callback=self.il_latency_cb)
        self.mod_latency_sub = rospy.Subscriber(MODULAR_LATENCY, Float64, callback=self.mod_latency_cb)

        self.sim_lat = []
        self.il_lat = []
        self.mod_lat = []

    def sim_latency_cb(self, msg: Float64):
        self.sim_lat.append(msg.data)
    def il_latency_cb(self, msg:Float64):
        self.il_lat.append(msg.data)
    def mod_latency_cb(self, msg: Float64):
        self.mod_lat.append(msg.data)
    def obstacle_dist_cb(self, gps_msg: Inspva, pt_msg: PointStamped):
        (t,Q) = tf_funcs.get_gps_tf(gps_msg)
        T_gps = utils.tf_to_transform(t, Q)
        pt_ego = T_gps[:3,3]
        pt_obst = [pt_msg.point.x, pt_msg.point.y, pt_msg.point.z]
        print("Distance to Primary Obstacle:", dist2d(pt_ego, pt_obst), 
            "Ego Pt:", pt_ego, "Obstacle Pt:", pt_obst)
    def on_shutdown(self):
        print("Shutting down metrics")
    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        self.start_time = rospy.Time.now().to_sec()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print("Avg Sim Lat:", avg(self.sim_lat), "Avg IL Lat:", avg(self.il_lat), "Avg Mod Lat:", avg(self.mod_lat))
            r.sleep()

def main():
    node_name = "metrics"
    param_name = "waypoint_csv_path"
    waypoint_csv_path = rospy.get_param(f'{node_name}/{param_name}')
    param_name = "experiment_name"
    experiment_name = rospy.get_param(f'{node_name}/{param_name}')

    node = metrics_node(waypoint_csv_path, experiment_name)
    node.run()

if __name__ == '__main__':
    main()