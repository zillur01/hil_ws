#!/usr/bin/env python3
import tf, tf2_ros
import numpy as np
import rospy
from hil_common import rviz_tools, tf_funcs, utils
from hil_common.globals import *
from std_msgs.msg import Float64
from hil_common.gnss import GnssState
from hil_common.origin import load_local_waypoints
from geometry_msgs.msg import PoseArray, Pose
import message_filters 
from pacmod_msgs.msg import PacmodCmd

k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync
PRINT_RATE = 5
GOAL_REACHING_THRESH_M = 1

def dist2d(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

def avg(data):
    if len(data) == 0:
        return -1
    return sum(data)/len(data)
class metrics_node:
    def __init__(self, waypoint_csv_path, experiment_name):
        rospy.init_node('metrics', anonymous=True)
        # Waypoints recorded in the local ENU frame (x, y, heading_deg, header-less CSV)
        wps = load_local_waypoints(waypoint_csv_path)
        self.start_pt = wps[0]
        self.goal_pt = wps[-1]
        # Subscribers
        self.gnss = GnssState()
        self.obstacle_pose_sub = rospy.Subscriber(SIM_OBSTACLE_POSES, PoseArray, callback=self.obstacle_dist_cb)
        self.pt_ego = None
        self.pt_obst = None 

        self.sim_latency_sub = rospy.Subscriber(SIM_LATENCY, Float64, callback=self.sim_latency_cb)
        self.il_latency_sub = rospy.Subscriber(IL_LATENCY, Float64, callback=self.il_latency_cb)
        self.mod_latency_sub = rospy.Subscriber(MODULAR_LATENCY, Float64, callback=self.mod_latency_cb)
        self.il_pred_brake_sub = rospy.Subscriber(IL_OUTPUT, PacmodCmd, callback=self.il_pred_brake_cb)

        self.sim_lat = []
        self.il_lat = []
        self.mod_lat = []
        self.pred_brake = 0.0
        self.start_goal_time = None
        self.end_goal_time = None
        self.first_brake_pressed = False

    def il_pred_brake_cb(self, msg: PacmodCmd):
        self.pred_brake = msg.f64_cmd
    def sim_latency_cb(self, msg: Float64):
        self.sim_lat.append(msg.data)
    def il_latency_cb(self, msg:Float64):
        self.il_lat.append(msg.data)
    def mod_latency_cb(self, msg: Float64):
        self.mod_lat.append(msg.data)
    def obstacle_dist_cb(self, pose_msg: PoseArray):
        gps_msg = self.gnss.snapshot()
        if gps_msg is None:
            return
        cur_spd = float(gps_msg.speed) # PACMod vehicle speed
        if cur_spd > 0.1 and self.start_goal_time is None:
            self.start_goal_time = rospy.Time.now().to_sec()
        (t,Q) = tf_funcs.get_gps_tf(gps_msg)
        T_gps = utils.tf_to_transform(t, Q)
        self.pt_ego = T_gps[:3,3]
        pt1 = pose_msg.poses[0].position
        self.pt_obst = [pt1.x, pt1.y, pt1.z]
        # print("Distance to Primary Obstacle:", dist2d(pt_ego, pt_obst), 
        #     "Ego Pt:", pt_ego, "Obstacle Pt:", pt_obst)
    def on_shutdown(self):
        print("Shutting down metrics")
    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.start_goal_time is not None:
                if dist2d(self.pt_ego, self.goal_pt) < GOAL_REACHING_THRESH_M and self.end_goal_time is None:
                    self.end_goal_time = rospy.Time.now().to_sec() 
                    print("Reached Goal in", self.end_goal_time-self.start_goal_time)
                    print("Avg Sim Lat:", avg(self.sim_lat), "Avg IL Lat:", avg(self.il_lat), "Avg Mod Lat:", avg(self.mod_lat))

                if self.pred_brake > 0.4 and not self.first_brake_pressed:
                    print("First Brake Pressed, obst dist:", dist2d(self.pt_ego, self.pt_obst))
                    t2goal = rospy.Time.now().to_sec() - self.start_goal_time
                    print("t2Goal:", t2goal, "Avg Sim Lat:", avg(self.sim_lat), "Avg IL Lat:", avg(self.il_lat), "Avg Mod Lat:", avg(self.mod_lat))
                    self.first_brake_pressed = True
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