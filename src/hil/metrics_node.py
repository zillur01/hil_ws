#!/usr/bin/env python3
"""Experiment metrics: time to reach the goal (last waypoint), distance to the first virtual obstacle
when the IL agent first brakes, and average insertion / IL / modular latencies.

Parameters: `waypoint_csv_path` (local x, y, heading_deg CSV), `experiment_name`."""
import rclpy
from geometry_msgs.msg import PoseArray
from pacmod2_msgs.msg import SystemCmdFloat
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

from hil_common import tf_funcs, utils
from hil_common.globals import IL_LATENCY, IL_OUTPUT, MODULAR_LATENCY, SIM_LATENCY, SIM_OBSTACLE_POSES
from hil_common.gnss import GnssState
from hil_common.origin import load_local_waypoints

PRINT_RATE = 5
GOAL_REACHING_THRESH_M = 1


def dist2d(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

def avg(data):
    if len(data) == 0:
        return -1
    return sum(data)/len(data)


class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics')
        waypoint_csv_path = self.declare_parameter('waypoint_csv_path', '').value
        self.experiment_name = self.declare_parameter('experiment_name', 'test').value
        if not waypoint_csv_path:
            raise RuntimeError("Parameter 'waypoint_csv_path' is required (local x,y,heading_deg CSV)")
        # Waypoints recorded in the local ENU frame (x, y, heading_deg, header-less CSV)
        wps = load_local_waypoints(waypoint_csv_path)
        self.start_pt = wps[0]
        self.goal_pt = wps[-1]
        # Subscribers
        self.gnss = GnssState(self)
        self.create_subscription(PoseArray, SIM_OBSTACLE_POSES, self.obstacle_dist_cb, 10)
        self.pt_ego = None
        self.pt_obst = None

        self.create_subscription(Float64, SIM_LATENCY, self.sim_latency_cb, 10)
        self.create_subscription(Float64, IL_LATENCY, self.il_latency_cb, 10)
        self.create_subscription(Float64, MODULAR_LATENCY, self.mod_latency_cb, 10)
        self.create_subscription(SystemCmdFloat, IL_OUTPUT, self.il_pred_brake_cb, 10)

        self.sim_lat = []
        self.il_lat = []
        self.mod_lat = []
        self.pred_brake = 0.0
        self.start_goal_time = None
        self.end_goal_time = None
        self.first_brake_pressed = False
        self.timer = self.create_timer(0.1, self.step)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def il_pred_brake_cb(self, msg: SystemCmdFloat):
        self.pred_brake = msg.command
    def sim_latency_cb(self, msg: Float64):
        self.sim_lat.append(msg.data)
    def il_latency_cb(self, msg: Float64):
        self.il_lat.append(msg.data)
    def mod_latency_cb(self, msg: Float64):
        self.mod_lat.append(msg.data)

    def obstacle_dist_cb(self, pose_msg: PoseArray):
        gps_msg = self.gnss.snapshot()
        if gps_msg is None or not pose_msg.poses:
            return
        cur_spd = float(gps_msg.speed) # PACMod vehicle speed
        if cur_spd > 0.1 and self.start_goal_time is None:
            self.start_goal_time = self.now_s()
        (t,Q) = tf_funcs.get_gps_tf(gps_msg)
        T_gps = utils.tf_to_transform(t, Q)
        self.pt_ego = T_gps[:3,3]
        # obstacle poses are in the same local ENU frame as the GNSS pose
        pt1 = pose_msg.poses[0].position
        self.pt_obst = [pt1.x, pt1.y, pt1.z]

    def step(self):
        if self.start_goal_time is None:
            return
        if dist2d(self.pt_ego, self.goal_pt) < GOAL_REACHING_THRESH_M and self.end_goal_time is None:
            self.end_goal_time = self.now_s()
            print("Reached Goal in", self.end_goal_time-self.start_goal_time)
            print("Avg Sim Lat:", avg(self.sim_lat), "Avg IL Lat:", avg(self.il_lat), "Avg Mod Lat:", avg(self.mod_lat))

        if self.pred_brake > 0.4 and not self.first_brake_pressed:
            print("First Brake Pressed, obst dist:", dist2d(self.pt_ego, self.pt_obst))
            t2goal = self.now_s() - self.start_goal_time
            print("t2Goal:", t2goal, "Avg Sim Lat:", avg(self.sim_lat), "Avg IL Lat:", avg(self.il_lat), "Avg Mod Lat:", avg(self.mod_lat))
            self.first_brake_pressed = True


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("Shutting down metrics")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
