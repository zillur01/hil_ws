#!/usr/bin/env python3
import numpy as np
import rospy
import message_filters
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, Image
from novatel_gps_msgs.msg import Inspva
from pacmod_msgs.msg import PacmodCmd, VehicleSpeedRpt
from cv_bridge import CvBridge
from hil.utils import *
from hil.tf_funcs import get_gps_tf
from hil.globals import *
from PIL import Image as pil_img

import rospkg, os, sys, time
rospack = rospkg.RosPack()
il_lib_path = os.path.join(rospack.get_path('hil'), 'src', 'HIL_imitation_learning')
sys.path.append(il_lib_path)
os.chdir(il_lib_path)

from HIL_imitation_learning.ros_main import IL_Agent, save_frames_as_gif, visualize_data
from HIL_imitation_learning.ros_main import il_config

FRAME_RATE = 10
k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync
BRAKE_THRESHOLD = 0.05
CRUISE_SPEED = 3.0 # 1.5
k_brake_gain = 1

class ilAgentNode:
    def __init__(self):
        rospy.init_node('saveData', anonymous=True)

        # IL Agent
        self.agent = IL_Agent(True)
        self.gif_frames = []
        self.start_time = None
        self.last_time = None
        
        # Synced topics
        self.image_sub= rospy.Subscriber(RGB_TOPIC,Image, callback=self.im_gps_cb)
        self.inspva_sub = rospy.Subscriber(GPS_TOPIC, Inspva, callback=self.gps_cb)

        self.bridge = CvBridge()

        # Pacmod Topics
        # self.pacmod_accel_sub = rospy.Subscriber("/pacmod/as_rx/accel_cmd", PacmodCmd, callback=self.accel_cb)
        self.pacmod_brake_sub = rospy.Subscriber("/pacmod/as_rx/brake_cmd", PacmodCmd, callback=self.brake_cb)
        # self.pacmod_steer_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, callback=self.steer_cb)
        # self.pacmod_speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, callback=self.speed_cb)
        
        # Predicted Speed Publisher
        self.pred_speed_pb = rospy.Publisher(IL_OUTPUT, PacmodCmd, queue_size=10)
        self.latency_pub = rospy.Publisher(IL_LATENCY, Float64, queue_size=10)
        self.desired_speed_pb = rospy.Publisher("/il/desired_speed", VehicleSpeedRpt, queue_size=10)

        self.count = 0
        
        self.last_cached_time_s = None
        self.latencies = [] # diff between last_cached_time_s and last_time every time a frame is saved
        self.is_running_prediction = False
        self.cur_speed = 0
        print("Node is Ready")
    # def accel_cb(self, cmd_msg: PacmodCmd):
    #     pass
    def brake_cb(self, cmd_msg: PacmodCmd):
        if not self.is_running_prediction and self.start_time:
            stamp = np.float32(rospy.Time.now().to_sec()-self.start_time)
            self.agent.add_brake_data(
                stamp,
                cmd_msg.f64_cmd
            )
    # def steer_cb(self, steer_msg: SystemRptFloat):
    #     self.agent.add_steer_data(
    #         steer_msg.header.stamp.to_nsec(),
    #         steer_msg.manual_input
    #     )
    # def speed_cb(self, speed_msg: VehicleSpeedRpt):
        # pass
    def gps_cb(self, gps_msg:Inspva):
        if not self.is_running_prediction and self.start_time:
            gps_stamp = np.float32(gps_msg.header.stamp.to_sec() - self.start_time)
            if len(self.agent.R_timestamp_cache)==0 or gps_stamp > self.agent.R_timestamp_cache[-1]:
                tot_speed = float(np.sqrt(np.square(gps_msg.east_velocity) + np.square(gps_msg.north_velocity)))
                self.cur_speed = tot_speed
                (t,Q) = get_gps_tf(gps_msg)
                pose = tf_to_transform(t, Q)
                
                self.agent.add_speed_data(gps_stamp, tot_speed)
                self.agent.add_pose_data(gps_stamp, pose)

    def im_gps_cb(self,img_msg: Image):
        if not self.is_running_prediction and self.start_time:
            stamp = np.float32(img_msg.header.stamp.to_sec() - self.start_time)
            cv2_rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
            cur_image = pil_img.fromarray(cv2_rgb_image)
            self.agent.add_img_data(stamp, cur_image)

    def publish_pred_brake(self, brake_val):
        brake_msg = PacmodCmd()
        brake_msg.header.stamp = rospy.Time.now()
        brake_msg.f64_cmd = brake_val
        self.pred_speed_pb.publish(brake_msg)

        desired_speed = CRUISE_SPEED
        if brake_val > BRAKE_THRESHOLD:
            # desired_speed = max(self.cur_speed - k_brake_gain * brake_val, 0)
            desired_speed = 0.0
        speed_msg = VehicleSpeedRpt()
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.vehicle_speed = desired_speed
        speed_msg.vehicle_speed_valid = True
        self.desired_speed_pb.publish(speed_msg)
    
    def on_shutdown(self):
        print(len(self.gif_frames))
        if len(self.gif_frames) > 0:
            save_frames_as_gif(self.gif_frames, 'rosbag_run')
            print("Saved Frames as GIF")
        # fps = self.count / (self.last_time - self.start_time)
        # print(f"fps: {fps}, count: {self.count}, total_time:{(self.last_time - self.start_time)}")
        # print(self.latencies)
        # print(f"Avg latency:{sum(self.latencies)/len(self.latencies)}")
    
    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        self.start_time = rospy.Time.now().to_sec()
        while self.start_time == 0.0:
            self.start_time = rospy.Time.now().to_sec()
        print("Ros Start_Time", self.start_time)
        frame_rate_start = time.time()
        r = rospy.Rate(FRAME_RATE)
        while not rospy.is_shutdown():
            # print(self.agent.curr_target_timestamp_barrier_count)
            self.is_running_prediction = True
            if len(self.agent.img_timestamp_cache) > 0:
                cache_time = self.agent.img_timestamp_cache[-1]
            output_data = self.agent.run_prediction()
            self.is_running_prediction = False
            if output_data is not None:
                curr_time = rospy.Time.now().to_sec() - self.start_time
                self.last_time = curr_time
                latency = abs(curr_time - cache_time)
                print("Cur_time:", curr_time, "Cur Latency", latency)
                self.latency_pub.publish(data=latency)
                self.publish_pred_brake(output_data['pred'])
                self.count += 1
                fps = self.count / (time.time() - frame_rate_start)
                print("fps", fps, "Frame_Count:", self.count, "Predicted Brake:", output_data['pred'])
                curr_gif_frame = visualize_data(output_data)
                self.gif_frames.append(curr_gif_frame)
            r.sleep()
        
        print("Shutting Down")
        sys.exit(0)
def main():
    node = ilAgentNode()
    node.run()

if __name__ == '__main__':
    main()
    