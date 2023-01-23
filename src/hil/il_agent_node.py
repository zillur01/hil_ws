#!/usr/bin/env python3
from lib2to3.refactor import get_fixers_from_package
import numpy as np
import rospy
import message_filters
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, Image
from novatel_gps_msgs.msg import Inspva
from pacmod_msgs.msg import PacmodCmd, SystemRptFloat, VehicleSpeedRpt
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

from HIL_imitation_learning.main import IL_Agent, save_frames_as_gif, visualize_data, RosStreamSimulator
from HIL_imitation_learning.main import config as il_config




k_time_sync_queue_size = 10
k_time_sync_slop = 0.03 # allowed difference in seconds between messages to sync

class ilAgentNode:
    def __init__(self):
        rospy.init_node('saveData', anonymous=True)

        # IL Agent
        self.agent = IL_Agent(True)
        self.gif_frames = []

        
        # Synced topics
        self.image_sub= rospy.Subscriber(RGB_TOPIC,Image, callback=self.im_gps_cb)
        self.inspva_sub = rospy.Subscriber(GPS_TOPIC, Inspva, callback=self.gps_cb)

        # self.im_gps_sub = message_filters.ApproximateTimeSynchronizer(
        #     [self.image_sub, self.inspva_sub], 
        #     k_time_sync_queue_size, 
        #     k_time_sync_slop
        #     )
        # self.im_gps_sub.registerCallback(self.im_gps_cb)
        self.bridge = CvBridge()

        # Pacmod Topics
        # self.pacmod_accel_sub = rospy.Subscriber("/pacmod/as_rx/accel_cmd", PacmodCmd, callback=self.accel_cb)
        self.pacmod_brake_sub = rospy.Subscriber("/pacmod/as_rx/brake_cmd", PacmodCmd, callback=self.brake_cb)
        self.pacmod_steer_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, callback=self.steer_cb)
        # self.pacmod_speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, callback=self.speed_cb)
        
        # Predicted Speed Publisher
        self.pred_speed_pb = rospy.Publisher("il_pred_speed_pub", VehicleSpeedRpt, queue_size=10)
        self.count = 0
        self.start_time = time.time()
        self.last_time = time.time()
        self.last_cached_time_s = None
        self.latencies = [] # diff between last_cached_time_s and last_time every time a frame is saved
    # def accel_cb(self, cmd_msg: PacmodCmd):
    #     pass
    def brake_cb(self, cmd_msg: PacmodCmd):
        if self.agent.curr_target_timestamp is None:
            self.agent.set_curr_target_timestamp(rospy.Time.now().to_nsec())
        self.agent.add_brake_data(
            rospy.Time.now().to_nsec(),
            cmd_msg.f64_cmd
        )
    def steer_cb(self, steer_msg: SystemRptFloat):
        if self.agent.curr_target_timestamp is None:
            self.agent.set_curr_target_timestamp(rospy.Time.now().to_nsec())
        self.agent.add_steer_data(
            steer_msg.header.stamp.to_nsec(),
            steer_msg.manual_input
        )
    # def speed_cb(self, speed_msg: VehicleSpeedRpt):
        # pass
    def gps_cb(self, gps_msg:Inspva):
        if self.agent.curr_target_timestamp is None:
            self.agent.set_curr_target_timestamp(rospy.Time.now().to_nsec())
        gps_stamp = gps_msg.header.stamp.to_nsec()
        tot_speed = float(np.sqrt(np.square(gps_msg.east_velocity) + np.square(gps_msg.north_velocity)))
        (t,Q) = get_gps_tf(gps_msg)
        pose = tf_to_transform(t, Q)
        
        self.agent.add_speed_data(gps_stamp, tot_speed)
        self.agent.add_pose_data(gps_stamp, pose)

    def im_gps_cb(self,img_msg: Image):
        if self.agent.curr_target_timestamp is None:
            self.agent.set_curr_target_timestamp(rospy.Time.now().to_nsec())
        cv2_rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        cur_image = pil_img.fromarray(cv2_rgb_image)
        was_image_cached = self.agent.add_img_data(img_msg.header.stamp.to_nsec(), cur_image)
        if was_image_cached:
            self.last_cached_time_s = img_msg.header.stamp.to_sec()

    def publish_pred_speed(self, speed_val):
        speed_msg = VehicleSpeedRpt()
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.vehicle_speed_valid = True
        speed_msg.vehicle_speed = speed_val
        self.pred_speed_pb.publish(speed_msg)
    
    def on_shutdown(self):
        print(len(self.gif_frames))
        if len(self.gif_frames) > 0:
            save_frames_as_gif(self.gif_frames, 'rosbag_run')
            print("Saved Frames as GIF")
        fps = self.count / (self.last_time - self.start_time)
        print(f"fps: {fps}, count: {self.count}, total_time:{(self.last_time - self.start_time)}")
        print(self.latencies)
        print(f"Avg latency:{sum(self.latencies)/len(self.latencies)}")
    
    def run(self):
        rospy.on_shutdown(self.on_shutdown)

        self.start_time = rospy.Time.now().to_sec()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # print(self.agent.curr_target_timestamp_barrier_count)
            output_data = self.agent.run_prediction()
            if output_data is not None:
                self.last_time = rospy.Time.now().to_sec()
                self.latencies.append(abs(self.last_time-self.last_cached_time_s))
                self.publish_pred_speed(output_data['pred_speed'])
                print("Predicted")
                self.count += 1
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
    