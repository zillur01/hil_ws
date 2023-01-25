import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
import numpy as np
from PIL import Image as pil_img
from tqdm import tqdm

import os, argparse

class il_data:
    def __init__(self, rosbag_path, save_dir):
        self.rosbag_path = rosbag_path
        self.fileDir = save_dir
        # Topics
        self.pc_top = "/lidar1/velodyne_points"
        self.gps_top = "/novatel/inspva"
        # Pacmod topics
        self.accel_top = "/pacmod/as_rx/accel_cmd"
        self.brake_top = "/pacmod/as_rx/brake_cmd"
        self.steer_top = "/pacmod/parsed_tx/steer_rpt"
        self.speed_top = "/pacmod/parsed_tx/vehicle_speed_rpt"
        # Zed topics
        self.depth_top = "/zed2/zed_node/depth/depth_registered"
        self.img_l_top = "/zed2/zed_node/left/image_rect_color"
        self.img_r_top = "/zed2/zed_node/right/image_rect_color"

        self.topics = [
            self.pc_top, 
            self.gps_top, self.accel_top, self.brake_top, self.steer_top, self.speed_top, 
            self.depth_top, self.img_l_top, self.img_r_top ]
        topic_save_paths = [
            "pc", 
            "gps.csv", "accel.csv", "brake.csv", "steer.csv", "speed.csv",
            "depth", "img_l", "img_r"
        ]
        topic_save_paths = [os.path.join(self.fileDir,subpath) for subpath in topic_save_paths]
        self.top2path = {}
        for i in range(len(self.topics)):
            self.top2path[self.topics[i]] = topic_save_paths[i]

        for _path in topic_save_paths:
            if ".csv" not in _path and not os.path.exists(_path):
                os.makedirs(_path)

        self.csv_topics = [self.gps_top, self.accel_top, self.brake_top, self.steer_top, self.speed_top]
        self.dir_topics = [self.pc_top, self.img_l_top, self.img_r_top, self.depth_top]
        self.bag_info_path = os.path.join(self.fileDir, "bag_info.txt")
        self.bridge = CvBridge()

    def save_image(self, msg, path, depth=False):
        """ Saves the sensor_msgs/Image msg input to the file path
            Set depth=true if you want to process the topic data as a depth image """
        timestamp = msg.header.stamp.to_nsec()
        if depth:
            mode_depth='F'
            depth_map = np.asarray(pil_img.frombytes(mode_depth, (msg.width, msg.height), msg.data))
            np.save(os.path.join(path, f'{timestamp}.npy'), depth_map)
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(path,f'{timestamp}.png'), cv_image)

    def save_pc(self, msg, path):
        timestamp = msg.header.stamp.to_nsec()
        pts_depth = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        np.save(os.path.join(path, f'{timestamp}.npy'), pts_depth)
    def floattime_to_nsec(self, t):
        secs, nsecs = [int(i) for i in str(t).split('.')]
        return secs * int(1e9) + nsecs
    def save_bag_info(self, start, end):
        with open(self.bag_info_path, 'w') as f:
            f.write(f"{start} {end}")
    def run_extraction(self):
        if not os.path.exists(self.bag_info_path):
            rostopic_cmd = "rostopic echo -b {} -p {} > {}"
            for topic_name in self.csv_topics:
                os.system(rostopic_cmd.format(self.rosbag_path, topic_name, self.top2path[topic_name]))
            print("Saved CSV topics")
            bag = rosbag.Bag(rosbag_path)
            start, end = self.floattime_to_nsec(bag.get_start_time()), self.floattime_to_nsec(bag.get_end_time())
            self.save_bag_info(start, end)
            img_gen = tqdm(enumerate(bag.read_messages(topics=self.dir_topics)))
            for n, (topic,msg,t) in img_gen:
                if topic == self.pc_top:
                    self.save_pc(msg, self.top2path[topic])
                elif topic == self.img_l_top or topic==self.img_r_top:
                    self.save_image(msg, self.top2path[topic])
                elif topic == self.depth_top:
                    self.save_image(msg, self.top2path[topic], depth=True)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS Bag image extraction and sync tool")
    parser.add_argument('-d', '--bag_dir',
            default="/media/bhargav/03E7-EC27/cone_bags",
            type=str, required=False, help='Path to rosbag file')
    args = parser.parse_args()
    for filename in os.listdir(args.bag_dir):
        rosbag_path = os.path.join(args.bag_dir, filename)
        bag_filename = os.path.splitext(os.path.basename(rosbag_path))[0]
        extracted_dir = os.path.dirname(rosbag_path) + "_extracted"
        # extracted_dir = "/home/yshen47/catkin_ws/src/HIL_Research/scripts/today_bags"
        save_dir = os.path.join(extracted_dir, bag_filename)
        ild = il_data(rosbag_path, save_dir)
        if not os.path.exists(ild.bag_info_path):
            # Get Synced Data
            os.system(f"roslaunch hil save_data_IL.launch input_bag_path:={rosbag_path}")
            print(f"Processing bag: {rosbag_path}")
            
            ild.run_extraction()
            print("Finished Extraction")
        else:
            print(f"skipping{rosbag_path}")
