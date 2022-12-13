#!/usr/bin/env python3
import numpy as np
import rospy
# from gps_common.msg import GPSFix
from novatel_gps_msgs.msg import Inspva
import csv
import rospkg
import os
import utm 
import matplotlib.pyplot as plt

rospack = rospkg.RosPack()
k_node_name = 'save_waypoint_node'
k_time_sync_queue_size = 10

class saveDataNode:
    def __init__(self, filename, save_relative=False):
        rospy.init_node(k_node_name, anonymous=True)
        fileDir = os.path.join(rospack.get_path('hil'), 'data', 'waypoints')
        if not os.path.exists(fileDir):
            os.mkdir(fileDir)
        self.csv_file = os.path.join(fileDir,filename + ".csv")
        self.img_file = self.csv_file.split('.')[0] + ".png"

        self.gps_sub = rospy.Subscriber("/novatel/inspva", Inspva, callback=self.gps_callback)
    
        self.data_rows = []
        self.save_relative = save_relative
        self.first = False
        self.start_east = None
        self.start_north = None

    def gps_callback(self, msg: Inspva):
        if not self.first:
            utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
            self.start_east, self.start_north = utm_coord[0], utm_coord[1]
            self.first = True

        data = [msg.latitude, msg.longitude, msg.azimuth]

        if self.save_relative:
            utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
            east, north = utm_coord[0], utm_coord[1]
            data = [east-self.start_east, north-self.start_north, msg.azimuth]
        self.data_rows.append(data)

    def write_data(self):
        if not (input("Save data?").lower() == "y"):
            print("Not saving")
            return
        for file in [self.csv_file, self.img_file]:
            if(os.path.exists(file) and os.path.isfile(file)):
                os.remove(file)
        with open(self.csv_file, 'w') as f:
            csvwriter = csv.writer(f) 
            csvwriter.writerows(self.data_rows)
        xs = [i[0] for i in self.data_rows]
        ys = [i[1] for i in self.data_rows]
        fig = plt.figure()
        ax = fig.add_subplot()
        plt.plot(xs[0], ys[0], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")

        plt.plot(xs, ys)
        plt.xlabel("Latitude/Easting")
        plt.ylabel("Longitude/Northing")
        ax.set_aspect('equal', adjustable='box')

        plt.savefig(self.img_file, bbox_inches='tight')
        print("Wrote Data")
    def run(self):
        rospy.spin()

def main():
    param_name = "filename"
    save_filename = rospy.get_param(f'{k_node_name}/{param_name}')
    node = saveDataNode(save_filename)
    print("Starting node, use ctrl+c to stop saving gps data, and type 'y' after the promt if you want to write the saved data to file")
    node.run()
    node.write_data()
if __name__ == '__main__':
    main()
