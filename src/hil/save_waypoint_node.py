#!/usr/bin/env python3
"""Records waypoints in the local ENU frame (hil_common/origin.py), same CSV format as
gem_gnss_control's record_waypoints.py: header-less rows of `x, y, heading_deg`.

Drive the route manually (or play a bag of it) with this node running. The output
file is what modular_agent_node / metrics_node read via `waypoint_csv_path`.
"""
import csv
import math
import os

import rospkg
import rospy

from hil_common.gnss import GnssState
from hil_common.origin import latlon_to_local

MIN_DISTANCE_M = 0.5  # only append a point after moving this far from the last one
RATE_HZ = 5.0


def main():
    rospy.init_node('save_waypoint_node')
    filename = rospy.get_param('~filename', 'recorded_track')
    out_dir = os.path.join(rospkg.RosPack().get_path('hil'), 'data', 'waypoints')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, filename if filename.endswith('.csv') else filename + '.csv')

    gnss = GnssState()
    last_xy = None
    n = 0
    rospy.loginfo(f"Recording waypoints to {out_path}")
    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            s = gnss.snapshot()
            if s is not None:
                x, y = latlon_to_local(s.lat, s.lon)
                if last_xy is None or math.hypot(x - last_xy[0], y - last_xy[1]) >= MIN_DISTANCE_M:
                    # raw INSNavGeod heading, like gem_gnss_control; origin.compass_heading_deg()
                    # is applied when the file is loaded.
                    writer.writerow([x, y, s.heading])
                    f.flush()
                    last_xy = (x, y)
                    n += 1
                    rospy.loginfo(f"[{n}] x={x:.3f} y={y:.3f} heading={s.heading:.2f}")
            rate.sleep()


if __name__ == '__main__':
    main()
