"""Latest-value GNSS/INS + vehicle speed state for the Septentrio/PACMod car.

Replaces the old Novatel `Inspva` subscription. Sources:
  - /navsatfix                    sensor_msgs/NavSatFix             lat, lon (deg)
  - /insnavgeod                   septentrio_gnss_driver/INSNavGeod heading/pitch/roll (deg), height (m), ve/vn (m/s)
  - /pacmod/vehicle_speed_rpt     pacmod VehicleSpeedRpt            vehicle_speed (m/s)

The three topics come in at different rates, so nodes sync only image + depth with
message_filters and read the latest GNSS values with `snapshot()`.
"""
import threading
from collections import namedtuple

import rospy
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
# TODO(USER): the ROS2 car publishes pacmod2_msgs/VehicleSpeedRpt, which is what a bag
# converted from ROS2 contains. The old ROS1 pacmod_msgs is only a fallback.
try:
    from pacmod2_msgs.msg import VehicleSpeedRpt
except ImportError:
    from pacmod_msgs.msg import VehicleSpeedRpt

from hil.globals import NAVSATFIX_TOPIC, INSNAVGEOD_TOPIC, PACMOD_SPEED_TOPIC

GnssSample = namedtuple('GnssSample', [
    'lat', 'lon',            # deg (NavSatFix)
    'height',                # m, ellipsoidal (INSNavGeod)
    'heading',               # deg, raw INSNavGeod heading
    'pitch', 'roll',         # deg (INSNavGeod)
    've', 'vn',              # m/s (INSNavGeod), only used for data logging
    'speed',                 # m/s (PACMod vehicle_speed)
    'stamp',                 # s, NavSatFix header stamp
])


class GnssState:
    def __init__(self):
        self._lock = threading.Lock()
        self._fix = None      # (lat, lon, stamp_s)
        self._ins = None      # (height, heading, pitch, roll, ve, vn)
        self._speed = None
        self._last_rx = {'fix': None, 'ins': None, 'speed': None}  # ros time received, s
        rospy.Subscriber(NAVSATFIX_TOPIC, NavSatFix, self._fix_cb, queue_size=1)
        rospy.Subscriber(INSNAVGEOD_TOPIC, INSNavGeod, self._ins_cb, queue_size=1)
        rospy.Subscriber(PACMOD_SPEED_TOPIC, VehicleSpeedRpt, self._speed_cb, queue_size=1)

    def _fix_cb(self, msg):
        with self._lock:
            self._fix = (msg.latitude, msg.longitude, msg.header.stamp.to_sec())
            self._last_rx['fix'] = rospy.Time.now().to_sec()

    def _ins_cb(self, msg):
        with self._lock:
            # NOTE: attitude/velocity are only filled if the driver enables those
            # INSNavGeod sub-blocks (see `sb_list`); otherwise they read 0.
            self._ins = (msg.height, msg.heading, msg.pitch, msg.roll, msg.ve, msg.vn)
            self._last_rx['ins'] = rospy.Time.now().to_sec()

    def _speed_cb(self, msg):
        with self._lock:
            self._speed = msg.vehicle_speed
            self._last_rx['speed'] = rospy.Time.now().to_sec()

    def snapshot(self, max_age_s=1.0):
        """Latest values as a GnssSample, or None until all three topics have been
        received (or if any has gone stale for more than max_age_s)."""
        with self._lock:
            if self._fix is None or self._ins is None or self._speed is None:
                return None
            now = rospy.Time.now().to_sec()
            if max_age_s is not None and any(now - t > max_age_s for t in self._last_rx.values()):
                return None
            lat, lon, stamp = self._fix
            height, heading, pitch, roll, ve, vn = self._ins
            return GnssSample(lat, lon, height, heading, pitch, roll, ve, vn, self._speed, stamp)
