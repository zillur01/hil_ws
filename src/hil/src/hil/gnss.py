"""Latest-value GNSS/INS + vehicle speed state for the Septentrio/PACMod car (ROS 2).

Replaces the old Novatel `Inspva` subscription. Sources:
  - /navsatfix                    sensor_msgs/NavSatFix             lat, lon (deg)
  - /insnavgeod                   septentrio_gnss_driver/INSNavGeod heading/pitch/roll (deg), height (m), ve/vn (m/s)
  - /pacmod/vehicle_speed_rpt     pacmod2_msgs/VehicleSpeedRpt      vehicle_speed (m/s)

The three topics come in at different rates, so nodes sync only image + depth with
message_filters and read the latest GNSS values with `snapshot()`.
"""
import threading
from collections import namedtuple

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
from pacmod2_msgs.msg import VehicleSpeedRpt

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
    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._fix = None      # (lat, lon, stamp_s)
        self._ins = None      # (height, heading, pitch, roll, ve, vn)
        self._speed = None
        self._last_rx = {'fix': None, 'ins': None, 'speed': None}  # node clock time received, s
        # Best-effort subscriptions can receive from both reliable and best-effort publishers,
        # so this works whichever QoS the Septentrio / PACMod drivers use.
        node.create_subscription(NavSatFix, NAVSATFIX_TOPIC, self._fix_cb, qos_profile_sensor_data)
        node.create_subscription(INSNavGeod, INSNAVGEOD_TOPIC, self._ins_cb, qos_profile_sensor_data)
        node.create_subscription(VehicleSpeedRpt, PACMOD_SPEED_TOPIC, self._speed_cb, qos_profile_sensor_data)

    def _now_s(self):
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _fix_cb(self, msg):
        with self._lock:
            self._fix = (msg.latitude, msg.longitude, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            self._last_rx['fix'] = self._now_s()

    def _ins_cb(self, msg):
        with self._lock:
            # NOTE: attitude/velocity are only filled if the driver enables those
            # INSNavGeod sub-blocks (see `sb_list`); otherwise they read 0.
            self._ins = (msg.height, msg.heading, msg.pitch, msg.roll, msg.ve, msg.vn)
            self._last_rx['ins'] = self._now_s()

    def _speed_cb(self, msg):
        with self._lock:
            self._speed = msg.vehicle_speed
            self._last_rx['speed'] = self._now_s()

    def snapshot(self, max_age_s=1.0, need_speed=True):
        """Latest values as a GnssSample, or None until the required topics have been
        received (or if any has gone stale for more than max_age_s).

        need_speed=False: pose-only consumers (e.g. the insertion node) don't wait for
        /pacmod/vehicle_speed_rpt; `speed` is then 0.0 if it was never received."""
        with self._lock:
            required = ['fix', 'ins'] + (['speed'] if need_speed else [])
            if any(self._last_rx[k] is None for k in required):
                return None
            now = self._now_s()
            if max_age_s is not None and any(now - self._last_rx[k] > max_age_s for k in required):
                return None
            lat, lon, stamp = self._fix
            height, heading, pitch, roll, ve, vn = self._ins
            speed = self._speed if self._speed is not None else 0.0
            return GnssSample(lat, lon, height, heading, pitch, roll, ve, vn, speed, stamp)
