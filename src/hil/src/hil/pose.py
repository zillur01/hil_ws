"""GNSS pose -> camera pose in the local ENU frame (pure numpy/scipy, no ROS imports)."""
import numpy as np
from scipy.spatial.transform import Rotation as R

from hil import origin


def gnss_to_cam2world(gnss: 'GnssSample') -> np.ndarray:
    """4x4 transform of the car/camera in local ENU world coordinates
    (east, north, up in metres from hil.origin.ORIGIN_LAT/LON; up is relative to origin.ORIGIN_ALT).

    The GNSS pose is used directly as the camera pose (no mount offset), as in the original code.
    Args:
        gnss: hil.gnss.GnssSample (lat/lon from NavSatFix, rest from INSNavGeod)
    """
    east, north, up = origin.geodetic_to_local(gnss.lat, gnss.lon, gnss.height)
    # Checked on the car: nose up -> pitch increases, right side up -> roll decreases, which is
    # the rotation direction used below (x = right, y = forward, z = up), so no sign flips.
    # Both read about -1.3 deg on level-ish ground (mount bias or slope, left uncorrected).
    # They only affect the rendered camera pose (insertion), not the modular agent.
    pitch, roll, azimuth = gnss.pitch, gnss.roll, float(origin.compass_heading_deg(gnss.heading))
    cam2world = np.eye(4)
    cam2world[:3, :3] = R.from_euler('xyz', [pitch, roll, -azimuth], degrees=True).as_matrix()
    cam2world[:3, 3] = [east, north, up]
    return cam2world


def get_gps_tf(gnss: 'GnssSample') -> 'tuple[np.ndarray, np.ndarray]':
    """(translation, quaternion xyzw) of gnss_to_cam2world()."""
    T = gnss_to_cam2world(gnss)
    return T[:3, 3], R.from_matrix(T[:3, :3]).as_quat()
