"""Single source of truth for the local ENU frame used across this repo.

Everything (agent pose, waypoints, rendered camera pose, virtual obstacle
positions) lives in a local East-North-Up frame in metres centred on
(ORIGIN_LAT, ORIGIN_LON). This is the same convention as gem_gnss_control's
pure_pursuit.py / record_waypoints.py (pymap3d.geodetic2enu with alt = 0), so
waypoints recorded there can be used here unchanged.

Pure python (numpy + pymap3d), no ROS imports, so it can be used by the
renderer and the agent standalone.
"""
import numpy as np
import pymap3d as pm

# Same origin as gem_gnss_control/config/e2_pp.yaml (origin_lat / origin_lon).
ORIGIN_LAT = 40.092857
ORIGIN_LON = -88.235992

# TODO(USER): ellipsoidal height (m) of the ground at the origin, as reported by
# /insnavgeod `height`. Only used to build the 3D camera pose for rendering
# (virtual objects rest on the ground relative to this). 200.0 is the value
# inherited from the original Novatel setup.
ORIGIN_ALT = 200.0

# TODO(USER): verify /insnavgeod `heading` is a compass heading (deg, clockwise
# from true north). pure_pursuit.py assumes so (yaw = 90 - heading). If it turns
# out to be offset or mirrored, fix it here and nothing else needs to change.
HEADING_OFFSET_DEG = 0.0


def compass_heading_deg(raw_heading_deg):
    """/insnavgeod heading -> compass heading (deg, clockwise from north, [0, 360))."""
    return np.mod(np.asarray(raw_heading_deg, dtype=float) + HEADING_OFFSET_DEG, 360.0)


def heading_to_yaw(heading_deg):
    """Compass heading (deg) -> ENU yaw (rad, counter-clockwise from east).
    Identical to PurePursuit.heading_to_yaw in gem_gnss_control."""
    return np.radians(90.0 - np.asarray(heading_deg, dtype=float))


def latlon_to_local(lat, lon):
    """(lat, lon) in degrees -> (x east, y north) in metres from the origin.
    Matches gem_gnss_control's wps_to_local_xy exactly (alt = 0)."""
    x, y, _ = pm.geodetic2enu(lat, lon, 0.0, ORIGIN_LAT, ORIGIN_LON, 0.0)
    return x, y


def geodetic_to_local(lat, lon, height):
    """(lat, lon, ellipsoidal height) -> (east, north, up) in metres from the origin."""
    return pm.geodetic2enu(lat, lon, height, ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT)


def local_to_latlon(x, y):
    """Inverse of latlon_to_local, handy for sanity checks against a map."""
    lat, lon, _ = pm.enu2geodetic(x, y, 0.0, ORIGIN_LAT, ORIGIN_LON, 0.0)
    return lat, lon


def load_local_waypoints(csv_path):
    """Load waypoints recorded in the local frame (header-less CSV: x, y, heading_deg,
    as written by gem_gnss_control's record_waypoints.py or save_waypoint_node.py).

    Returns an (n, 3) array of [x, y, heading_rad] where heading_rad is the compass
    heading in radians, which is what ModularAgent expects ("xza")."""
    wps = np.loadtxt(csv_path, delimiter=',', ndmin=2)
    xza = np.zeros((len(wps), 3))
    xza[:, 0] = wps[:, 0]
    xza[:, 1] = wps[:, 1]
    xza[:, 2] = np.radians(compass_heading_deg(wps[:, 2]))
    return xza
