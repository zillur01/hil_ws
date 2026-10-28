import numpy as np

SHADOW_CONTRAST = 1
CAR_MODEL_NAME = 'car-suv'
HUMAN_ANIMATION = 'male_running_still'
T_blender2opengl = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ np.array(
    [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

# Local ENU origin (lat/lon in deg, ellipsoidal height in m). Single source of truth is
# hil/origin.py; the literals below are only a fallback for running the renderer standalone
# (e.g. `python kitti360.py`) and must be kept in sync with it.
try:
    from hil.origin import ORIGIN_LAT as ORIGIN_LATITUDE, ORIGIN_LON as ORIGIN_LONGITUDE, ORIGIN_ALT as ORIGIN_HEIGHT
except ImportError:
    ORIGIN_LATITUDE = 40.092857
    ORIGIN_LONGITUDE = -88.235992
    ORIGIN_HEIGHT = 200.0  # TODO(USER): keep in sync with hil/origin.py ORIGIN_ALT
