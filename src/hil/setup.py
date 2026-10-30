## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hil', 'calibration', 'HIL_realtime_rendering', 'HIL_imitation_learning'],
    package_dir={'': 'src'},
)

setup(**setup_args)