from setuptools import setup
from glob import glob
import os

package_name = 'hil'

setup(
    name=package_name,
    version='0.0.0',
    # Only the `hil` library package is installed. The renderer (HIL_realtime_rendering) and the
    # agent (HIL_modular_autonomy) are run from the source tree, so build with
    # `colcon build --symlink-install`.
    package_dir={'': 'src'},
    packages=['hil'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # node scripts, runnable with `ros2 run hil <name>.py`
        (os.path.join('lib', package_name), ['modular_agent_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhargav Chandaka',
    maintainer_email='bhargav2900@gmail.com',
    description='Sim-on-Wheels HIL: virtual object insertion and modular autonomy',
    license='TODO',
)
