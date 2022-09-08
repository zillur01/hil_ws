# HIL_Research
## Getting Started
1. Setup your catkin workspace: `mkdir -p ~/catkin_ws/src` and clone this repo into the `src` subfolder
2. Run `sh setup_scripts/ros_noetic_install.sh` to install ros noetic
3. Run `sh setup_scripts/hdl_slam_install.sh` to install the hdl_slam dependency
4. TODO: Add rosdep dependencies to package.xml
5. `cd ~/catkin_ws` and `catkin_make` to build the packages in your workspace
6. `source devel/setup.bash` so ros know where to find your custom packages
7. TODO: Download some demo rosbags 
8. Try out some of `HIL` package launch files!

## Helpful tools
- Catkin_tools: https://catkin-tools.readthedocs.io/en/latest/installing.html
## Resources
- http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
- http://wiki.ros.org/rosbag/Commandline#record
- https://docs.docker.com/engine/install/ubuntu/
- https://docs.docker.com/compose/install/
- https://github.com/tkunic/ros-docker-simple
