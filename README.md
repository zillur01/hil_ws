# HIL_Research
## Data setup
- A box folder with recorded data and model files mirrors the structure of the `HIL_Research/hil/data/` folder
- Download desired data from [box storage](https://uofi.box.com/s/ivdfygn9meyj8cjahn5d5j6hd570sjz8) into the respective subfolder

## Codebase Structure
Each main folder in the codebase is a separate [ros package](http://wiki.ros.org/Packages). This entire repo is intended to be cloned into the source folder of a catkin workspace(Ex. `~/catkin_ws/src`) and compiled using `catkin_make` just like any other ros package. 
### Folder Descriptions
- `hil/` : holds main HIL research code
    - `data/` :  all input files, should primarily be populated with data from box
        - `extracted_bags`: Bag messages converted into image files, pointcloud/transform numpy arrays, etc.
        - `GEM_rosbags/`: Rosbags recorded from the GEM vehicle 
        - `models/` : all 3d objects and textures
        - `scenes/` : tbd, but will likely be some json format description of different testing scenes(waypoints + animated objects + timestamps)
        - `config.json` : tbd, any future configuration settings
    - `experimentation/` : random python notebooks and scripts
    - `launch/` : launch scripts that run multiple ros nodes together
    - `src/hil` : Main custom python module for all source code
    - `*.py` : ros scripts that launch a single node
    - `CMakeLists.txt` and `setup.py` : Define how to compile this package
    - `package.xml` : Metadata for the package

- `lidar_camera_calibration/` : calibration package cloned from [repo](https://github.com/ankitdhall/lidar_camera_calibration)
- `novatel_gps_msgs` : Defines gps-related ros messages, cloned from [repo](https://github.com/swri-robotics/novatel_gps_driver/tree/master/novatel_gps_msgs)
- `scripts` : not a ros package, holds some convience shell scripts to record rosbags or setup dependecies
### General Notes
- In most cases, it is relatively easy to convert non-ros code to use ros. But it is often easier to debug when working with a single image/data point instead of an entire rosbag(note that we have ros script to convert a bag in to standard image/data files). 
- So after running a certain step(insertion, object detection etc.) without ros, we can just convert the appropriate ros messages into the desired python format(cv2, numpy, etc.) and call our standalone python code in to create a functioning ros node.
- For each ros node, the future plan is to define the main class/logic inside `hil/src/hil` and then write the ros wrapper in a respective `hil/*.py` script(this may or may not work out :)

## ~~Getting Started~~ [TODO: Update with docker container setup]
1. Setup your catkin workspace: `mkdir -p ~/catkin_ws/src` and clone this repo into the `src` subfolder
2. Run `sh setup_scripts/ros_noetic_install.sh` to install ros noetic
3. ~~Run `sh setup_scripts/hdl_slam_install.sh` to install the hdl_slam dependency~~
4. Install package dependencies with: `cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y`
5. `cd ~/catkin_ws` and `catkin_make` to build the packages in your workspace
6. `source devel/setup.bash` so ros know where to find your custom packages
7. TODO: Download some demo rosbags into the data folder
8. Try out some of `hil` package launch files!
