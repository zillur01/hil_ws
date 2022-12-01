# HIL_Research
## Dependencies
- `docker`, `docker-compose`
- `make` (kinda optional, but used for convienence of building/running docker containers)

*Note that I have only successfully tested this on Ubuntu. I've attempted to run the containers on windows and though the containers would build and run, I was not able to get the gui/some networking stuff to work from inside the container.*
## Quickstart
### Docker Setup
1. Clone this repo, and in `cd` into it
2. `make docker`: builds the linux-based docker image with ros, python requirements, and all other dependencies
3. `make` or `make dev`: starts the container, builds the ros packages in `src/`, and opens up a tmux environment.
4. Download the desired data from box. See [Data Setup](https://github.com/bchandaka/HIL_Research/edit/main/README.md#data-setup)
5. In the `ros` window in tmux, you can run any ros/python scripts from a terminal inside the container.
   - Ex. `roslaunch hil insertion_testing.launch`
   
*See the `Makefile` for other command shortcuts*

*Depending on your feature, you may not even need ros or the docker container to do your intial work and can just work in python directly since we can integrate it into ros later. See the example below.
### Example running standalone insertion code with python
1. Download the desired data from box. See [Data Setup](https://github.com/bchandaka/HIL_Research/edit/main/README.md#data-setup)
  - You will need to populate at least the `extracted_bags/outdoor1/` and `models/` folders from box to run the insertion code
2. `pip install -r requirements.txt`
3. You may need to change the absolute paths/ros-based paths on lines 196-199 in insertionEngine.py to reflect the correct extracted data and model/texture paths
4. `cd src/hil/src/hil && python insertionEngine.py`
## Data Setup
- A box folder with recorded data and model files mirrors the structure of the `src/HIL_Research/hil/data/` folder
- Download desired data from the `data/` folder in [box storage](https://uofi.box.com/s/ivdfygn9meyj8cjahn5d5j6hd570sjz8) into the respective subfolder

## Codebase Structure
- The docker structure of this codebase is based on the template here: https://github.com/nachovizzo/ros_in_docker \
- Essentially the repo root directory is treated as our [catkin workspace](http://wiki.ros.org/catkin/workspaces). After building automatically inside the container with `catkin_make`, the `devel`, `log`, and `install` will be created as normal(but these folders will be ignored in git) next to the `src/` folder. \
- All source code is in `src/`. Each main folder in `src/` is a separate [ros package](http://wiki.ros.org/Packages).
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

## Visualizations
- A good visualizer for rosbags and live ros data is [Foxglove Studio](https://foxglove.dev/)
- TODO: Upload example panel jsons
## ~~OLD Getting Started with local ros install~~ 
1. Setup your catkin workspace: `mkdir -p ~/catkin_ws/src` and clone this repo into the `src` subfolder
2. Run `sh setup_scripts/ros_noetic_install.sh` to install ros noetic
3. Install package dependencies with: `cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y`
4. `cd ~/catkin_ws` and `catkin_make` to build the packages in your workspace
5. `source devel/setup.bash` so ros know where to find your custom packages
6. Download the desired data from box. See [Data Setup](https://github.com/bchandaka/HIL_Research/edit/main/README.md#data-setup)
7. Try out some of `hil` package launch files!
