# HIL_Research
> **Status (branch `dev`):** being ported from ROS 1 Noetic/Docker to **ROS 2 Humble**. Ported so far: the modular
> agent and the virtual-object insertion (see "ROS 2 quickstart" and "First test"). The Docker/catkin instructions
> further down are obsolete, and the other nodes in `src/hil/` (metrics, save_data, IL, mapping, lidar calibration)
> are still ROS 1 only.
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
## ROS 2 quickstart (modular agent)
```bash
python3.10 -m venv --system-site-packages ~/venvs/hil && source ~/venvs/hil/bin/activate   # see requirements.txt header
pip install -r requirements.txt            # + torch and detectron2 as described in its header
# put this repo's src/ in a colcon workspace next to septentrio_gnss_driver and pacmod2_msgs, then:
source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select hil
source install/setup.bash
ros2 launch hil modular_agent.launch.py waypoint_csv_path:=/abs/path/my_track.csv \
    rgb_topic:=/zed/zed_node/left/image_rect_color depth_topic:=/zed/zed_node/depth/depth_registered
```
- `--symlink-install` is required: the agent and its weights are loaded from the source tree
  (`src/hil/src/HIL_modular_autonomy`), or pass `agent_dir:=...`.
- Output: `/modular/pred_speed` (`pacmod2_msgs/VehicleSpeedRpt`, desired speed) and `/modular/latency`.
- With a bag, use `ros2 bag play --clock` and `use_sim_time:=true`.
- **Recording waypoints:** use `gem_gnss_control`'s `record_waypoints` with the same origin as this repo
  (it defaults to a different one!): `ros2 run gem_gnss_control record_waypoints --ros-args -p origin_lat:=40.092857 -p origin_lon:=-88.235992`.

## First test: are the objects rendered where I want them?
The insertion node renders virtual objects into the camera image at the car's pose from `/navsatfix` +
`/insnavgeod` and publishes `/insertion/rendered_image`. Object positions come from a YAML config; ready-made
ones are in `src/hil/src/HIL_realtime_rendering/configs/gem/` (positions are in the local frame, see below):

| config | what it shows |
|---|---|
| `static_cone_30m.yaml` | one cone 30 m east of the origin, on the `my_track.csv` route (y = -13) |
| `static_multi.yaml` | a cone (20 m), a pedestrian beside the road (35 m) and a parked car (50 m) along the route |
| `compass_ring.yaml` | cone east, car north, chair west, person south, each 10 m from the origin (orientation check) |
| `jaywalker.yaml` | a pedestrian crosses the route 40 m east of the origin, triggered within 15 m |

**1. Run it.** Build as in the quickstart, then either with the car (ZED, Septentrio running; start the node with
the car at the point where it should be "ground level"):
```bash
ros2 launch hil insertion.launch.py config_path:=gem/static_multi.yaml auto_origin_alt:=true \
    rgb_topic:=/zed/zed_node/left/image_rect_color depth_topic:=/zed/zed_node/depth/depth_registered
```
or at a desk with fake sensors, which publish a camera image (grey ground, blue sky), `/navsatfix`, `/insnavgeod`
and a PACMod speed for a car standing at local (x, y) facing `heading_deg` (compass degrees, 0 = north, 90 = east),
or driving along a waypoint file:
```bash
ros2 run hil fake_sensor_node.py --ros-args -p x:=-3.7 -p y:=-13.5 -p heading_deg:=90          # terminal 1
ros2 launch hil insertion.launch.py config_path:=gem/static_multi.yaml auto_origin_alt:=true   # terminal 2
ros2 run hil fake_sensor_node.py --ros-args -p waypoint_csv_path:=/abs/my_track.csv -p speed:=2.0   # or: drive it
```
Look at the result with `ros2 run rqt_image_view rqt_image_view /insertion/rendered_image` (or add
`show_debug:=true` to get OpenCV windows). `config_path` is absolute, or relative to the renderer's `configs/` folder.
To try several configs, stop the node and start it again with another `config_path` (it is read at start-up).

**2. Check the placement.** The node logs every 2 s the car's position and the objects' positions in the same local
frame, e.g. `camera ENU (-3.70, -13.50, 0.00)` and `obstacles (ENU east, north, up): (20.0, -13.0, -1.8); ...`;
the same objects are published on `/insertion/obstacle_poses`. For a car at (-3.7, -13.5) facing east, the cone
(20, -13) must appear almost straight ahead and the car (50, -17) to the right. With `compass_ring.yaml`, standing
at the origin and turning to 90 / 0 / 270 / 180 degrees must show cone / car / chair / person.

**3. Define your own objects.** Copy a config from `configs/gem/` and edit `object_configs` (or `actor_configs`):
- **Position:** the renderer's world frame is x = -East, y = Up, z = North. An object at local position
  (E, N) therefore needs `pos: [-E, 0.0, N]` in `object2scenario` (with `scenario2world.pos: [0.0, -1.8, 0.0]`).
  Mind the sign of E: `pos: [10, 0, 0]` is 10 m **west** of the origin.
- **Heading:** `rot: [0.0, yaw, 0.0]`, degrees about the vertical axis. Walking actors move along their own
  heading: yaw 0 = north, 180 = south, 90 = west, 270 = east.
- **Types** (`object_type`): `cone` (traffic cone, `scale: 1.0` is about 0.75 m; `sim_cone` is a scanned cone, 0.9 m at scale 1), `benz`, `suv`, `bus`, `baby`, `building`, `yuan`,
  `albert`, `zhi-hao`, `chair_0`, `chair_1`, `ladder`, `traffic_light` and the scanned `real_*` objects;
  walking/running people (`animation_type`) are `man_walking`, `woman_walking`, `man_running`, `lady_walking`, ... at
  `scale: 1.0` (about 1.8 m tall).
- Other scenarios (`intersection`, `jaywalking`) are described by the example configs in `configs/exp_*`.

**4. Things that move the objects away from where you expect:**
- **Height.** The ground plane is 1.8 m below the camera height at the origin. `auto_origin_alt:=true` takes the
  first GNSS height as local z = 0; otherwise set `ORIGIN_ALT` in `hil/origin.py`. If the antenna is not about
  1.8 m above the ground, change every `-1.8` in the config (`world.scenario2world`, `static_objects.scenario2world`)
  to minus your antenna height, or the objects float or sink.
- **Camera model.** `camera_config.fov` is the *vertical* field of view in degrees; for your ZED use
  `2*atan(720 / (2*fy))` from `camera_info` (about 68.6 for a ZED2 at HD720; the configs use 72).
- **Heading / roll / pitch.** The `TODO(USER)` items: confirm `/insnavgeod` heading is a compass heading (the
  compass test above does that), and the roll/pitch signs.
- **GNSS pose = camera pose.** There is no camera mount offset, as in the original code.
- **Output size.** The image is rendered and published at the config's `window_size` (640x360), not at camera
  resolution, and the depth is the virtual objects only (100 m elsewhere). The modular agent assumes 1280x720
  ZED images: for closed-loop runs set `window_size: [1280, 720]` (slower) or adapt the agent's intrinsics.
- **Pedestrian speed** depends on the rendered frame rate (one step per rendered frame).
- `/insertion/obstacle_poses` is now in the local ENU frame (the ROS 1 node published renderer coordinates).

## Septentrio / PACMod inputs and the local frame
The nodes no longer use the Novatel `Inspva` topic. Inputs (names fixed in `src/hil/src/hil/globals.py`):
- `/navsatfix` (`sensor_msgs/NavSatFix`): lat/lon
- `/insnavgeod` (`septentrio_gnss_driver/INSNavGeod`): heading, pitch, roll, height
- `/pacmod/vehicle_speed_rpt` (`VehicleSpeedRpt`): ego speed in m/s

All poses, waypoints and virtual objects live in one local East-North-Up frame (metres) centred on the
origin in `src/hil/src/hil/origin.py` (same as `gem_gnss_control` `e2_pp.yaml`). Waypoints are header-less
CSV rows `x, y, heading_deg` in that frame, so files recorded by `gem_gnss_control/record_waypoints.py`
(`save_waypoint_node.py` here is ROS 1 only) work directly as `waypoint_csv_path`. Search for `TODO(USER)` for the
values you still need to check (heading convention, roll/pitch signs, origin height, ZED topic names).

Python dependencies for everything: `pip install -r requirements.txt` (see the header for torch/detectron2).

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

## Virtual object insertion 
[Data arrangement]
- Data are under `src/hil/data/`
- Download data **extracted_bags** from [Box](https://uofi.app.box.com/folder/182731812480), and put under `src/hil/data/`

[Run insertion]
```bash
git checkout bug_drifting
cd src/hil/src/hil
python insertion_colmap.py # insert with COLMAP poses
python insertionEngine.py  # insert with GPS poses
```
and the output files are:
- COLMAP: `src/hil/data/extracted_bags/outdoor1/insertion_output_colmap`
- GPS: `src/hil/data/extracted_bags/outdoor1_w_raw_gps_depth/insertion_output`.