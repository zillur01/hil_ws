# HIL_Research
> **Status (branch `dev`):** ported from ROS 1 Noetic/Docker to **ROS 2 Humble**. All nodes and launch files in
> `src/hil/` are ROS 2 (see "ROS 2 quickstart", "First test" and "Other nodes"), except the lidar-camera calibration
> (`aruco_markers.py`, `src/calibration/`, `launch/find_transform_real.launch` and the `lidar_camera_calibration`
> package), which is still ROS 1 only. The Docker/catkin instructions and the shell scripts in `scripts/` further
> down are ROS 1 and obsolete.
>
> **Layout:** `src/hil/` is the ROS 2 package (`package.xml`, `setup.py`, nodes, launch files).
> `src/hil/src/hil_common/` is the Python library the nodes import (`from hil_common import origin`; it was
> called `hil` before, which was confusing). If you built an older checkout, delete `build/hil install/hil`
> before running `colcon build` again.
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
4. `cd src/hil/src/hil_common && python insertionEngine.py`
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

**2b. Car test: a pedestrian coming from the left.** No waypoints are needed: the insertion node only uses the
car's current pose. (Waypoints are for the modular agent, later.) Park the car at the start of a straight, empty
stretch, facing along it, with the ZED, Septentrio and PACMod drivers running, and run one command:
```bash
ros2 run hil plan_pedestrian.py --ahead 25 --left 4
```
It reads the car's pose and prints the launch command to run, for example:
```
Car: ENU (-3.70, -13.50), heading 90.0 deg (compass)
Pedestrian: ENU (21.30, -9.50), 25 m ahead, 4 m left, walks towards compass 180.0 deg (right), starts when the car is within 15 m

ros2 launch hil insertion.launch.py config_path:=gem/jaywalker.yaml auto_origin_alt:=true actor_east:=21.30 actor_north:=-9.50 actor_walk_heading_deg:=180.0 trigger_distance:=15
```
Copy that line and run it (with the car still at the same spot: `auto_origin_alt` takes the starting height as ground
level), then watch `/insertion/rendered_image` (`rqt_image_view`). Options of `plan_pedestrian.py`: `--ahead` and `--left`
(metres, relative to the car; use a negative `--left` for the right side), `--walk right|left` (which way it crosses),
`--trigger` (it stands still until the car is that close, then walks), `--config` (a jaywalking config with an
`actor_0`). Add `rgb_topic:=... depth_topic:=...` to the launch line if your ZED topics differ from the defaults.

The launch flags override `actor_0` of the config, so nothing needs editing: `actor_east` / `actor_north` are the
local ENU position (the same numbers the node logs as `obstacles (ENU ...)`), `actor_walk_heading_deg` is the compass
direction it walks, `trigger_distance` is in metres. The `obstacles (ENU ...)` log line must show the pedestrian where
you intended. If it is on the wrong side, the heading is off (see "Heading" below); if it floats or sinks, the camera
height is not 1.8 m.

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
  first GNSS height as local z = 0; otherwise set `ORIGIN_ALT` in `hil_common/origin.py`. If the antenna is not about
  1.8 m above the ground, change every `-1.8` in the config (`world.scenario2world`, `static_objects.scenario2world`)
  to minus your antenna height, or the objects float or sink.
- **Camera model.** `camera_config.fov` is the *vertical* field of view in degrees; for your ZED use
  `2*atan(720 / (2*fy))` from `camera_info` (about 68.6 for a ZED2 at HD720; the configs use 72).
- **Heading.** `/insnavgeod` heading reads about 0 facing north on the car; a phone compass differed by ~10 deg, which
  may just be the phone. A heading error rotates the camera against the objects (10 deg = 5 m sideways at 30 m).
  To measure it, drive straight at 3-5 m/s and compare `atan2(dEast, dNorth)` from `/navsatfix` with the reported
  heading; a constant offset goes into `HEADING_OFFSET_DEG` in `hil_common/origin.py` (do this before recording waypoints).
- **Roll / pitch.** Checked on the car: nose up gives larger pitch, right side up gives smaller roll, which matches
  `pose.py`, so no sign flips. Both read about -1.3 deg on level ground (mount bias or slope), left uncorrected.
- **GNSS pose = camera pose.** There is no camera mount offset, as in the original code.
- **Output size.** The image is rendered and published at the config's `window_size` (640x360), not at camera
  resolution, and the depth is the virtual objects only (100 m elsewhere). The modular agent assumes 1280x720
  ZED images: for closed-loop runs set `window_size: [1280, 720]` (slower) or adapt the agent's intrinsics.
- **Pedestrian speed** depends on the rendered frame rate (one step per rendered frame).
- `/insertion/obstacle_poses` is now in the local ENU frame (the ROS 1 node published renderer coordinates).
- **Frame rate.** The renderer alone does ~110 FPS at 640x360 on an RTX A4500; the node used to reach only 16 FPS
  because rclpy's `throttle_duration_sec` inspects the Python stack on every log call (about 40% of the frame
  time with torch loaded). The node now throttles its logs itself and keeps up with a 30 Hz camera. If you still
  see ~5 FPS on the car, check the `renderer:` line printed at start-up (it must name the NVIDIA GPU, not
  `llvmpipe`), watch `nvidia-smi` while it runs, and profile with
  `python -m cProfile -o node.prof $(ros2 pkg prefix hil)/lib/hil/insertion_node.py --ros-args -p config_path:=...`
  (stop with Ctrl-C, then `python -c "import pstats; pstats.Stats('node.prof').sort_stats('tottime').print_stats(15)"`).
  The field of view only changes the projection, not the speed. The ZED field of view from your `camera_info`
  (fy = 263.43 at 360 rows) is 2*atan(360/(2*263.43)) = 68.7 deg; the configs use the original 72.

## Closed-loop test: modular agent with the virtual pedestrian
What the agent outputs: **only a desired speed** (IDM longitudinal controller), no steering. It looks at the
inserted RGB + rendered depth, tracks the pedestrian, predicts whether it comes within 3 m of the planned path
(the waypoints) in the next 10 s and lowers the speed (free-flow speed is 2 m/s, minimum spacing 3 m).
It is published on `/modular/pred_speed` as a `pacmod2_msgs/VehicleSpeedRpt`; the desired acceleration is only logged.

In the paper (arXiv 2306.08807, IV-B.3) the agent's output goes to a separate vehicle controller: a PI speed
controller for the longitudinal direction and a Stanley controller (bicycle model) for steering along the planned
path. That controller is not in this repo. Also, the paper describes the agent's output as 0 or 2 m/s, while the code
computes a smooth IDM speed.

**Nothing in this repo turns that into PACMod commands.** Steering and speed tracking come from `gem_gnss_control`'s
`pure_pursuit`, which uses a fixed `desired_speed` parameter (read once at start-up), never subscribes to
`/modular/pred_speed`, and only brakes in its final stopping sequence. So the first test is open loop:
you drive (or pure_pursuit drives) and you watch what the agent *would* command.

**Needed before the run**
1. **Image size must match the agent's camera model.** `HIL_modular_autonomy/agent.py` crops 720x1280 and hard-codes
   the ZED intrinsics for that size (fx = fy = 527.58, cx = 616.25, cy = 359.22). Your ZED publishes 640x360 because
   `common_stereo.yaml` has `pub_resolution: 'CUSTOM'`, `pub_downscale_factor: 2.0`; its `camera_info` is exactly half
   (fx = 263.43, cx = 308.12, cy = 179.61). Fed 640x360 with the 1280x720 numbers the agent would put objects at the
   wrong sideways positions. Two ways out: (a) `pub_resolution: 'NATIVE'` and `window_size: [1280, 720]`, no code
   change, slower; or (b) stay at 640x360 and change the agent's crop (`i2, j2 = 360, 640`) and intrinsics to your
   `camera_info` values. (b) is faster, but the segmentation networks then see small, low-resolution pedestrians; the
   paper's pipeline ran at 720p, so try it and compare.
2. **Depth range:** with `use_inserted_data:=true` the agent reads `/insertion/rendered_depth` (virtual objects only,
   up to 100 m), not the ZED depth, so the ZED `max_depth` (10 m in `zed2.yaml`) does not matter for this test. It
   matters when the agent runs on the real camera (`use_inserted_data:=false`): raise it to 30-40 m then.
3. Waypoints along the stretch, in the local frame, same origin as this repo (the path the pedestrian must cross):
   ```bash
   ros2 run gem_gnss_control record_waypoints --ros-args -p origin_lat:=40.092857 -p origin_lon:=-88.235992 \
       -p output_file:=/abs/path/straight.csv      # drive the stretch slowly, then Ctrl-C
   ```
   Place the pedestrian (step 2b) so that it crosses this path, not just the road. The render size comes from the
   config's `window_size` (640x360 in `jaywalker.yaml`; edit it there if you choose option (a) above).

**Run** (3 terminals, car on the stretch start; ZED, Septentrio and PACMod already running):
```bash
ros2 run hil plan_pedestrian.py --ahead 25 --left 4      # prints the insertion launch command: run that line
ros2 launch hil modular_agent.launch.py waypoint_csv_path:=/abs/path/straight.csv use_inserted_data:=true
ros2 topic echo /modular/pred_speed --field vehicle_speed
```
Drive towards the pedestrian by hand (or with `pure_pursuit` at a low speed). Expected: `vehicle_speed` stays near the
current speed, then falls towards 0 as the pedestrian is tracked and predicted to cross the path. The agent logs
`desired spd` and `desired acc` per frame. Stay in control of the car: the pedestrian is virtual and nothing
brakes the car for it yet.

**To close the loop** `pure_pursuit` (or a small bridge node) has to follow `/modular/pred_speed` and brake when the
target is below the current speed (today the PID output is clamped to throttle only). That is a change in
`gem_gnss_control`, not in this repo, so it has not been done.

## Septentrio / PACMod inputs and the local frame
The nodes no longer use the Novatel `Inspva` topic. Inputs (names fixed in `src/hil/src/hil_common/globals.py`):
- `/navsatfix` (`sensor_msgs/NavSatFix`): lat/lon
- `/insnavgeod` (`septentrio_gnss_driver/INSNavGeod`): heading, pitch, roll, height
- `/pacmod/vehicle_speed_rpt` (`VehicleSpeedRpt`): ego speed in m/s

All poses, waypoints and virtual objects live in one local East-North-Up frame (metres) centred on the
origin in `src/hil/src/hil_common/origin.py` (same as `gem_gnss_control` `e2_pp.yaml`). Waypoints are header-less
CSV rows `x, y, heading_deg` in that frame, so files recorded by `gem_gnss_control/record_waypoints.py`
(or by `save_waypoints.launch.py` here) work directly as `waypoint_csv_path`. Search for `TODO(USER)` for the
values you still need to check (heading convention, roll/pitch signs, origin height, ZED topic names).

Python dependencies for everything: `pip install -r requirements.txt` (see the header for torch/detectron2).

## Other nodes (ROS 2)
Build as in the quickstart (`--symlink-install`). Every launch file takes `use_sim_time` or plays a bag with
`--clock` itself; topic names default to the ZED (`/zed/zed_node/...`) and `/lidar1/velodyne_points` and can be
changed with the `*_topic` arguments (`ros2 launch hil <file> --show-args`).

| launch | what it does |
|---|---|
| `base.launch.py` | tf `world -> gps` (car pose in the local frame) and an axis marker on `/gps_vis_marker` for RViz |
| `metrics.launch.py waypoint_csv_path:=...` | time to the last waypoint, distance to the virtual obstacle at the first IL brake, average latencies |
| `save_waypoints.launch.py csv_name:=...` | records waypoints (same format as `gem_gnss_control`) to `src/hil/data/waypoints/`; `bag_path:=` to record from a bag |
| `save_data.launch.py input_bag_path:=...` | plays a bag and saves synced images, depth (`.npy`, metres) and GNSS poses to `<bag dir>_extracted/<bag name>/` |
| `save_data_IL.launch.py input_bag_path:=...` | plays a bag at half speed and writes `synced.csv` for imitation learning; stops by itself when the bag ends |
| `gen_map.launch.py bag_path:=...` | colored lidar map on `/projected` (RViz, fixed frame `world`) |
| `lidar2cam.launch.py` | lidar points projected onto the image (`/projected_vis`) to check `T_lidar_rgb` in `globals.py`; `bag_path:=` optional |
| `il_agent.launch.py` | imitation-learning agent: `/il/pred_brake` (`pacmod2_msgs/SystemCmdFloat`), `/il/desired_speed` |

Notes:
- **Bags** are ros2 bags (directories). ROS 1 `.bag` files can be converted with `pip install rosbags` and
  `rosbags-convert --src old.bag --dst old`, but the old bags have the Novatel and ROS 1 PACMod topics, not
  `/navsatfix`, `/insnavgeod` and `/pacmod/vehicle_speed_rpt`, so the GNSS-based nodes need new recordings.
- **PACMod** topics are those of the ROS 2 driver: `/pacmod/accel_cmd` and `/pacmod/brake_cmd`
  (`SystemCmdFloat`), `/pacmod/steering_rpt` (`SystemRptFloat`). The IL brake output changed from `PacmodCmd.f64_cmd`
  to `SystemCmdFloat.command`. ROS 2 headers have no `seq`, so the `*_seqnum` columns of `synced.csv` count the
  messages received per topic (0 = first one) instead of holding ROS 1 sequence numbers.
- The **IL agent** is a git submodule: `git submodule update --init src/hil/src/HIL_imitation_learning`.
- `gen_map` / `lidar2cam` need a lidar (none is mentioned in the car setup above); the mapping node syncs lidar and
  image within 10 ms, as before.

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
- `scripts` : not a ros package, holds some convience shell scripts to record rosbags or setup dependecies
### General Notes
- In most cases, it is relatively easy to convert non-ros code to use ros. But it is often easier to debug when working with a single image/data point instead of an entire rosbag(note that we have ros script to convert a bag in to standard image/data files). 
- So after running a certain step(insertion, object detection etc.) without ros, we can just convert the appropriate ros messages into the desired python format(cv2, numpy, etc.) and call our standalone python code in to create a functioning ros node.


## Virtual object insertion with COLMAP poses (offline)
[COLMAP](https://colmap.github.io/) is a structure-from-motion tool: given many photos of a scene, it finds matching
features between them and computes the camera pose of every photo (and a sparse 3D point cloud). Here it is an
offline alternative to the GNSS poses: `insertion_colmap.py` renders a 3D object into a recorded image sequence using
the camera poses COLMAP estimated from the images themselves, so the object should not drift the way it can with
GNSS/INS noise. It does not use ROS and is not part of the real-time pipeline. COLMAP poses have an arbitrary origin,
orientation and scale, so the object position (`model_mat` in `run_offline`) is in COLMAP units, not metres: tune it
by hand per sequence.

**1. Get an image sequence.** Download **extracted_bags** from [Box](https://uofi.app.box.com/folder/182731812480) into
`src/hil/data/` (it has `outdoor1/img/im_<N>.png` and, possibly, a ready COLMAP model), or extract your own drive with
`save_data.launch.py` (above), which writes `img/im_<N>.png`. The renderer is hard-coded to 1280x720 images.

**2. Run COLMAP** (`sudo apt install colmap`, or the GUI). The script expects one camera with the `PINHOLE` or `OPENCV`
model and the text model in `colmap/opencv/sparse/txt/` next to `img/`:
```bash
cd src/hil/data/extracted_bags/outdoor1
mkdir -p colmap/opencv/sparse/txt
colmap feature_extractor --database_path colmap/opencv/db.db --image_path img \
    --ImageReader.camera_model OPENCV --ImageReader.single_camera 1
colmap sequential_matcher --database_path colmap/opencv/db.db      # images are a video sequence
colmap mapper --database_path colmap/opencv/db.db --image_path img --output_path colmap/opencv/sparse
colmap model_converter --input_path colmap/opencv/sparse/0 --output_path colmap/opencv/sparse/txt --output_type TXT
```
Check that `sparse/0` is the only (or the largest) model: if COLMAP splits the sequence, only the frames in the model
you convert get poses.

**3. Insert the object.** It renders `data/models/dummy.obj` (texture `dummy.jpg`) into every frame that has a pose:
```bash
cd src/hil/src/hil_common
python insertion_colmap.py   # output: data/extracted_bags/outdoor1/insertion_output_colmap/ (PNGs + video.mp4)
python insertionEngine.py    # same, with the GNSS poses, for comparison (reads outdoor1_better_sync)
```
`src/hil/colmap.py` (run from `src/hil/`) exports the COLMAP poses to `.npy` files and shows the camera path with
`vedo` (`test_pose()`); edit its `dir_colmap` if your model is in `colmap/opencv/sparse/txt`.
