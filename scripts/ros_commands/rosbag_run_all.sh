EXPERIMENT_NAME="il_test"

WAYPOINT_CSV="/home/gem/HIL_Research/rosbags/waypoints/static_path_2_1.csv"
CONFIG_DIR="/home/gem/HIL_Research/src/hil/src/HIL_realtime_rendering/configs"
RENDERING_CONFIG="$CONFIG_DIR"/exp_1/scen_1/path_1.yaml
INSERTION="roslaunch hil insertion_testing.launch use_bag:=1 config_path:=$RENDERING_CONFIG"
MODULAR="roslaunch hil modular_agent.launch use_bag:=1 use_inserted_data:=1 waypoint_csv_path:=$WAYPOINT_CSV"
IL_AGENT="roslaunch hil il_agent.launch use_bag:=1 use_inserted_data:=1"
METRICS="roslaunch hil metrics.launch use_bag:=1 waypoint_csv_path:=$WAYPOINT_CSV experiment_name:=$EXPERIMENT_NAME"
ROSBAG_RECORD="rosbag record -O experiments/$EXPERIMENT_NAME \
               /zed2/zed_node/left/image_rect_color /pacmod/as_rx/brake_cmd \
               /il/pred_brake /modular/pred_speed /il/desired_speed \
               /novatel/inspva /novatel/fix \
               /insertion/rendered_image /insertion/rendered_depth \
               /insertion/obstacle_poses \
               /insertion/latency /il/latency /modular/latency"
mkdir -p experiments
gnome-terminal --tab -- $INSERTION 
sleep 2
gnome-terminal --tab -- $IL_AGENT 
# gnome-terminal --tab -- $MODULAR 
sleep 2
gnome-terminal --tab -- $METRICS 
sleep 2
gnome-terminal --tab -- $ROSBAG_RECORD 