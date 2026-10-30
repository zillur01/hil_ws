EXPERIMENT_NAME="human_driver/exp5_scen_1_path_fast_3"

WAYPOINT_CSV="/home/ashwath/HIL_Research/rosbags/waypoints/jaywalker.csv"
CONFIG_DIR="/home/ashwath/HIL_Research/src/hil/src/HIL_realtime_rendering/configs"
RENDERING_CONFIG="$CONFIG_DIR"/exp_5/scen_1/path_fast_3.yaml
INSERTION="roslaunch hil insertion_testing.launch use_bag:=0 config_path:=$RENDERING_CONFIG"
MODULAR="roslaunch hil modular_agent.launch use_bag:=0 use_inserted_data:=1 waypoint_csv_path:=$WAYPOINT_CSV"
IL_AGENT="roslaunch hil il_agent.launch use_bag:=0 use_inserted_data:=1"
METRICS="roslaunch hil metrics.launch use_bag:=0 waypoint_csv_path:=$WAYPOINT_CSV experiment_name:=$EXPERIMENT_NAME"
ROSBAG_RECORD="rosbag record -O experiments/$EXPERIMENT_NAME \
               /zed2/zed_node/left/image_rect_color /pacmod/as_rx/brake_cmd \
               /il/pred_brake /modular/pred_speed /il/desired_speed \
               /novatel/inspva /novatel/fix \
               /insertion/rendered_image /insertion/rendered_depth \
               /insertion/obstacle_poses \
               /insertion/latency /il/latency /modular/latency"
mkdir -p experiments
gnome-terminal --tab -- $INSERTION 
sleep 1
# SWITCH THE COMMENT BASED ON WHICH AGENT YOU WANT TO TEST
# gnome-terminal --tab -- $IL_AGENT 
# gnome-terminal --tab -- $MODULAR 
sleep 1
gnome-terminal --tab -- $METRICS 
sleep 1
gnome-terminal --tab -- $ROSBAG_RECORD 
