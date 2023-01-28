EXPERIMENT_NAME="exp1_scene1"
WAYPOINT_CSV="/home/yshen47/Documents/rosbags/static_path1_gps.csv"
CONFIG_DIR="/home/yshen47/catkin_ws/src/HIL_Research/src/hil/src/HIL_realtime_rendering/configs/"
RENDERING_CONFIG="$CONFIG_DIR"/exp_1/scen_1/path_1.yaml
INSERTION="roslaunch hil insertion_testing.launch use_bag:=0 config_path:=$RENDERING_CONFIG"
MODULAR="roslaunch hil modular_agent.launch use_bag:=0 use_inserted_data:=1 waypoint_csv_path:=$WAYPOINT_CSV"
IL_AGENT="roslaunch hil il_agent.launch use_bag:=0 use_inserted_data:=1"
METRICS="roslaunch hil metrics.launch use_bag:=0 waypoint_csv_path:=$WAYPOINT_CSV experiment_name:=$EXPERIMENT_NAME"
gnome-terminal --tab -- $INSERTION 
sleep 2
gnome-terminal --tab -- $IL_AGENT 
sleep 2
gnome-terminal --tab -- $METRICS 
