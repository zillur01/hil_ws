INSERTION="roslaunch hil insertion_testing.launch use_bag:=1"
MODULAR="roslaunch hil modular_agent.launch use_bag:=1 use_inserted_data:=1 waypoint_csv_path:=\"/media/bhargav/03E7-EC27/cone_bags/speed_profile/extracted_data/gps.csv\""
METRICS="roslaunch hil metrics.launch use_bag:=1"
gnome-terminal --tab -- $INSERTION 
sleep 2
gnome-terminal --tab -- $MODULAR 
sleep 2
gnome-terminal --tab -- $METRICS 
