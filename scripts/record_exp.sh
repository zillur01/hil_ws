EXPERIMENT_NAME="exp1_scene1_path_1"
mkdir -p experiments/$EXPERIMENT_NAME
rosbag record -O experiments/$EXPERIMENT_NAME subset \
               /zed2/zed_node/left/image_rect_color /pacmod/as_rx/brake_cmd \
               /il/pred_brake /modular/pred_speed \
               /novatel/inspva /novatel/fix \
               /insertion/rendered_image /insertion/rendered_depth \
               /insertion/obstacle_point \
               /insertion/latency /il/latency /modular/latency