rosbag record -O subset \
    --duration=200 \
    /gem/velodyne_points \
    /gem/front_single_camera/image_raw \
    /gem/front_single_camera/camera_info \
    /tf_static \
    /gem/imu \
    /gem/gps/fix \
    /gem/gps/fix_velocity