rosbag record -O subset \
    --duration=20
    /camera/color/camera_info \
    /camera/color/image_raw \
    /camera/depth/image_rect_raw \
    /camera/depth/camera_info \
    /camera/depth/color/points \
    /camera/extrinsics/depth_to_color \
    /tf_static