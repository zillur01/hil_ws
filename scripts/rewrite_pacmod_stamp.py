import rosbag
import argparse

input = "/media/bhargav/03E7-EC27/cone_bags/cone2_2022-12-12-16-30-37.bag"
output = "/media/bhargav/03E7-EC27/cone_bags/cone2_2022-12-12-16-30-37_out.bag"
accel_top = "/pacmod/as_rx/accel_cmd"
brake_top = "/pacmod/as_rx/brake_cmd"
def rewrite_pacmod_stamp(input, output):
    with rosbag.Bag(output, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input).read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            elif topic == accel_top or topic == brake_top:
                msg.header.stamp = t
                outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS Bag image extraction and sync tool")
    parser.add_argument('-b', '--rosbag',
            default="/media/bhargav/03E7-EC27/cone_bags/cone2_2022-12-12-16-30-37.bag",
            type=str, required=False, help='Path to rosbag file')
    args = parser.parse_args()
    output = input.split(".")[0]+"_out.bag"
    rewrite_pacmod_stamp(args.rosbag, output)