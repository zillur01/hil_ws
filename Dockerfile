FROM ros:noetic
RUN mkdir -p /catkin_ws/src
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    tmux \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc