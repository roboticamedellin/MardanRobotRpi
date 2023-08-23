FROM ros:melodic-ros-base

# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    tmux \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN apt-get install python-smbus -y

RUN mkdir -p /catkin_ws/src

COPY ./autostart.sh /
RUN chmod +x /autostart.sh
ENTRYPOINT /autostart.sh
# ENTRYPOINT /bin/bash