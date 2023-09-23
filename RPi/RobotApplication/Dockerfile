FROM ros:melodic-ros-base

RUN apt update && apt install ros-melodic-teleop-twist-keyboard

RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN mkdir -p /catkin_ws/src

COPY ./autostart.sh /
RUN chmod +x /autostart.sh
ENTRYPOINT /autostart.sh
# ENTRYPOINT /bin/bash
