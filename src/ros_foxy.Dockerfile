FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt install -y software-properties-common && \
    add-apt-repository universe

RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y

RUN apt install -y ros-foxy-ros-base python3-argcomplete ros-dev-tools

RUN apt-get install -y ros-foxy-rmw-cyclonedds-cpp

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp