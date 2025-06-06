FROM ros-foxy

SHELL ["/bin/bash", "-c"]

RUN apt install -y ros-foxy-xacro ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3-gazebo ros-foxy-robot-localization

RUN mkdir -p /home/workspace/src/rm_navigation

WORKDIR /home/workspace/

COPY ./rm_navigation/ ./src/rm_navigation/

RUN ls -la ./src/rm_navigation/*

RUN source /opt/ros/foxy/setup.bash && colcon build

ENTRYPOINT ["/bin/bash","-c","source install/setup.bash && ros2 launch rm_navigation navigation.launch.py"]