FROM ros-foxy

SHELL ["/bin/bash", "-c"]

RUN apt install -y ros-foxy-xacro ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3-gazebo

RUN mkdir -p /home/workspace/src/waypoint_follower

WORKDIR /home/workspace/

COPY ./waypoint_follower/ ./src/waypoint_follower/

RUN ls -la ./src/waypoint_follower/*

RUN source /opt/ros/foxy/setup.bash && colcon build

ENTRYPOINT ["/bin/bash","-c","source install/setup.bash && ros2 run waypoint_follower navigator -6.0 -6.0 0.0"]