FROM ros:humble-ros-base

WORKDIR /ros_ws/src

RUN apt-get update \
    && apt-get install -y python3-pip

# ought to be able to specify Python dependencies through rosdep, but can't get it to work
RUN python3 -m pip install pymavlink transforms3d

COPY . /ros_ws/src/macaw

# Install dependencies
WORKDIR /ros_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src \
    && rm -rf /var/lib/apt/lists/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

EXPOSE 14000/udp
EXPOSE 5900

ENTRYPOINT [ "/ros_ws/src/macaw/macaw_entrypoint.sh" ]

CMD [ "ros2", "launch", "macaw", "macaw.launch.xml" ]
