ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

RUN apt update && apt upgrade -y && \
    apt install autoconf libtool python3-flask \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp -y
RUN mkdir ec_dev && cd ec_dev && \
    git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.6 && \
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install

RUN mkdir -p /ros2_dev/src && cd /ros2_dev/src && \
    git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git -b jazzy

COPY . /ros2_dev/src

RUN cd /ros2_dev && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]