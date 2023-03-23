ARG BASE_IMAGE
FROM ${BASE_IMAGE} as base

RUN sudo apt-get update &&\
    sudo apt-get install -y \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    # best practice to keep the Docker image lean
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

##### CYCLONE DDS #####
RUN echo export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp >> /root/.bashrc && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

##### SOURCE ROS2 #####
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc


#####################
### RELEASE STAGE ###
#####################
FROM base as release

WORKDIR /root/ros2_ws
ADD . /root/ros2_ws

# compiling the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# add the entrypoint script
RUN chmod +x /root/ros2_ws/.devcontainer/entrypoint.sh
ENTRYPOINT ["/root/ros2_ws/.devcontainer/entrypoint.sh"]