FROM osrf/ros:melodic-desktop-full

## see https://github.com/nvidia/nvidia-container-runtime#environment-variables-oci-spec
##ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics,display

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

SHELL ["/bin/bash", "-c"]

RUN apt-get update -q -qq \
    && apt-get dist-upgrade -q -qq -y \
    && apt-get install -q -qq -y mesa-utils gettext less net-tools \
    lsb-release python-pip ros-${ROS_DISTRO}-catkin python-wstools python-catkin-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

RUN wstool init src https://github.com/agent-system/robot_assembler/raw/master/config/robot_assembler.rosinstall && \
    wstool update -t src

RUN apt-get update -q -qq && \
    rosdep update -q -y && \
    rosdep install -q -y -r --from-paths src --ignore-src || echo 'Ignore rosdep error' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build robot_assembler

### create environment
#RUN groupadd -g 1000 leus && \
#    useradd  -g      leus -G sudo -m -s /bin/bash leus && \
#    echo 'leus:leus' | chpasswd
#RUN echo 'Defaults visiblepw'          >> /etc/sudoers
#RUN echo 'leus ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
#USER leus

RUN mkdir -p /ros_home/log && chmod a+rwx /ros_home && chmod a+rwx /ros_home/log

RUN echo -e '#!/bin/bash\n\
set -e\n\
source /my_entryrc\n\
exec "$@"\n' >> /my_entrypoint.sh
RUN chmod a+x /my_entrypoint.sh
RUN echo -e '# setup ros environment\n\
if [ -e "${DOCKER_ROS_SETUP}" ]; then\n\
    source "${DOCKER_ROS_SETUP}"\n\
else\n\
    source "/opt/ros/$ROS_DISTRO/setup.bash"\n\
fi\n\
MY_IP=${DOCKER_ROS_IP:-$(hostname -i)}\n\
if [ "$ROS_IP" == "" ]; then\n\
    export ROS_IP=${MY_IP}\n\
fi\n\
if [ "$ROS_HOSTNAME" == "" ]; then\n\
    export ROS_HOSTNAME=${MY_IP}\n\
fi\n\
if [ "${DOCKER_ROS_MASTER_URI}" != "" ]; then\n\
    export ROS_MASTER_URI=${DOCKER_ROS_MASTER_URI}\n\
fi\n\
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64\n\
export PATH=$PATH:/usr/local/cuda/bin\n\
export HOME=/ros_home\n\
export ROS_HOME=/ros_home\n' >>  /my_entryrc

#ADD ./my_entrypoint.sh /
#ADD ./my_entryrc /
ENTRYPOINT ["/my_entrypoint.sh"]
CMD ["bash"]
