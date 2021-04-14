#!/bin/bash

OPT=${DOCKER_OPTION} ## -it --cpuset-cpus 0-2
iname=${DOCKER_IMAGE:-"yoheikakiuchi/robot_assembler"} ## name of image (should be same as in build.sh)
cname=${DOCKER_CONTAINER:-"cont_robot_assembler"} ## name of container (should be same as in exec.sh)

DEFAULT_USER_DIR="$(pwd)"

#VAR=${@:-"bash --rcfile /my_entryrc"}
VAR=${@:-"roslaunch robot_assembler robot_assembler.launch OUTPUT_DIR:=/userdir"}
##
#VAR=${@:-"roslaunch robot_assembler robot_assembler.launch OUTPUT_DIR:=/userdir ROBOT_NAME:=SAMPLE START_WITH:=/catkin_ws/src/robot_assembler/sample/SAMPLE.roboasm.l"}
#VAR=${@:-"roslaunch robot_assembler urdf_check.launch model:=/userdir/SAMPLE.urdf"}
#VAR=${@:-"roslaunch robot_assembler robot_assembler_gazebo.launch model:=/userdir/SAMPLE.urdf"}

## --net=mynetworkname ## share docker inside network with another docker containar
## docker inspect -f '{{.NetworkSettings.Networks.mynetworkname.IPAddress}}' cont_test_ra
## docker inspect -f '{{.NetworkSettings.Networks.mynetworkname.Gateway}}'   cont_test_ra

if [ "$DOCKER_ROS_IP" == "" ]; then
#    export DOCKER_ROS_IP=127.0.0.1
    export DOCKER_ROS_IP=localhost
fi

NET_OPT="--net=host --env=DOCKER_ROS_IP --env=DOCKER_ROS_MASTER_URI"
# for gdb
#NET_OPT="--net=host --env=DOCKER_ROS_IP --env=DOCKER_ROS_MASTER_URI --cap-add=SYS_PTRACE --security-opt=seccomp=unconfined"
#NET_OPT="--net=host --env=NVIDIA_DRIVER_CAPABILITIES --env=NVIDIA_VISIBLE_DEVICES"

if [ $(nvidia-smi | grep Xorg | wc -l) -gt 0 ]; then
    GPU="--gpus all"
else
    GPU=""
fi

xhost +local:
##xhost +si:localuser:$(id -u ${USER})
##xhost +si:localuser:root

docker rm ${cname}

docker run ${OPT}    \
    --privileged     \
    ${GPU}           \
    ${NET_OPT}       \
    --env="DOCKER_ROS_SETUP=/catkin_ws/devel/setup.bash" \
    --env="DISPLAY"  \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name=${cname} \
    --volume="${PROG_DIR:-$DEFAULT_USER_DIR}:/userdir" \
    -w="/userdir" \
    -u $(id -u ${USER}):$(id -g ${USER}) \
    ${iname} ${VAR}

xhost -local:
