#!/bin/bash

OPT=${DOCKER_OPTION} ## -it --cpuset-cpus 0-2
cname=${DOCKER_CONTAINER:-"cont_robot_assembler"} ## name of container (should be same as in run.sh)

VAR=${@:-"bash --rcfile /my_entryrc"}
if [ $# -eq 0 -a -z "$OPT" ]; then
    OPT=-it
fi

docker exec ${OPT}          \
       --privileged         \
       --env="DISPLAY"      \
       --env="QT_X11_NO_MITSHM=1" \
       --workdir="/userdir" \
       ${cname} ${VAR}
