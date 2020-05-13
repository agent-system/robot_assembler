#!/bin/bash

URDF_FILE=$1
ROBOT_NAME=$(basename ${URDF_FILE} .urdf)
XACRO_FILE=${URDF_FILE}.xacro
URDF_URI=''

if [ $# -gt 1 ]; then
    package_name=$2
    target_dir=$(rospack find ${package_name})
    URDF_URI="\$(find ${package_name})/${URDF_FILE}"
    URDF_FILE=${target_dir}/${URDF_FILE}
    XACRO_FILE=${target_dir}/${XACRO_FILE}
fi

YAML_FILE=${URDF_FILE}.euscollada.yaml
EUS_FILE=$(dirname ${URDF_FILE})/${ROBOT_NAME}.l

rosrun euscollada collada2eus -I ${URDF_FILE} -C ${YAML_FILE} -O ${EUS_FILE}

if [ URDF_URI == '' ]; then
    rosrun robot_assembler create_model_with_sensors.py -N ${ROBOT_NAME} -Y ${YAML_FILE} -U ${URDF_FILE} -O ${XACRO_FILE}
else
    rosrun robot_assembler create_model_with_sensors.py -N ${ROBOT_NAME} -Y ${YAML_FILE} -U "${URDF_URI}" -O ${XACRO_FILE}
fi

#cat<<EOF >> ${EUS_FILE}
#(when (and (boundp 'euscollada-robot) (class euscollada-robot))
#  (let ((clst (remove-if-not #'(lambda (cl) (eq (send cl :super) euscollada-robot))
#                             (system::list-all-classes))))
#    (when (= (length clst) 1)
#      (setq *assembled-robot-class* (car clst)))
#))
#EOF
