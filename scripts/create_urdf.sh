#!/bin/bash
# ./create_urdf.sh ../sample/kxr_rcb4robots/kxrl2l6a6h2.roboasm.l /tmp/hoge.urdf [ /tmp/hoge.l ] [ ../sample/kxr_rcb4robots/kxrl2l6a6h2.urdf.euscollada.fixed.yaml ]  [ robotname ]

ROBOASM_FILE=$1
URDF_FILE=$2
arg3_str=""
if [ "$3" != "" ]; then
    arg3_str=" :euslisp-model-file-name \"$3\""
fi
arg4_str=""
if [ "$4" != "" ]; then
    arg4_str=" :collada-yaml-file-name \"$4\""
fi
arg5_str=""
if [ "$5" != "" ]; then
    arg5_str=" :robotname \"$5\""
fi

prog_str="(progn (load \"package://robot_assembler/euslisp/compile-kxr-robot.l\")"
arg_str="(compile-kxr-robot \"${ROBOASM_FILE}\" \"${URDF_FILE}\" "$arg3_str$arg4_str$arg5_str" )(exit))"

roseus "$prog_str$arg_str"
