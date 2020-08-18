# robot-assembler
assemble robot using GUI

## Install
You should install ROS( https://www.ros.org/ ) before install robot_assembler

mkdir new_ws; cd new_ws
$ wstool init src \
https://github.com/agent-system/robot_assembler/raw/master/config/robot_assembler.rosinstall

OR you can merge existing workspace
$ wstool merge -t src \
https://github.com/agent-system/robot_assembler/raw/master/config/robot_assembler.rosinstall

rosdep install -q -y -r --from-paths src --ignore-src

catkin build robot_assembler
source devel/setup.bash

## Document
https://github.com/agent-system/robot_assembler/blob/master/doc/Robot_Assembler%E8%AA%AC%E6%98%8E.pdf
