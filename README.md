# robot-assembler
Support designing a robot using actuator module.

See new version as a choreonoid Plugin ( https://github.com/IRSL-tut/robot_assembler_plugin )

## Install
You should install ROS( https://www.ros.org/ ) before install robot_assembler
```
$ source /opt/ros/melodic/setup.bash
$ mkdir new_ws; cd new_ws
$ wstool init src \
https://github.com/agent-system/robot_assembler/raw/master/config/robot_assembler.rosinstall
```

OR you can merge existing workspace
```
$ wstool merge -t src \
https://github.com/agent-system/robot_assembler/raw/master/config/robot_assembler.rosinstall
```

Then, install dependant packages
```
wstool update -t src
rosdep install -q -y -r --from-paths src --ignore-src
```

Finally, build robot_assembler
```
catkin build robot_assembler robot_assembler_gui
```

Before using, source setup script
```
source your_ws/devel/setup.bash
```

## Document
https://github.com/agent-system/robot_assembler/blob/master/doc/Robot_Assembler%E8%AA%AC%E6%98%8E.pdf
