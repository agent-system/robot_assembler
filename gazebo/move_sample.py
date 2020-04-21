#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('send_motion')
act_client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

act_client.wait_for_server()

# gen msg
traj_msg = FollowJointTrajectoryGoal()
traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
traj_msg.trajectory.joint_names = ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7', 'JOINT8', 'JOINT9', 'JOINT10', 'JOINT11']

##
traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, 0.20, -0.40, 0.20, 0,   0, 0, 0.20, -0.40, 0.20, 0 ], #姿勢1
                                                       time_from_start = rospy.Duration(2))) ## 前の姿勢から2sec
traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, 0.40, -0.80, 0.40, 0,   0, 0, 0.40, -0.80, 0.40, 0 ], #姿勢2
                                                       time_from_start = rospy.Duration(6)))## 前の姿勢から4sec
traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, 0.20, -0.40, 0.20, 0,   0, 0, 0.20, -0.40, 0.20, 0 ], #姿勢3
                                                       time_from_start = rospy.Duration(10)))## 前の姿勢から4sec
traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, 0.15, -0.3, 0.15, 0,   0, 0, 0.15, -0.3, 0.15, 0 ], #姿勢4
                                                       time_from_start = rospy.Duration(12)))## 前の姿勢から2sec

# send to robot arm
act_client.send_goal(traj_msg)

act_client.wait_for_result()

rospy.loginfo("done")
