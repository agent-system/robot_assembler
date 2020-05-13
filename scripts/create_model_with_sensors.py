#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import argparse
import yaml
import math
import re
from pyquaternion import Quaternion
from tf import transformations as tfs

##q = Quaternion(angle=(ang*math.pi)/180, axis=ax)
def create_quaternion_from_ypr(yaw, roll, pitch):
    r_y = Quaternion(angle=yaw, axis=[0, 0, 1])
    r_p = Quaternion(angle=roll, axis=[0, 1, 0])
    r_r = Quaternion(angle=pitch, axis=[1, 0, 0])
    #q = ((r_y * r_p) * r_r)
    return ((r_y * r_p) * r_r)
def create_quaternion_from_ypr2(yaw, roll, pitch):
    r_y = Quaternion(angle=yaw, axis=[0, 0, 1])
    r_p = Quaternion(angle=roll, axis=[0, 1, 0])
    r_r = Quaternion(angle=pitch, axis=[1, 0, 0])
    #q = ((r_y * r_p) * r_r)
    return ((r_r * r_p) * r_y)

tfs.euler_from_quaternion(tfs.quaternion_about_axis(angle=0.1, axis=[1, 1, 0]))

def write_xacro_file(data, urdf_file, name='xacro', strm=sys.stdout):
    slst = []
    if 'sensors' in data:
        slst = data['sensors']
    print(
'''<?xml version="1.0"?>
<!-- XML namespaces -->
<robot xmlns:xi="http://www.w3.org/2001/XInclude"
       xmlns:gazebo="http://playerstage.sourceforge.net/gazebo/xmlschema/#gz"
       xmlns:model="http://playerstage.sourceforge.net/gazebo/xmlschema/#model"
       xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:body="http://playerstage.sourceforge.net/gazebo/xmlschema/#body"
       xmlns:geom="http://playerstage.sourceforge.net/gazebo/xmlschema/#geom"
       xmlns:joint="http://playerstage.sourceforge.net/gazebo/xmlschema/#joint"
       xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:rendering="http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering"
       xmlns:renderable="http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable"
       xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
       xmlns:physics="http://playerstage.sourceforge.net/gazebo/xmlschema/#physics"
       xmlns:xacro="http://ros.org/wiki/xacro"''' , file=strm)

    print('       name="%s" >'%(name), file=strm)
    print('  <xacro:include filename="$(find robot_assembler)/gazebo/gazebo_plugins.xacro" />', file=strm)
    print('  <xacro:include filename="%s" />'%(urdf_file), file=strm)
    for s in slst:
        sensor_name = None
        sensor_type = None
        parent_link = None
        translate = None
        rotate = None
        gazebo_frame = None
        gazebo_rate  = None
        gazebo_topic = None
        gazebo_joint = None
        cam_rotate = None
        if 'sensor_name' in s:
            sensor_name = s['sensor_name']
        if 'sensor_type' in s:
            sensor_type = s['sensor_type']
        if 'parent_link' in s:
            parent_link = s['parent_link']
        if 'translate' in s:
            translate = s['translate']
        if 'rotate' in s:
            ##rotate = s['rotate']
            tmp = s['rotate']
            tmp = re.split(r'\s+', tmp)
            ax = [float(tmp[0]), float(tmp[1]), float(tmp[2])]
            ang = float(tmp[3])*math.pi/180
            roll, pitch, yaw = tfs.euler_from_quaternion(tfs.quaternion_about_axis(angle=ang, axis=ax))
            rotate = '%f %f %f'%(roll,pitch,yaw)
            ## rotate for camera frame
            q = tfs.quaternion_about_axis(angle=ang, axis=ax)
            p = tfs.quaternion_about_axis(angle=120*math.pi/180, axis=[-0.57735, 0.57735, -0.57735])
            roll, pitch, yaw = tfs.euler_from_quaternion(tfs.quaternion_multiply(q, p))
            cam_rotate = '%f %f %f'%(roll,pitch,yaw)
        if 'gazebo_frame' in s:
            gazebo_frame = s['gazebo_frame']
        if 'gazebo_rate' in s:
            gazebo_rate  = s['gazebo_rate']
        if 'gazebo_topic' in s:
            gazebo_topic = s['gazebo_topic']
        if 'gazebo_joint' in s:
            gazebo_joint = s['gazebo_joint']
        ###
        if sensor_type == 'base_force6d' or sensor_type == 'force':
            print('  <xacro:add_gazebo_ros_ft_sensor ', end='', file=strm)
            #if parent_link:
            #    print('link_name="%s" '%(parent_link), end='', file=strm)
            if gazebo_topic:
                print('topic="%s" '%(gazebo_topic), end='', file=strm)
            #if gazebo_frame:
            #    print('frame="%s" '%(gazebo_frame), end='', file=strm)
            if gazebo_rate:
                print('rate="%s" '%(gazebo_rate), end='', file=strm)
            if gazebo_joint:
                print('joint_name="%s" '%(gazebo_joint), end='', file=strm)
            print('/>', file=strm)
        ###
        if sensor_type == 'camera':
            gazebo_type = None
            width = None
            height = None
            fov = None
            if 'gazebo_type' in s:
                gazebo_type = s['gazebo_type'] ## camera only
            if 'width' in s:
                width = s['width']  ## camera only
            if 'height' in s:
                height = s['height']## camera only
            if 'fov' in s:
                fov = s['fov']      ## camera only
            if not gazebo_type or gazebo_type == 'camera':
                print('  <xacro:add_gazebo_camera ', end='', file=strm)
            elif gazebo_type == 'depth':
                print('  <xacro:add_gazebo_depth_camera ', end='', file=strm)
            if translate or rotate:
                if not translate:
                    translate = '0 0 0'
                if not rotate:
                    rotate = '0 0 0'
                print('pose="%s %s" '%(translate, rotate), end='', file=strm)
                print('xyz="%s" '%(translate), end='', file=strm)
                if cam_rotate:
                    print('rpy="%s" '%(cam_rotate), end='', file=strm)
            if parent_link:
                print('link_name="%s" '%(parent_link), end='', file=strm)
            if gazebo_topic:
                print('topic="%s" '%(gazebo_topic), end='', file=strm)
            if gazebo_frame:
                print('frame="%s" '%(gazebo_frame), end='', file=strm)
            if gazebo_rate:
                print('rate="%s" '%(gazebo_rate), end='', file=strm)
            if width:
                print('width="%s" '%(width), end='', file=strm)
            if height:
                print('height="%s" '%(height), end='', file=strm)
            if fov:
                print('fov="%s" '%(fov), end='', file=strm)
            print('/>', file=strm)
        ###
        if sensor_type == 'gyro' or sensor_type == 'acceleration':
            print('  <xacro:add_gazebo_imu_sensor ', end='', file=strm)
            if translate or rotate:
                if not translate:
                    translate = '0 0 0'
                if not rotate:
                    rotate = '0 0 0'
                print('pose="%s %s" '%(translate, rotate), end='', file=strm)
            if parent_link:
                print('link_name="%s" '%(parent_link), end='', file=strm)
            if gazebo_topic:
                print('topic="%s" '%(gazebo_topic), end='', file=strm)
            if gazebo_frame:
                print('frame="%s" '%(gazebo_frame), end='', file=strm)
            if gazebo_rate:
                print('rate="%s" '%(gazebo_rate), end='', file=strm)
            print('/>', file=strm)
        ###
        if sensor_type == 'range':
            print('  <xacro:add_gazebo_range_sensor ', end='', file=strm)
            if translate or rotate:
                if not translate:
                    translate = '0 0 0'
                if not rotate:
                    rotate = '0 0 0'
                print('pose="%s %s" '%(translate, rotate), end='', file=strm)
                print('xyz="%s" '%(translate), end='', file=strm)
                print('rpy="%s" '%(rotate), end='', file=strm)
            if parent_link:
                print('link_name="%s" '%(parent_link), end='', file=strm)
            if gazebo_topic:
                print('topic="%s" '%(gazebo_topic), end='', file=strm)
            if gazebo_frame:
                print('frame="%s" '%(gazebo_frame), end='', file=strm)
            if gazebo_rate:
                print('rate="%s" '%(gazebo_rate), end='', file=strm)
            print('/>', file=strm)
    print('</robot>', file=strm)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='create xacro file from yaml')
    parser.add_argument('-U', '--urdf', default = '', type=str)
    parser.add_argument('-Y', '--yaml', default = '', type=str)
    parser.add_argument('-O', '--output', default = '', type=str)
    parser.add_argument('-N', '--robot-name', default = '', type=str)

    args = parser.parse_args()

    urdf_file = args.urdf
    yaml_file = args.yaml
    output_f  = args.output
    rname = args.robot_name

    if urdf_file == '':
        print('please set urdf filename with -U urdf_file')
        exit(1)
    if yaml_file == '':
        print('please set yaml filename with -Y yaml_file')
        exit(2)

    if not os.path.isfile(yaml_file):
        print('yaml_file(%s) is not a file'%(yaml_file))
        exit(4)

    f = open(yaml_file)
    data = yaml.load(f)
    f.close()

    if output_f:
        strm = open(output_f, mode='w')
    else:
        strm = sys.stdout

    write_xacro_file(data, name=rname, urdf_file=urdf_file, strm=strm)
