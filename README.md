# robot_navigation
机器人导航（SLAM建图、定位、底盘控制等）

# ROS1
source /opt/ros/noetic/setup.bash
source /home/work/catkin_ws/devel/setup.bash

export ROS_IP=`hostname -I | awk '{print $1}'`
export ROS_HOSTNAME=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://`hostname -I | awk '{print $1}'`:11311

# bingda
#BASE_TYPE can Set As NanoRobot NanoCar NanoRobot_Pro NanoCar_Pro NanoCar_SE 4WD 4WD_OMNI
export BASE_TYPE=NanoRobot_Pro
#LIDAR_TYPE can set As rplidar/sclidar/rpliadr_super
#export LIDAR_TYPE=rplidar_super
export LIDAR_TYPE=rplidar
#CAMERA_TYPE can set As astrapro/csi72
export CAMERA_TYPE=astrapro
export SONAR_NUM=2

# privoxy
export ALL_PROXY=http://127.0.0.1:8118



# 1.教程
a.使用激光雷达RpLidar A1进行SLAM定位建图
https://blog.yanjingang.com/?p=4303

b.树莓派Pi4B+激光雷达SLAM建图环境搭建(Ubuntu20.04.3 + ROS Noetic)
https://blog.yanjingang.com/?p=5299

