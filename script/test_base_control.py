#!/usr/bin/python
# coding=utf-8

# 测试cmd_vel
import os
import rospy
import tf
import time
import sys
import math
import serial
import string
import traceback
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import String


# class BaseControl is design for hardware base relative control
class BaseControl:
    def __init__(self):
        # Get params
        self.baseId = rospy.get_param('~base_id', 'base_footprint')
        self.odomId = rospy.get_param('~odom_id', 'odom')
        self.device_port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = int(rospy.get_param('~baudrate', '57600'))
        self.odom_freq = int(rospy.get_param('~odom_freq', '50'))
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.cmd_freq = float(rospy.get_param('~cmd_freq', '1'))
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')

        # define param
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.serialIDLE_flag = 0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        # 末次指令时间
        self.last_cmd_vel_time = rospy.Time.now()

        """
        # 连接底盘serial串口 Serial Communication
        try:
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=10)
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.logerr("Opening Serial Try Faild")
                traceback.print_exc()
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")
        # 底盘数据监听（裸串口方式）
        #self.timer_communication = rospy.Timer(rospy.Duration(1.0/500), self.subSerialCommunication)
        #self.timer_communication = rospy.Timer(rospy.Duration(1.0/500), self.subSerial)
        """

        
        #self.tf_broadcaster = tf.TransformBroadcaster()
        # test rospy sub
        #self.sub = rospy.Subscriber("/tank/data", String, self.subTankData, queue_size=10)
        # 监听move_base发布给底盘的移动命令，并发送给底盘串口执行
        #self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.subCmd, queue_size=20)
        # 定频发布/cmd_vel数据
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.timer_cmd = rospy.Timer(rospy.Duration(1.0/self.cmd_freq), self.pubCmd)
        # 定频发布里程计数据
        #self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)    #里程计数据发布对象
        #self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq), self.pubOdom)
        # 定频发布电量数据
        #self.battery_pub = rospy.Publisher(self.battery_topic, BatteryState, queue_size=3)  #电量数据发布对象
        #self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq), self.pubBattery)



    def pubCmd(self, event):
        msg = Twist()
        msg.linear.x = 0.155857156127   #0.01928486814345191 #-0.133198200144
        msg.angular.z = -0.128752999067    #-0.226007277507
        self.cmd_pub.publish(msg)
        rospy.loginfo("pub /cmd_vel data:" + str(msg))

    # test sub data
    def subTankData(self, data):
        rospy.loginfo("sub /tank/data data: " + str(data))

    # 底盘数据监听（裸串口方式）
    def subSerial(self, event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading) != 0:
                #for i in range(0, len(reading)):
                #    data = (int(reading[i].encode('hex'), 16))
                rospy.loginfo("get serial data:" + str(reading))
        else:
            #rospy.loginfo("timerCommunication is Empty!")
            pass
    
    
    # 监听move_base发布给底盘的vel_cmd移动命令，并发送给底盘串口(也可以底盘直接监听vel_cmd topic)  Subscribe vel_cmd call this to send vel cmd to move base
    def subCmd(self, data):
        self.trans_x = data.linear.x
        self.trans_y = data.linear.y
        self.rotat_z = data.angular.z
        self.last_cmd_vel_time = rospy.Time.now()
        rospy.loginfo("get cmd data:" + str(data))
        """
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x01) + \
            chr((int(self.trans_x*1000.0) >> 8) & 0xff) + chr(int(self.trans_x*1000.0) & 0xff) + \
            chr((int(self.trans_y*1000.0) >> 8) & 0xff) + chr(int(self.trans_y*1000.0) & 0xff) + \
            chr((int(self.rotat_z*1000.0) >> 8) & 0xff) + chr(int(self.rotat_z*1000.0) & 0xff) + \
            chr(0x00)
        outputdata = [0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        outputdata[4] = (int(self.trans_x*1000.0) >> 8) & 0xff
        outputdata[5] = int(self.trans_x*1000.0) & 0xff
        outputdata[6] = (int(self.trans_y*1000.0) >> 8) & 0xff
        outputdata[7] = int(self.trans_y*1000.0) & 0xff
        outputdata[8] = (int(self.rotat_z*1000.0) >> 8) & 0xff
        outputdata[9] = int(self.rotat_z*1000.0) & 0xff
        crc_8 = self.crc_byte(outputdata, len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel_cmd Command Send Faild! output: " + output)
        self.serialIDLE_flag = 0
        """


# main function
if __name__ == "__main__":
    try:
        rospy.init_node('base_control', anonymous=True)
        rospy.loginfo('base control ...')

        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
