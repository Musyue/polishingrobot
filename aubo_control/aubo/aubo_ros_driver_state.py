#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
from aubo_control.msg import catersian_vel,physical_para

import numpy as np
import time
import re
import os
class AuboRosDriver():
    def __init__(self):
        Auboi5Robot.initialize()
        self.robot = Auboi5Robot()
        
        self.aubo_joint_pub = rospy.Publisher('aubo_joints', JointState, queue_size=10)
        self.aubo_pose_pub = rospy.Publisher('aubo_pose', Pose, queue_size=10)
        self.aubo_vel_pub = rospy.Publisher('aubo_vel', catersian_vel,queue_size=10)
        self.aubo_status_pub = rospy.Publisher('aubo_status',physical_para ,queue_size=10)

        self.move_to_point=[]
        self.move_line_points={}
        self.count=1
        self.array1=np.array([0.0,0.0,0.0])
        self.array2=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.record_time=0.0
    def Init_node(self):
        rospy.init_node("aubo_driver_node2")
    def obtain_aubo_info(self):
        current_pos = self.robot.get_current_waypoint()
        # obtain joint position
        current_joint_msg = JointState()
        current_joint_msg.header.frame_id = ""
        current_joint_msg.header.stamp = rospy.Time.now()
        current_joint_msg.position = current_pos['joint']
        current_joint_msg.effort = []
        #obtiain joint velocity
        time1=rospy.get_time()
        # print("time1",time1)
        if self.count==1:
            current_joint_msg.velocity=np.zeros(6)
        else:
            delta_t=time1-self.record_time
            # print("delta_time is:",delta_t)
            # print((current_joint_msg.position-self.array2)/delta_t)
            current_joint_msg.velocity=(current_joint_msg.position-self.array2)/delta_t
        self.array2=np.array(current_joint_msg.position)
        # print(np.array(current_joint_msg.position))
        # obtain catersian position
        current_pose_msg = Pose()
        current_pose_msg.position.x = current_pos['pos'][0]
        current_pose_msg.position.y = current_pos['pos'][1]
        current_pose_msg.position.z = current_pos['pos'][2]
        current_pose_msg.orientation.x = current_pos['ori'][0]
        current_pose_msg.orientation.y = current_pos['ori'][1]
        current_pose_msg.orientation.z = current_pos['ori'][2]
        current_pose_msg.orientation.w = current_pos['ori'][3]                
        # obtian catersian velocity
        catersian_velocity=catersian_vel()
        if self.count==1:
            catersian_velocity.vx=0.0
            catersian_velocity.vy=0.0
            catersian_velocity.vz=0.9
            catersian_velocity.v=0.0
        else:
            delta_t=time1-self.record_time
            #print("delta_time is:",delta_t)
            catersian_velocity.vx=(current_pose_msg.position.x-self.array1[0])/delta_t
            catersian_velocity.vy=(current_pose_msg.position.y-self.array1[1])/delta_t
            catersian_velocity.vz=(current_pose_msg.position.z-self.array1[2])/delta_t
            catersian_velocity.v=math.sqrt(catersian_velocity.vx**2+catersian_velocity.vy**2+catersian_velocity.vz**2)
        self.count=self.count+1
        # print("vx",catersian_velocity.vx)
        # print("vy",catersian_velocity.vy)
        # print("vz",catersian_velocity.vz)
        # print("v_total",catersian_velocity.v)

        self.array1[0]=current_pos['pos'][0]
        self.array1[1]=current_pos['pos'][1]
        self.array1[2]=current_pos['pos'][2]
        self.record_time=time1

        # obtain joint status
        joint_status_msg=physical_para()
        joint_status=self.robot.get_joint_status()

        joint_status_msg.current=np.array([joint_status['joint1']['current'],joint_status['joint2']['current'],joint_status['joint3']['current'],\
        joint_status['joint4']['current'],joint_status['joint5']['current'],joint_status['joint6']['current']])*0.001

        joint_status_msg.voltage=np.array([joint_status['joint1']['voltage'],joint_status['joint2']['voltage'],joint_status['joint3']['voltage'],\
        joint_status['joint4']['voltage'],joint_status['joint5']['voltage'],joint_status['joint6']['voltage']])

        joint_status_msg.temperature=np.array([joint_status['joint1']['temperature'],joint_status['joint2']['temperature'],joint_status['joint3']['temperature'],\
        joint_status['joint4']['temperature'],joint_status['joint5']['temperature'],joint_status['joint6']['temperature']])

        self.aubo_joint_pub.publish(current_joint_msg)
        self.aubo_pose_pub.publish(current_pose_msg)
        self.aubo_vel_pub.publish(catersian_velocity)
        self.aubo_status_pub.publish(joint_status_msg)


    def Tuple_string_to_tuple(self,tuplestring):
        tupletemp = re.findall(r'\-?\d+\.?\d*', tuplestring)
        resdata=[]
        for i in tupletemp:
            resdata.append(float(i))
        return tuple(resdata)

    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)

    def Init_aubo_driver(self,Aubo_IP,maxacctuple,maxvelctuple,ee_maxacc,ee_maxvelc,blend_radius):
        # 初始化logger
        #logger_init()
        # 启动测试
        rospy.loginfo("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        # Auboi5Robot.initialize()
        # # 创建机械臂控制类
        # robot = Auboi5Robot()
        # 创建上下文
        handle = self.robot.create_context()
        # 打印上下文
        rospy.loginfo("robot.rshd={0}".format(handle))
        try:
            # 链接服务器
            ip = Aubo_IP#'192.168.1.11'
            port = 8899
            result = self.robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                rospy.loginfo("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                # # 上电
                #robot.robot_startup()
                # # 设置碰撞等级
                # robot.set_collision_class(7)
                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)
                # 设置工具端ＩＯ_0为输出
                self.robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)
                # 获取工具端ＩＯ_0当前状态
                tool_io_status = self.robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                rospy.loginfo("tool_io_0={0}".format(tool_io_status))
                # 设置工具端ＩＯ_0状态
                self.robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)
                # 获取控制柜用户DI
                io_config = self.robot.get_board_io_config(RobotIOType.User_DI)
                # 输出DI配置
                rospy.loginfo(io_config)
                # 获取控制柜用户DO
                io_config = self.robot.get_board_io_config(RobotIOType.User_DO)
                # 输出DO配置
                rospy.loginfo(io_config)
                # 当前机械臂是否运行在联机模式
                rospy.loginfo("robot online mode is {0}".format(robot.is_online_mode()))
                self.Aubo_trajectory_init(self.robot,maxacctuple,maxvelctuple)
        except RobotError,e:
            rospy.logerr("{0} robot Event:{1}".format(self.robot.get_local_time(), e))
        return robot
    def AuboStartPower(self):
        self.robot.robot_startup()
    def AuboDownPower(self):
        self.robot.robot_shutdown()
    def DisConnect_Aubo_No_ShutDown(self):
        # 断开服务器链接
        self.robot.disconnect()
    def DisConnect_Aubo(self):
        # 断开服务器链接
        if self.robot.connected:
            # 关闭机械臂
            self.robot.robot_shutdown()
            # 断开机械臂链接
            self.robot.disconnect()
        # 释放库资源
        self.robot.uninitialize()
        rospy.loginfo("{0} test completed.".format(self.robot.get_local_time()))
    def Aubo_trajectory_init(self,joint_maxacctuple,joint_maxvelctuple,ee_maxacc,ee_maxvelc,blend_radius):
        joint_status = self.robot.get_joint_status()
        rospy.loginfo("joint_status={0}".format(joint_status))
        # initial file system
        self.robot.init_profile()
        #set joint max acc
        self.robot.set_joint_maxacc(joint_maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)
        #set joint max vel
        self.robot.set_joint_maxvelc(joint_maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        #set ee max acc
        self.robot.set_end_max_line_acc(ee_maxacc)
        #set ee max vel
        self.robot.set_end_max_line_velc(ee_maxvelc)
        #set blend radius for manipulator
        self.robot.set_blend_radius(blend_radius=blend_radius)


def main():
    ratet=30
    Aub=AuboRosDriver()

    Aub.Init_node()
    rate = rospy.Rate(ratet)
    IPP=rospy.get_param('aubo_ip')
    StartPoint=Aub.Tuple_string_to_tuple(rospy.get_param('aubo_start_point'))
    joint_maxacctuple=Aub.Tuple_string_to_tuple(rospy.get_param('joint_maxacc_tuple'))
    joint_maxvelctuple=Aub.Tuple_string_to_tuple(rospy.get_param('joint_maxvelc_tuple'))
    ee_maxacc=rospy.get_param('ee_maxacc')     
    ee_maxvelc=rospy.get_param('ee_maxvelc') 
    blend_radius=rospy.get_param('blend_radius')

    try:
        Aub.Init_aubo_driver(IPP,joint_maxacctuple,joint_maxvelctuple,ee_maxacc,ee_maxvelc,blend_radius)
    except:
        logger.error("Aubo robot disconnect,Please check!")
    try:
        while not rospy.is_shutdown():
            rospy.loginfo("aubo driver 2 is ok")
            Aub.obtain_aubo_info()
            rate.sleep()
    except:
        rospy.logerr("aubo driver 2 died")
if __name__ == '__main__':
    main()

