#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
import time
import numpy
import os
import socket
from aubo_kienamatics import *
import yaml
class MoveSmartEyeVisonControl():
    def __init__(self):
        self.configname='/data/ros/yue_ws/src/aubo_control/config/camera_aubo_config.yaml'
        self.yamlDic=None
        self.Opreating_yaml()
        self.SmartEye_bTc=self.yamlDic['TBC']
        self.aubo_my_kienamatics=Aubo_kinematics()
        # self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.tcp_socket.bind(("", self.yamlDic['ServerPort']))#192.168.1.23
        # self.tcp_socket.listen(128)
        # self.client_socket_client_addr = self.tcp_socket.accept()
        
        # self.client_scoket=client_socket
        
        self.Aubo_IP=self.yamlDic['AuboIP']
        self.maxacctuple=tuple(self.yamlDic['Aubomaxacctuple'])
        self.maxvelctuple=tuple(self.yamlDic['Aubomaxvelctuple'])
    def Init_node(self):
        rospy.init_node("aubo_smart_eye")
    def Opreating_yaml(self,):       
        yaml_path = self.configname#str("/data/ros/yue_wk_2019/src/mobile_robot/src/config/"+self.configname)
        # print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        # print "hhh",self.yamlDic
        file_data.close()
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)
    def Init_aubo_driver(self):
        # 初始化logger
        #logger_init()
        # 启动测试
        print("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        print("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = self.Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                print("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                #robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                # print("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                # print(io_config)

                # 获取控制柜用户DO
                io_config = robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                # print(io_config)
                # 当前机械臂是否运行在联机模式
                # print("robot online mode is {0}".format(robot.is_online_mode()))
        except RobotError,e:
            logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
        return robot
    def DisConnect_Aubo_No_ShutDown(self,auboRobot):
        # 断开服务器链接
        auboRobot.disconnect()
    def DisConnect_Aubo(self,auboRobot):
        # 断开服务器链接
        if auboRobot.connected:
            # 关闭机械臂
            auboRobot.robot_shutdown()
            # 断开机械臂链接
            auboRobot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("{0} test completed.".format(Auboi5Robot.get_local_time()))

    def Aubo_trajectory_init(self,robot):
        joint_status = robot.get_joint_status()
        # print("joint_status={0}".format(joint_status))
        # 初始化全局配置文件
        robot.init_profile()
        # 设置关节最大加速度
        robot.set_joint_maxacc(self.maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)

        # 设置关节最大加速度
        robot.set_joint_maxvelc(self.maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        # 设置机械臂末端最大线加速度(m/s)
        robot.set_end_max_line_acc(0.5)
        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        robot.set_end_max_line_velc(0.5)
    def Aubo_forward_kinematics(self,robot,jointangular):
        joint_radian = self.deg_to_rad(jointangular)
        fk_ret = robot.forward_kin(joint_radian)
        print("fk--------")
        print(fk_ret)
        return fk_ret
    def Aubo_inverse_kinematics(self,robot,jointangular,newpose,neworientaion_Quaternion):
        # 获取关节最大加速度
        print(robot.get_joint_maxacc())
        joint_radian = jointangular#self.deg_to_rad(jointangular)
        print("pose and ori")
        print(newpose)
        print(neworientaion_Quaternion)
        pos_test = newpose#(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)  # the right
        ori_test = neworientaion_Quaternion#(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
        print("----ik----after--------")
        ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
        print(ik_result)
        return ik_result
    def Aubo_Line_trajectory(self,robot,start_point,End_point,):
        joint_radian = self.deg_to_rad(start_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
        joint_radian = self.deg_to_rad(End_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_line(joint_radian)

    def Aubo_Move_to_Point(self,robot,jointAngular):
        joint_radian = self.deg_to_rad(jointAngular)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
    def transfer_2normal(self,normal_vector,trans_point):
        a=normal_vector[0]
        b=normal_vector[1]
        c=normal_vector[2]
        theta=math.atan(c/a)
        pha=math.acos(-b/math.sqrt(a*a+b*b+c*c))
        Rxph=[1,0,0,0,math.cos(pha),-math.sin(pha),0,math.sin(pha),math.cos(pha)]
        Rzthe=[math.cos(theta),-math.sin(theta),0,math.sin(theta),math.cos(theta),0,0,0,1]
        nRc=numpy.dot(numpy.matrix(Rxph).reshape((3,3)),numpy.matrix(Rxph).reshape((3,3)))
        print(Rxph)
        print(Rzthe)
        print(nRc)
        nRc_list=nRc.tolist()
        nRctemp=[]
        reslist=[]
        for i in range(len(nRc_list)):
            for j in range(len(nRc_list[i])):
                nRctemp.append(nRc_list[i][j])
        for i in range(len(nRctemp)):
            if i ==2:
                reslist.append(nRctemp[i])
                reslist.append(trans_point[0])
            elif i==5:
                reslist.append(nRctemp[i])
                reslist.append(trans_point[1])
            elif i==8:
                reslist.append(nRctemp[i])
                reslist.append(trans_point[2])
            else:
                reslist.append(nRctemp[i])

        return reslist+[0,0,0,1]
    def Get_bTp_from_SmartEye_WithOrientaion_In_Normal(self,normal_vector,point_data):#in Frame of camera
        cTp=self.transfer_2normal(normal_vector,point_data)
        return numpy.dot(numpy.matrix(self.SmartEye_bTc).reshape((4,4)),numpy.matrix(cTp).reshape((4,4)))
    def Get_bTp_from_SmartEye(self,point_data):#in Frame of camera
        #point_data=[0,0,0,1]
        # point_data_temp=list(point_data).append(1)
        # print point_data_temp
        # print point_data+[1]
        return numpy.dot(numpy.matrix(self.SmartEye_bTc).reshape((4,4)),numpy.matrix(point_data+[1]).reshape((4,1)))
    def caculate_eTcp_matrix_with_other_kienamatics(self,robot,jointangular,trans_bTcp):
        """
        jointangular:base on TCP point(deg)
        trans_bTcp: from tcp to base translation
        """
        bTe=self.Aubo_forward_kinematics(robot,jointangular)
        eTcp=numpy.dot(bTe.I,numpy.matrix(trans_bTcp).reshape((4,4)))
        return eTcp
    def list_change_3_7_11(self,inital_list_data,list_data):
        temp=[]
        for i in range(len(inital_list_data)):
            if i==3:
                temp.append(list_data[0])
            elif i==7:
                temp.append(list_data[1])
            elif i==11:
                temp.append(list_data[2])
            else:
                temp.append(inital_list_data[i])
        return temp
    def caculate_eTcp_matrix_with_my_kienamatics(self,jointangular,trans_bTcp):
        """
        jointangular:base on TCP point(deg)
        trans_bTcp: from tcp to base translation
        """

        bTe=self.aubo_my_kienamatics.aubo_forward(jointangular)
        print("bTe",numpy.matrix(bTe).reshape((4,4)))
        bTcp=self.list_change_3_7_11(bTe,trans_bTcp)
        print("bTcp",numpy.matrix(bTcp).reshape((4,4)))
        eTcp=numpy.dot(numpy.matrix(bTe).reshape((4,4)).I,numpy.matrix(bTcp).reshape((4,4)))
        return eTcp
    def caculate_bTe_from_bTcp_matrix_with_my_kienamatics(self,point_data,eTcp,jointangular):
        bTp=self.Get_bTp_from_SmartEye(point_data)#[4*1]
        print("bTp",bTp.tolist())
        list_data=bTp.tolist()
        bTe=self.aubo_my_kienamatics.aubo_forward(jointangular)

        bTcp_point=self.list_change_3_7_11(bTe,[list_data[0][0],list_data[1][0],list_data[2][0]])
        
        print(bTcp_point)
        bTe=numpy.dot(numpy.matrix(bTcp_point).reshape((4,4)),numpy.matrix(eTcp).reshape((4,4)).I)
        
        return self.array_to_list(bTe)
    def array_to_list(self,list_data):
        list_data=list_data.tolist()
        temp=[]
        for i in range(len(list_data)):
            for j in range(len(list_data[i])):
                temp.append(list_data[i][j])
        return temp
    def get_joint_rad_from_inv(self,newT,jointAngular):
        return self.aubo_my_kienamatics.GetInverseResult(newT,self.deg_to_rad(jointAngular))
    def rad_to_degree(self,data_list):
        temp=[]
        for i in range(len(data_list)):
            temp.append(data_list[i]*180/numpy.pi)
        return temp
    def tcp_connect_with_windows(self,):
        rospy.logerr("I'm in tcp connecting ")
        print(self.client_socket_client_addr)
        client_socket=self.client_socket_client_addr[0]
        recv_data=client_socket.recv(1024)
        print(recv_data)
        return recv_data
    def send_data_to_client(self,mesdata):
        self.client_scoket.send("GoGoGo".encode("utf-8"))
    def close_tcp(self,):
        self.tcp_socket.close()
    def close_client_socket(self,):
        self.client_scoket.close()
    def caculate_eTc_matrix_with_my_kienamatics(self,jointangular):
        """
        jointangular:base on TCP point(deg)
        bTc: from camera to base T matrix
        """

        bTe=self.aubo_my_kienamatics.aubo_forward(jointangular)
        print("bTe",numpy.matrix(bTe).reshape((4,4)))

        eTc=numpy.dot(numpy.matrix(self.SmartEye_bTc).reshape((4,4)).I,numpy.matrix(bTe).reshape((4,4)))
        return eTc
def main():
    
    Point_data_1=[-0.104937,  0.131739, 0.923667]#[0.07149108,-0.1309188,1.040412]#1.033339]#[0.15765,-0.05829,0.9410576]#[-0.239463,-0.0300859,0.983125]#[0.131,-0.242,0.903]#[0.119,-0.116,1.003]
    ratet=1
    Aub=MoveSmartEyeVisonControl()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    #相机向下是y,左相机为主为x，光轴为Z
    eTcp=Aub.caculate_eTcp_matrix_with_my_kienamatics(Aub.yamlDic['StartPoint'],Aub.yamlDic['StartPointXYZ_IN_TCP'])
    print("eTcp",numpy.matrix(eTcp).reshape((4,4)))
    eTc=Aub.caculate_eTc_matrix_with_my_kienamatics(Aub.yamlDic['StartPoint'])
    print("eTc",numpy.matrix(eTc).reshape((4,4)))
    try:
        Robot = Aub.Init_aubo_driver()
        Aub.Aubo_trajectory_init(Robot)
    except:
        rospy.loginfo("init aubo not OK")
    # try:
    while not rospy.is_shutdown():

        # Aub.Aubo_Move_to_Point(Robot,StartPoint)
        # Aub.Aubo_forward_kinematics(Robot,StartPoint)
    
        bTe_p1=Aub.caculate_bTe_from_bTcp_matrix_with_my_kienamatics(Point_data_1,eTcp,Aub.yamlDic['StartPoint'])
        print("bTe_p1",bTe_p1)
        joint_p1_in_jointspace=Aub.get_joint_rad_from_inv(bTe_p1,Aub.yamlDic['StartPoint'])
        print(joint_p1_in_jointspace)
        print(Aub.rad_to_degree(joint_p1_in_jointspace))
        print(Robot.forward_kin(Aub.rad_to_degree(joint_p1_in_jointspace)))
        # Aub.tcp_connect_with_windows()
        # print(Aub.yamlDic)
        Aub.Aubo_Move_to_Point(Robot,Aub.rad_to_degree(joint_p1_in_jointspace))
        rate.sleep()
    # except:
    #     pass
if __name__ == '__main__':
    main()