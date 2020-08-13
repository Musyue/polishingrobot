#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
import time
import numpy
import os
from aubo_kienamatics import *
import yaml
import socket
import threading
import commands
import re
class MoveSmartEyeVisonControl():
    def __init__(self):
        self.configname='/data/ros/yue_ws/src/aubo_control/config/camera_aubo_config.yaml'
        self.yamlDic=None
        self.Opreating_yaml()
        self.SmartEye_bTc=self.yamlDic['TBC']
        self.EE_TCP_DIS=self.yamlDic['EE_DIS_TCP']
        self.start_point=self.yamlDic['StartPoint']
        self.aubo_my_kienamatics=Aubo_kinematics()
        self.camera_data=[]
        self.camera_dict={}
        # self.send_s1_flag = rospy.get_param("send_s1_flag")#USe for open camera
        
        self.Aubo_IP=self.yamlDic['AuboIP']
        self.maxacctuple=tuple(self.yamlDic['Aubominacctuple'])
        self.maxvelctuple=tuple(self.yamlDic['Aubominvelctuple'])
        self.cont=30
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
    def socket_read(self,socket_info,):
        while True:
            recv_data = socket_info.recv(1024)
            if recv_data:
                print(recv_data.decode('utf-8'))      #windows,gbk encoding
                # self.camera_data=recv_data
                
                nums=re.findall(r'-?\d+\.*\d*', recv_data)
                if len(nums)>=6:
                    returndata = {"x":float(nums[0]),"y":float(nums[1]),"z":float(nums[2]),"a":float(nums[3]),"b":float(nums[4]),"c":float(nums[5]),"d":float(nums[6]),"height":float(nums[7])}
                    self.camera_dict=returndata

    def socket_write(self,socket_info,):
        count=0
        while True:
            status,output=commands.getstatusoutput("rosparam get /move2_camera_ns/send_s1_flag")
            # send_s1_flag = rospy.get_param("send_s1_flag")#USe for open camera
            # print(output)
            # print("send_s1_flag---:%d",send_s1_flag)
            #here we need to a flag to open this function
            if int(output):
                socket_info.send("S1".encode('utf-8'))
                # time.sleep(1)
                count+=1
                print("go to next point %d",count)
            else:
                # print("waiting next camera opreating--")
                pass
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
                robot.set_collision_class(6)

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
        # print("fk--------")
        # print(fk_ret)
        return fk_ret
    def Aubo_inverse_kinematics(self,robot,jointangular,newpose,neworientaion_Quaternion):
        # 获取关节最大加速度
        print(robot.get_joint_maxacc())
        joint_radian = jointangular#self.deg_to_rad(jointangular)
        # print("pose and ori")
        # print(newpose)
        # print(neworientaion_Quaternion)
        pos_test = newpose#(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)  # the right
        ori_test = neworientaion_Quaternion#(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
        # print("----ik----after--------")
        ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
        # print(ik_result)
        return ik_result
    def Aubo_Line_trajectory(self,robot,start_point,End_point,):
        joint_radian = self.deg_to_rad(start_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
        joint_radian = self.deg_to_rad(End_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_line(joint_radian)
    def Get_bTp_from_SmartEye_WithOrientaion_In_Normal(self,point_data):#in Frame of camera
        return numpy.dot(numpy.matrix(self.SmartEye_bTc).reshape((4,4)),numpy.matrix(point_data+[1]).reshape((4,1)))
    def Aubo_Move_to_Point(self,robot,jointAngular):
        joint_radian = self.deg_to_rad(jointAngular)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
    def Aubo_move_track(self,robot,q_list_rad,joint_angular_first):
        self.Aubo_Move_to_Point(robot,joint_angular_first)
        # 设置机械臂末端最大线加速度(m/s)
        robot.set_end_max_line_acc(0.1)

        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        robot.set_end_max_line_velc(0.1)
        robot.set_joint_maxacc(self.maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)

        # 设置关节最大加速度
        robot.set_joint_maxvelc(self.maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        # 清除所有已经设置的全局路点
        robot.remove_all_waypoint()
        joint_radian =self.deg_to_rad(joint_angular_first)
        robot.add_waypoint(joint_radian)
        # 添加全局路点1,用于轨迹运动
        for i in range(len(q_list_rad)):
            robot.move_joint(q_list_rad[i])
            # time.sleep(10)
            # robot.add_waypoint(tuple(q_list_rad[i]))

        # 设置圆运动圈数
        # robot.set_circular_loop_times(1)

        # 圆弧运动
        # rospy.loginfo("move_track ARC_CIR")
        # robot.move_track(RobotMoveTrackType.ARC_CIR)

        # 清除所有已经设置的全局路点
        # robot.remove_all_waypoint()
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
    def transfer_2normal(self,normal_vector):
        a=normal_vector[0]
        b=normal_vector[1]
        c=normal_vector[2]
        theta=math.atan2(c,a)
        theta_p=math.atan2(a,c)
        pha=math.acos(b/math.sqrt(a*a+b*b+c*c))
        pha_p=math.asin(b/math.sqrt(a*a+b*b+c*c))
        phanew =pi/2+pha#-pha_p #-(pi/2-pha_p)
        newtheta = theta_p
        Rxph=[1,0,0,
        0,math.cos(phanew),-math.sin(phanew),
        0,math.sin(phanew),math.cos(phanew)]
        # Rzthe=[math.cos(theta),-math.sin(theta),0,math.sin(theta),math.cos(theta),0,0,0,1]
        Rythe=[math.cos(newtheta),0,math.sin(newtheta),
        0,1,0,-math.sin(newtheta),0,math.cos(newtheta)]
        # Rx_pi_2=[1,0,0,0,math.cos(-pi/2),-math.sin(-pi/2),0,math.sin(-pi/2),math.cos(-pi/2)]
        # nRc=numpy.dot(numpy.matrix(Rxph).reshape((3,3)),numpy.matrix(Rythe).reshape((3,3)))
        nRc=numpy.dot(numpy.matrix(Rythe).reshape((3,3)),numpy.matrix(Rxph).reshape((3,3)))
        # nRc=numpy.dot(numpy.matrix(Rx_pi_2).reshape((3,3)),numpy.dot(numpy.matrix(Rzthe).reshape((3,3)),numpy.matrix(Rxph).reshape((3,3))))
        # print(Rxph)
        # print(Rzthe)
        print(numpy.dot(nRc,numpy.matrix([0,0,1]).reshape((3,1))))
        print("abc",math.sin(theta)*math.sin(pha),math.cos(pha),math.cos(theta)*math.sin(pha))
        print("normal_vector",normal_vector)
        print("nRc",nRc)
        return nRc
    def caculate_eTcp_matrix_with_my_kienamatics(self,jointangular,trans_bTcp):
        """
        jointangular:base on TCP point(deg)
        trans_bTcp: from tcp to base translation
        """

        bTe=self.aubo_my_kienamatics.aubo_forward(jointangular)
        # print("bTe",numpy.matrix(bTe).reshape((4,4)))
        bTcp=self.list_change_3_7_11(bTe,trans_bTcp)
        # print("bTcp",numpy.matrix(bTcp).reshape((4,4)))
        eTcp=numpy.dot(numpy.matrix(bTe).reshape((4,4)).I,numpy.matrix(bTcp).reshape((4,4)))
        return eTcp
    def insert_new_yz(self,T,ny,nz):
        temp=[]
        for i in range(len(T)):
            if i==7:
                temp.append(ny)
            elif i==11:
                temp.append(nz)
            else:
                temp.append(T[i])
        return temp
    def caculate_circle_draw(self,num,radius,yz_center_pos):
        y = yz_center_pos[0] + radius * math.cos( 2 * math.pi * num / self.cont )
        z = yz_center_pos[1] + radius * math.sin( 2 * math.pi * num / self.cont)
        return  [y,z]
    def draw_circle_from_joint(self,num,joint_rad,radius):
        T=self.aubo_my_kienamatics.aubo_forward(self.rad_to_degree(joint_rad))
        yz_center_pos=[T[7],T[11]]
        New_T_res=[]
        for i in range(num):
            newcircle_yz=self.caculate_circle_draw(i,radius,yz_center_pos)
            # print(newcircle_xy)
            New_T_res.append(self.insert_new_yz(T,newcircle_yz[0],newcircle_yz[1]))
        return New_T_res
    def get_circle_q_list(self,num,joint_rad,radius):
        all_new_t=self.draw_circle_from_joint(num,joint_rad,radius)
        rospy.loginfo(joint_rad)
        q_list=[]
        for i in range(len(all_new_t)):
            if self.aubo_my_kienamatics.GetInverseResult(all_new_t[i],joint_rad)!=None:
                q_list.append(self.aubo_my_kienamatics.GetInverseResult(all_new_t[i],joint_rad))
        return q_list
    def move_circle_aubo(self,robot,num,joint_rad,radius):
        q_list=self.get_circle_q_list(num,joint_rad,radius)
        print(q_list)
        # for i in range(len(q_list)):
        self.Aubo_move_track(robot,q_list,self.rad_to_degree(joint_rad))
        # self.Aubo_Move_to_Point(robot,self.rad_to_degree(q_list[i]))
        # print("q_list_forcircle",str(q_list[i]))
        # time.sleep(0.05)
    def caculate_bTe_from_bTcp_matrix_with_my_kienamatics(self,nromal_vector,point_data,eTcp,jointangular):

        bTp=self.Get_bTp_from_SmartEye_WithOrientaion_In_Normal(point_data)#self.Get_bTp_from_SmartEye(point_data)#[4*1]
        print("bTp",bTp)
        temp_bRc=[]
        for i in range(len(self.SmartEye_bTc[:11])):
            if i==3:
                pass
            elif i==7:
                pass
            elif i==11:
                pass
            else:
                temp_bRc.append(self.SmartEye_bTc[i])
        # print("numpy.matrix(temp_bRc).reshape((3,3))",numpy.matrix(temp_bRc).reshape((3,3)))
        bRn=numpy.dot(numpy.matrix(temp_bRc).reshape((3,3)),self.transfer_2normal(nromal_vector))
        row_last=numpy.matrix([0,0,0,1]).reshape((1,4))
        newbrn=numpy.column_stack((bRn,bTp[:3]))
        bTp_matrix=numpy.row_stack((newbrn,row_last))
        print("bTp_matrix",bTp_matrix)
        # list_data=bTp.tolist()
        bTe=self.aubo_my_kienamatics.aubo_forward(jointangular)
        print("bTe",bTe)

        # bTcp_point=self.list_change_3_7_11(bTe,[list_data[0][0],list_data[1][0],list_data[2][0]])
        
        # print(bTcp_point)
        bTe=numpy.dot(bTp_matrix,numpy.matrix(eTcp).reshape((4,4)).I)
        print("bTe_new",bTe)
        # print(self.array_to_list(bTe))
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
    def regex_from_camera_data(self,strdata):
        # returndata={}
        nums=re.findall(r'\d+(?:\.\d+)?', strdata)
        print("num---",nums)
        returndata = {"x":float(nums[0]),"y":float(nums[1]),"z":float(nums[2]),"a":float(nums[3]),"b":float(nums[4]),"c":float(nums[5]),"d":float(nums[6]),"height":float(nums[7])}
        return returndata


def main():

    Point_data_1=[0.09538765,-0.09967687,0.8083981]#[-0.239463,-0.0300859,0.983125]#[0.131,-0.242,0.903]#[0.119,-0.116,1.003]
    ratet=1
    Aub=MoveSmartEyeVisonControl()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    #相机向下是y,左相机为主为x，光轴为Z
    eTcp=Aub.caculate_eTcp_matrix_with_my_kienamatics(Aub.yamlDic['StartPoint'],Aub.yamlDic['StartPointXYZ_IN_TCP'])
    # print numpy.matrix(eTcp).reshape((4,4))

    try:
        Robot = Aub.Init_aubo_driver()
        Aub.Aubo_trajectory_init(Robot)
    except:
        rospy.loginfo("init aubo not OK")
    try:
        Aub.move_circle_aubo(Robot,30,Aub.deg_to_rad(Aub.start_point),0.4)
    except:
        pass
    while not rospy.is_shutdown():
        print("Nothing")
        # try:
        #     Aub.move_circle_aubo(Robot,30,Aub.deg_to_rad(Aub.start_point),0.5)
        # except:
        #     pass

        # Aub.Aubo_Move_to_Point(Robot,Aub.yamlDic['StartPoint'])



                
        
        rate.sleep()
    # except:
    #     pass
if __name__ == '__main__':
    main()
