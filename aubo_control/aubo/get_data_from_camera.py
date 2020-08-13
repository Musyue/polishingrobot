#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool

import time
import numpy
import os
import socket

import yaml
import socket

class GETPIXFROMSMARTEYE():
    def __init__(self):
        self.configname='/data/ros/code/test_ws/src/aubo_control/config/camera_tcp_config.yaml'
        self.yamlDic=None
        self.Opreating_yaml()
        self.ServerPort=self.yamlDic['ServerPort']

    def Init_node(self):
        rospy.init_node("tcp_smart_eye")
    def Opreating_yaml(self,):       
        yaml_path = self.configname#str("/data/ros/yue_wk_2019/src/mobile_robot/src/config/"+self.configname)
        # print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        # print "hhh",self.yamlDic
        file_data.close()
    
def main():
    ratet=0.1
    TCPpix=GETPIXFROMSMARTEYE()
    TCPpix.Init_node()
    rate = rospy.Rate(ratet)

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
    tcp_socket.bind(("", TCPpix.ServerPort))
    #listen使套接字变成可以被动链接
    tcp_socket.listen(128)
    # 为多个客户端服务while True:
    count=0

    while not rospy.is_shutdown():
        #accept等待客户端的链接,返回一个客户端元组
        client_scoket , client_addr = tcp_socket.accept()
        print(client_addr)
        count+=1
        print("Send S1 to client to start cam---:the send num:%d",count)
        client_scoket.send("S1".encode("utf-8"))
        recv_data=client_scoket.recv(1024)
        print(recv_data)
        print("time out,go to next cam point")
        rate.sleep()

        client_scoket.close()
    tcp_socket.close()
if __name__ == '__main__':
    main()