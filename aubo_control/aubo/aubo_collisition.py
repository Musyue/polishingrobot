#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
import time
import numpy
import os
import socket
from aubo_kienamatics import *
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import numpy.matlib
import json

from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt


from scipy.io import loadmat
class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.M=0
        self.N=0
    def Init_node(self):
        rospy.init_node("aubo_collision_cehck")

    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * pi / 180)
        return tuple(dd)
    def rad_to_degree(self,tuplelist):
        dd=[]
        for i in tuplelist:
            dd.append(i*180/math.pi)
        return dd
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'yue'
        js.name = ["base_joint1", 
                    "base_joint2",
                    "mobilebase_joint",
                    "rodclimbing_joint",
                    "shoulder_joint",
                    "upperArm_joint",
                    "foreArm_joint",
                    "wrist1_joint",
                    "wrist2_joint",
                    "wrist3_joint"
                    ]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)
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
    def deg_to_rad(self,listdata):
        dd=[]
        for i in listdata:
            dd.append(i*pi/180)
        return dd
    def wall_viewpoint_planning(self,wall_num,wall_length,wall_width,R_Max_Aubo_Reachability,D_cameara2Base,D_wall2Base,FOV_length,FOV_width):
        last_data={}
        R_opreating=sqrt(R_Max_Aubo_Reachability**2-D_wall2Base**2)
        AB=sqrt(2)*R_opreating
        CD=AB
        AD=AB
        mobile_base_num_in_world=int(wall_length/AB)
        arm_base_num_in_world=int(wall_width/CD)
        #Step 1 caculate the opreating center,the point is from down to up
        
        # last_data={wall_num:{}}
        one_wall_data={}
        horizon_data={}
        view_M=AB/FOV_length #horizon
        view_N=AD/FOV_width #vertical
        
        for i in range(mobile_base_num_in_world):
            vertical_data={}
            for j in range(arm_base_num_in_world):
                mobile_base_l1=[D_wall2Base,0-AB/2*2*i,CD/2+CD/2*2*j]            #in base frame
                A=[D_wall2Base,0-AB/2*2*i+AB/2,CD/2+CD/2*2*j+AB/2]
                view_point={}
                y0=A[1]-AB/view_M*0.5
                z0=A[2]-AD/view_N*0.5
                count=0
                for k in range(int(view_M)):
                    y=y0-(AB/view_M)*0.5*2*k
                    for m in range(int(view_N)):
                        z=z0-(AD/view_N)*0.5*2*m
                        view_point.update({count:[D_cameara2Base,y,z]})
                        count+=1


                vertical_data.update({j:mobile_base_l1,"viewgroup"+str(j):view_point})
                #step2 caculate the viewpoint from opreating pose 

            horizon_data.update({i:vertical_data})
        one_wall_data.update({wall_num:horizon_data})
        print(one_wall_data)
        return one_wall_data


    def print_json(self,data):
        print(json.dumps(data, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False))

def main():
    
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)
    Aubo_k=Aubo_kinematics()
    count=0
    
    # Aub.print_json(Aub.wall_viewpoint_planning(1,10,3,1.5,0.4,1.2,0.54,0.36))
    ax = plt.axes(projection='3d')
    zline = np.linspace(0, 15, 1000)
    xline = np.sin(zline)
    yline = np.cos(zline)
    ax.plot3D(xline, yline, zline, 'gray')

    # 三维散点的数据
    zdata = 15 * np.random.random(100)
    xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
    ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
    k=input("input:")

if __name__ == '__main__':
    main()