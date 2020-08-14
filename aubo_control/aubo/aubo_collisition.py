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
    def wall_viewpoint_planning(self,wall_num,wall_length,wall_width,R_Max_Aubo_Reachability,D_wall2Camera,D_wall2Base,FOV_length,FOV_width,arm_base_height):
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
                mobile_base_l1=[D_wall2Base,0-AB/2*2*i,arm_base_height]            #in base frame
                A=[D_wall2Base,0-AB/2*2*i+AB/2,CD/2+CD/2*2*j+AB/2]
                view_point={}
                y0=A[1]-AB/view_M*0.5
                z0=A[2]-AD/view_N*0.5
                count=0
                for k in range(int(view_M)):
                    y=y0-(AB/view_M)*0.5*2*k
                    for m in range(int(view_N)):
                        z=z0-(AD/view_N)*0.5*2*m
                        view_point.update({count:[D_wall2Camera,y,z]})
                        count+=1


                vertical_data.update({j:mobile_base_l1,"viewgroup"+str(j):view_point})
                #step2 caculate the viewpoint from opreating pose 

            horizon_data.update({i:vertical_data})
        one_wall_data.update({wall_num:horizon_data})
        # print(one_wall_data)
        return one_wall_data


    def print_json(self,data):
        print(json.dumps(data, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False))
    def caculate_the_workspace_with_aubo_and_climb(self,wall2base,aubo_the_max_opreating_radius,base2ground,climb_value):
        """     ^ z
            A-------B
            -   -   -
        y<--------------
            -   -   -
            -   -   -
            D-------C
        The max rectangle length and width and four corner point coordinate in aubo base frame
        """
        R_aubo_opreating_Max=sqrt(aubo_the_max_opreating_radius**2-wall2base**2)
        The_max_square_len=sqrt(2)*R_aubo_opreating_Max
        #we can obtatin the lowest z 
        lowest_z=base2ground-The_max_square_len/2
        rospy.loginfo("The lowest aubo opreating z aix value is : "+str(lowest_z))
        #The highest side
        highest_z=base2ground+The_max_square_len/2+climb_value
        rospy.loginfo("The hightest aubo opreating z aix value is : "+str(highest_z))
        The_max_rectangle_len_wid=[The_max_square_len,The_max_square_len+climb_value]
        rospy.loginfo("The max aubo opreating rectangle length and width is : "+str(The_max_rectangle_len_wid))

        # Aubo_base_coordinate=[0.0,0.0,0.0]
        # The_aubo_base_projection_to_wall_coordinate=[wall2base,0,0]

        A=[wall2base,The_max_square_len/2,highest_z]
        B=[wall2base,-The_max_square_len/2,highest_z]

        C=[wall2base,The_max_square_len/2,lowest_z]
        D=[wall2base,-The_max_square_len/2,lowest_z]
        rospy.loginfo("The Aubo+climb opreating workspace 3D coordinate value in aubo base frame is show :")
        rospy.loginfo("A---> "+str(A))
        rospy.loginfo("B---> "+str(B))
        rospy.loginfo("C---> "+str(C))
        rospy.loginfo("D---> "+str(D))
        return [A,B,C,D,The_max_rectangle_len_wid]
def main():
    
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)
    Aubo_k=Aubo_kinematics()
    count=0
    rectangle_l=[]
    rectangle_w=[]
    rectangle_area=[]
    for i in np.arange(0,1.5,0.1):
        res_data=Aub.caculate_the_workspace_with_aubo_and_climb(i,1.5,1.2,0.4)
        rospy.loginfo("The area show:--->"+str(res_data[4][0]*res_data[4][1]))
        rectangle_l.append(res_data[4][0])
        rectangle_w.append(res_data[4][1])
        rectangle_area.append(res_data[4][0]*res_data[4][1])
    ax = plt.axes(projection='3d')
    zdata = np.array(rectangle_area)
    ydata = np.array(rectangle_w)
    xdata = np.array(rectangle_l)
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Blues_r')
    plt.show()





    # one_wall_data=Aub.wall_viewpoint_planning(1,10,3,1.5,0.7,1.1,0.54,0.36,1.2)#=Aub.print_json(Aub.wall_viewpoint_planning(1,10,3,1.5,0.4,1.2,0.54,0.36))
    
    # ax = plt.axes(projection='3d')
    # if len(one_wall_data[1])!=0:
    #     mobile_z_list=[]
    #     mobile_y_list=[]
    #     mobile_x_list=[]
    #     camera_z_list=[]
    #     camera_y_list=[]
    #     camera_x_list=[]
    #     for i in range(len(one_wall_data[1])):
    #         for j in range(len(one_wall_data[1][i])/2):
    #             print(one_wall_data[1][i][j])
    #             mobile_z_list.append(one_wall_data[1][i][j][2])
    #             mobile_y_list.append(one_wall_data[1][i][j][1])
    #             mobile_x_list.append(one_wall_data[1][i][j][0])
    #         for k in range(len(one_wall_data[1][i])/2):
    #             for m in range(len(one_wall_data[1][i]["viewgroup"+str(k)])):
    #                 camera_x_list.append(one_wall_data[1][i]["viewgroup"+str(k)][m][0])
    #                 camera_y_list.append(one_wall_data[1][i]["viewgroup"+str(k)][m][1])
    #                 camera_z_list.append(one_wall_data[1][i]["viewgroup"+str(k)][m][2])

    #     zdata = np.array(mobile_z_list)
    #     ydata = np.array(mobile_y_list)
    #     xdata = np.array(mobile_x_list)
    #     ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Blues_r')

    #     zdata = np.array(camera_z_list)
    #     ydata = np.array(camera_y_list)
    #     xdata = np.array(camera_x_list)
    #     ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='gray')
    #     # ax.plot3D(xline, yline, zline, 'gray')
    #     plt.show()


if __name__ == '__main__':
    main()