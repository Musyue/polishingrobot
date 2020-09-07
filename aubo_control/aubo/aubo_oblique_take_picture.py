#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool

import time
import numpy
import os


from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import yaml
import numpy as np
import numpy.matlib
import json

from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt


from scipy.io import loadmat

from rectpack import *
from moveit_msgs.msg import DisplayTrajectory
from aubo_kienamatics import *
# from rectpack.maxrects import MaxRectsBssf
class AuboObilquePicture():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.pub_movel = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
        self.pub_movej = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)

    def Init_node(self):
        rospy.init_node("aubo_oblique_takepicture")

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
    def pub_pose(self,pub,pub_data):
        print("pub_data",pub_data)
        P = Pose()
        P.position.x=pub_data[0]
        P.position.y=pub_data[1]
        P.position.z=pub_data[2]
        P.orientation.x=pub_data[3]
        P.orientation.y=pub_data[4]
        P.orientation.z=pub_data[5]
        P.orientation.w=pub_data[6]
        pub.publish(P)

    
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'aubo_10'
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

    def caculate_take_pose_in_joint_space(self,q_ref,z_max,step_differential):
        Aubo_ki=Aubo_kinematics()
        T_Start=Aubo_ki.aubo_forward(q_ref)#degree
        
        Z_min=T_Start[11]
        
        deta_d=(z_max-Z_min)/step_differential
        rospy.loginfo("Z_min"+str(Z_min)+" "+str(z_max)+" "+str(deta_d))
        d_temp=T_Start[11]
        last_T=[]
        count=0
        while(d_temp+deta_d*count<=z_max):

            New_T=self.list_change_3_7_11(T_Start,[T_Start[3],T_Start[7],d_temp+deta_d*count])
            
            last_T.append(New_T)
            rospy.loginfo("count"+str(count))
            count+=1
        rospy.loginfo("we will have oblique viewpoints :"+str(count))
        count=0
        q_list={}
        for i in range(len(last_T)):
            Q_inver=Aubo_ki.GetInverseResult_with_ref(last_T[i],self.deg_to_rad(q_ref))
            if Q_inver!=None:
                q_list.update({i:Q_inver})
        return q_list
            

    

def main():
    
    ratet=1
    Aub=AuboObilquePicture()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    q_ref=[23.033,-71.261,143.8354,165.27,77.05,-75.4]
    q_ref_max=[23.033,-3.3075,19.223,-27.29,77.05,-75.4391]
    q_ref_rad=Aub.deg_to_rad(q_ref)
    Aubo_ki=Aubo_kinematics()
    T_max=Aubo_ki.aubo_forward(q_ref_max)
    print("----->",T_max[11])
    z_max=T_max[11]
    # print(Aub.caculate_take_pose_in_joint_space(q_ref,z_max,10))
    q_move_list=Aub.caculate_take_pose_in_joint_space(q_ref,z_max,5)
    
    count=0
    count_jian=len(q_move_list)
    rospy.logerr("len(q_move_list)"+str(len(q_move_list)))
    rospy.set_param("/smarteye_ros_demo/save_pcd_name",1)
    open_once_flag=0
    while not rospy.is_shutdown():
        if len(q_move_list)!=0 and count<len(q_move_list):
            if count!=0:
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(4)

            rospy.loginfo("Num"+" "+str(count)+" "+"move_to"+" "+str(q_move_list[count]))
            Aub.pub_movej.publish("movej"+str(q_ref_rad+q_move_list[count]))
            count+=1
            time.sleep(3)
            
        if count>=len(q_move_list) and count_jian>0:
            if open_once_flag==0:
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(4)
                open_once_flag=1
            count_jian-=1
            rospy.logerr("move_to_back"+" "+str(q_move_list[count_jian]))
            Aub.pub_movej.publish("movej"+str(q_ref_rad+q_move_list[count_jian]))
            time.sleep(2)
        rate.sleep()

if __name__ == '__main__':
    main()