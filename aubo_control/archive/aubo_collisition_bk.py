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
    o1=[]
    o2=[]
    o3=[]
    o4=[]
    o5=[]
    o6=[]
    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)


    Aubo_k=Aubo_kinematics()

    count=0

    
    Aub.print_json(Aub.wall_viewpoint_planning(1,10,3,1.5,0.4,1.2,0.54,0.36))

    m = loadmat("/data/ros/yue_ws/src/aubo_control/aubo/data1.mat")
    print(len(m["camera_viewpoints_inlowerbaseframe"]))

    count_o=0
    q_sol_without_collistion_dict={}
    num_count=0
    flag=1
    judge_self_collision_list=[]
    judge_count_flag=0

    judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')
    rospy.logerr("judge =="+str(judge_self_collision_flag))

    judge_self_collision_list.append(judge_self_collision_flag)
    last_state=[]


    last_data_degree=[]
    last_data_rad=[]
    use_ref_rad_onece_flag=0
    while not rospy.is_shutdown():

        if flag==1:
            # temp=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            temp1=[0.0,0.0,0.0,0.0]
            Aub.pub_state(temp1+q_ref_rad)
            flag=0
        else:
            if count_o<len(m["camera_viewpoints_inlowerbaseframe"]):
                new_t=Aub.list_change_3_7_11(Aubo_k.aubo_forward(q_ref),m["camera_viewpoints_inlowerbaseframe"][count_o])
                q_dict=Aubo_k.GetInverseResult_without_ref(new_t)
                # rospy.loginfo(str(m["camera_viewpoints_inlowerbaseframe"][count_o]))
                
                rospy.loginfo(str(m["camera_viewpoints_inlowerbaseframe"][count_o]))
                if q_dict!=None:
                    print(len(q_dict))
                    if count< len(q_dict):
                        temp=[0.0,0.0,0.0,0.0]


                        Aub.pub_state(temp+q_dict[count])
                        # print(q_dict[count],count)
                        # print(Aub.rad_to_degree(q_dict[count]))

                        judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')

                        if count!=0:
                            rospy.loginfo("haha--> "+str(count_o))
                            rospy.logerr("last state pub judge =="+str(judge_self_collision_flag)+" "+str(q_dict[count-1]))
                        
                            if judge_self_collision_flag==False:
                                rospy.logerr("The o last point : "+str(count_o)+" judge_self_collision sol "+str(count-1)+" ["+str(judge_self_collision_flag)+"]")
                                # print(q_sol_without_collistion_dict,count)
                                # temp=[0.0,0.0,0.0,0.0]
                                # Aub.pub_state(temp+q_dict[count-1])

                                q_sol_without_collistion_dict.update({num_count:q_dict[count-1]})
                                num_count+=1
                        count+=1
                        
                    else:

                        # print(q_dict[count],count)
                        count=0
                        count_o+=1
                        num_count=0
                        # judge_self_collision_list=[]
                        # # print(q_sol_without_collistion_dict)
                        if len(q_sol_without_collistion_dict)!=0:
                            if use_ref_rad_onece_flag==0:
                                rets,q_choose=Aubo_k.chooseIKonRefJoint(q_sol_without_collistion_dict,q_ref_rad)
                                use_ref_rad_onece_flag=1
                            else:
                                rets,q_choose=Aubo_k.chooseIKonRefJoint(q_sol_without_collistion_dict,last_data_rad[-1])
                            # temp=[0.0,0.0,0.0,0.0]
                            # Aub.pub_state(temp+q_choose)
                            rospy.logerr("q_choose"+str(q_choose))
                            print(Aub.rad_to_degree(q_choose))
                            last_data_rad.append(q_choose)
                            last_data_degree.append(Aub.rad_to_degree(q_choose))
                            # flag=0
                        q_sol_without_collistion_dict={}
                        # break
                else:
                    count_o+=1
            else:
                print(last_data_rad) 
                print(last_data_degree)   
                
               



        rate.sleep()
            

if __name__ == '__main__':
    main()