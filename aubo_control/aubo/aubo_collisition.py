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

from rectpack import *
# from rectpack.maxrects import MaxRectsBssf
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

        C=[wall2base,The_max_square_len/2,-lowest_z]
        D=[wall2base,-The_max_square_len/2,-lowest_z]
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
    # q_ref=[0,0,0,0,0,0]
    # q_ref_rad=q_ref
    Aubo_k=Aubo_kinematics()
    count=0
    rectangle_l=[]
    rectangle_w=[]
    rectangle_area=[]
    for i in np.arange(0,1.5,0.1):
        res_data=Aub.caculate_the_workspace_with_aubo_and_climb(i,1.5,1.2,0.)
        rospy.loginfo("The area show:--->"+str(res_data[4][0]*res_data[4][1]))
        rectangle_l.append(res_data[4][0])
        rectangle_w.append(res_data[4][1])
        rectangle_area.append(res_data[4][0]*res_data[4][1])
    # ax = plt.axes(projection='3d')
    # zdata = np.array(rectangle_area)
    # ydata = np.array(rectangle_w)
    # xdata = np.array(rectangle_l)
    # ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Blues_r')
    # plt.show()
    #540*800
    rectangles=[]
    # rectangles = [(100, 30), (40, 60), (30, 30),(70, 70), (100, 50), (30, 30)]
    bins = [(1790,1790)]#[(1790,2190)]
    A=[0.80000000000000004, 0.89721792224631802, 2.497217922246318]
    C=[0.40000000000000004, 0.89721792224631802, -0.30278207775368193]
    k=540.0/800#width/length
    for i in range(100):
        rectangles.append((300+i*2,int((300+i*2.0)/k)))
    print(rectangles)
    packer = newPacker(mode=PackingMode.Offline, 
         bin_algo=PackingBin.Global, 
        pack_algo=MaxRectsBssf,
        sort_algo=SORT_AREA, 
        rotation=False)

    # Add the rectangles to packing queue
    for r in rectangles:
        packer.add_rect(*r)

    # Add the bins where the rectangles will be placed
    for b in bins:
        packer.add_bin(*b)
    # Start packing
    packer.pack()
    # Obtain number of bins used for packing
    nbins = len(packer)

    # Index first bin
    abin = packer[0]

    # Bin dimmensions (bins can be reordered during packing)
    width, height = abin.width, abin.height

    # Number of rectangles packed into first bin
    nrect = len(packer[0])
    print nrect
    rect = packer[0][0]

    # rect is a Rectangle object
    x = rect.x # rectangle bottom-left x coordinate
    y = rect.y # rectangle bottom-left y coordinate
    w = rect.width
    h = rect.height
    # Full rectangle list
    all_rects = packer.rect_list()
    print all_rects



    fig = plt.figure()
    ax = fig.add_subplot(111)

    rect = plt.Rectangle((0.0,0.0),bins[0][0],bins[0][1],facecolor='r',edgecolor="black",linewidth=3)

    ax.add_patch(rect)
    new_coordinate=[]
    color_list=['r','b','y','p','w']
    Oyipie=[C[0],C[1]-bins[0][0]/2.0,C[2]-(-1.0*C[2])]
    area_sum=[]
    for rect in all_rects:
        # print(rect)
        b, x, y, w, h, rid = rect
        rect = plt.Rectangle((x, y), w, h,facecolor='w',edgecolor="black",linewidth=1)
        area_sum.append(w*h)
        ax.add_patch(rect)
        
        if (w,h) not in rectangles:
            print(w,h)
            new_coordinate.append([C[0],(x+h/2.0-C[1]*1000.0)/1000.0,(y+w/2.0+C[2]*1000.0)/1000.0])
        else:
            new_coordinate.append([C[0],(x+w/2.0-C[1]*1000.0)/1000.0,(y+h/2.0+C[2]*1000.0)/1000.0])
    area_sum_data=0.0
    for i in range(len(area_sum)):
        area_sum_data+=area_sum[i]
    rospy.loginfo("Occupy "+str(bins[0][0]*bins[0][1])+" percent "+str(area_sum_data/(1.0*bins[0][0]*bins[0][1])))
    print new_coordinate
    plt.xlim((0, 1900))
    plt.ylim((0, 2300))

    plt.show()
    judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')
    # rospy.logerr("judge =="+str(judge_self_collision_flag))


    last_state=[]


    last_data_degree=[]
    last_data_rad=[]
    use_ref_rad_onece_flag=0
    flag=1
    count_o=0
    q_sol_without_collistion_dict={}
    num_count=0
    while not rospy.is_shutdown():

        if flag==1:
            # temp=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            temp1=[0.0,0.0,0.0,0.0]
            Aub.pub_state(temp1+q_ref_rad)
            flag=0
        else:
            if count_o<len(new_coordinate):
                rospy.loginfo("The viewpoint-----> "+str(count_o)+" "+str(new_coordinate[count_o]))
                new_t=Aub.list_change_3_7_11(Aubo_k.aubo_forward(q_ref),new_coordinate[count_o])
                q_dict=Aubo_k.GetInverseResult_without_ref(new_t)
                if q_dict!=None:
                    print("q_dict----->",len(q_dict))
                else:
                    rospy.logerr("No solution----")
                if q_dict!=None:
                    
                    if count< len(q_dict):
                        temp=[0.0,0.0,0.0,0.0]


                        Aub.pub_state(temp+q_dict[count])
                        # print(q_dict[count],count)
                        # print(Aub.rad_to_degree(q_dict[count]))

                        judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')

                        if count!=0:
                            rospy.logerr("last state pub judge =="+str(judge_self_collision_flag)+" "+str(q_dict[count-1]))
                        
                            if judge_self_collision_flag==False:
                                rospy.loginfo("The o last point : "+str(count_o)+" judge_self_collision sol "+str(count-1)+" ["+str(judge_self_collision_flag)+"]")
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
                print(last_data_rad,len(last_data_rad))
                print(last_data_degree)   
                
               



        rate.sleep()




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