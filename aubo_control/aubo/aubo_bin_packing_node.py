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
class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.pub_target_pose_ = rospy.Publisher('/move_path_from_bin_packing', Pose, queue_size=10)
        self.pub_start_pose_ = rospy.Publisher('/move_path_from_bin_packing_start_pose', Pose, queue_size=10)
        self.planning_sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.trajectory_planning_callback, queue_size=1)
        self.M=0
        self.N=0
        self.planning_list=[]
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
        The_max_square_len=R_aubo_opreating_Max/sqrt(2)
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
    def trajectory_planning_callback(self,data):
        self.planning_list=[]
        for i in range(len(data.trajectory[0].joint_trajectory.points)):
            # print data.trajectory[0].joint_trajectory.points[i].positions
            temp_list=[]
            count_zero=0
            for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):
                if abs(data.trajectory[0].joint_trajectory.points[i].positions[j])<0.0000000001:
                    count_zero+=1
            if count_zero>=3: #more than three zero will append to the final path list
                for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):

                    if abs(data.trajectory[0].joint_trajectory.points[i].positions[j])<0.0001:
                        temp_list.append(0.0)
                    else:
                        temp_list.append(data.trajectory[0].joint_trajectory.points[i].positions[j])

            self.planning_list.append((0,0,0,0,0,0)+tuple(temp_list))
    def rotation_2_quaterion(self,r):
        """
        R=[0 0 0 ->0
           0 0 0 ->1
           0 0 0] ->2
        """
        temp1=r[0]+r[4]+r[8]+1.0
        if abs(temp1)<0.000001:
            temp1=0.0
        w=1.0/2*sqrt(temp1)
        x=(r[7]-r[5])/(4*w)
        y=(r[2]-r[6])/(4*w)
        z=(r[3]-r[1])/(4*w)
        return [x,y,z,w]
    def get_orientation_2quaterion(self,q):#q is degree unit
        Aub=Aubo_kinematics()
        R=[]
        for i in range(9):
            R.append(0.0)
        T=Aub.aubo_forward(q)
        R[0]=T[0]
        R[1]=T[1]
        R[2]=T[2]
        R[3]=T[4]
        R[4]=T[5]
        R[5]=T[6]
        R[6]=T[8]
        R[7]=T[9]
        R[8]=T[10]
        print "========================"
        print T

        # print R
        Q_ua=self.rotation_2_quaterion(R)
        rospy.loginfo("quaterion"+str(Q_ua))
        return Q_ua
    def caculate_camera_FOV(self,d):
        """
        d-->mm
        """
        wl1=tan(abs(-20.826)*pi/180)*d+100
        wl2=tan(abs(28.826)*pi/180)*d-100
        wl3=tan(abs(22.3)*pi/180)*d
        wr1=tan(abs(26.826)*pi/180)*d-100
        wr2=tan(abs(-18.826)*pi/180)*d+100
        wr3=tan(abs(22.3)*pi/180)*d
        wl=[wl1,wl2,wl3]
        wr=[wr1,wr2,wr3]
        
        W=min(wl)+min(wr)
        # print("wl+wr",wl,wr,W)
        h1=tan(18.32*pi/180)*d*2.0
        h2=tan(14.4*pi/180)*d*2.0
        h=[h1,h2]
       
        H=min(h)
        # print("h1+h2",h,H)
        return [W,H]


def main():
    
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)
    ref_quaternion=Aub.get_orientation_2quaterion(q_ref)
    print(ref_quaternion)
    count=0
    rectangle_l=[]
    rectangle_w=[]
    rectangle_area=[]
    bins_w_h=[0,0]
    for i in np.arange(0,1.5,0.1):
        res_data=Aub.caculate_the_workspace_with_aubo_and_climb(i,1.5,1.2,0.)
        rospy.loginfo("The area show:--->"+str(res_data[4][0]*res_data[4][1]))
        rectangle_l.append(res_data[4][0])
        rectangle_w.append(res_data[4][1])
        rectangle_area.append(res_data[4][0]*res_data[4][1])
        if i ==0.8:
            bins_w_h[0]=rectangle_w
            bins_w_h[1]=rectangle_l
            # print("bins_w_h[0]",bins_w_h[0],bins_w_h[1])

    # ax = plt.axes(projection='3d')
    # zdata = np.array(rectangle_area)
    # ydata = np.array(rectangle_w)
    # xdata = np.array(rectangle_l)
    # ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Blues_r')
    # plt.show()
    #540*800
    rectangles=[]
    # rectangles = [(100, 30), (40, 60), (30, 30),(70, 70), (100, 50), (30, 30)]
    bins = [(int(bins_w_h[0][0])*1000,int(bins_w_h[0][1])*1000)]#[(1790,2190)]#base2wall=1.0
    # print(bins)
    A=[0.80000000000000004, 0.89721792224631802, 2.497217922246318]
    C=[0.40000000000000004, 0.89721792224631802, -0.30278207775368193]
    differential_num=100
    camera_distance_min=500.0 #mm
    camera_distance_max=1100.0
    deta_d=(camera_distance_max-camera_distance_min)/differential_num
    for i in range(differential_num):
        wh=Aub.caculate_camera_FOV(camera_distance_min+deta_d*i)
        # print(i,wh)
        rectangles.append((wh[0],wh[1]))
    # print(rectangles)
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
    # print nrect
    rect = packer[0][0]

    # rect is a Rectangle object
    x = rect.x # rectangle bottom-left x coordinate
    y = rect.y # rectangle bottom-left y coordinate
    w = rect.width
    h = rect.height
    # Full rectangle list
    all_rects = packer.rect_list()
    # print all_rects



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
            # print(w,h)
            new_coordinate.append([C[0],(x+h/2.0-C[1]*1000.0)/1000.0,(y+w/2.0+C[2]*1000.0)/1000.0])
        else:
            new_coordinate.append([C[0],(x+w/2.0-C[1]*1000.0)/1000.0,(y+h/2.0+C[2]*1000.0)/1000.0])
    area_sum_data=0.0
    for i in range(len(area_sum)):
        area_sum_data+=area_sum[i]
    rospy.loginfo("Occupy "+str(bins[0][0]*bins[0][1])+" percent "+str(area_sum_data/(1.0*bins[0][0]*bins[0][1])))
    print("new_coordinate",new_coordinate)
    plt.xlim((0, 1900))
    plt.ylim((0, 2300))

    plt.show()
    


    last_state=[]


    last_data_degree=[]
    last_data_rad=[]
    use_ref_rad_onece_flag=0
    flag=1
    count_o=0
    q_sol_without_collistion_dict={}
    num_count=0
        
    while not rospy.is_shutdown():
    
        viewpoint_count=rospy.get_param('/viewpoint_num')
        planning_over_flag=rospy.get_param('planning_over_flag')
        # if count_o<len(new_coordinate):
        if ref_quaternion!=None:
            rospy.loginfo("The viewpoint-----> "+str(viewpoint_count)+" "+str(new_coordinate[viewpoint_count]+ref_quaternion))
            if planning_over_flag:
                rospy.logerr("planning path is over----")
                rospy.set_param("planning_over_flag",0)
                # count_o+=1
            else:
                Aub.pub_pose(Aub.pub_target_pose_,new_coordinate[viewpoint_count]+ref_quaternion)
                # count_o+=1
                rospy.loginfo("wait---the planning ok")
            
        rate.sleep()

if __name__ == '__main__':
    main()