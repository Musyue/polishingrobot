#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import tf

sys.path.append("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_offlineplanner/scripts")
from  planning.polishingrobot_mobileplatform_positions_planner import *

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from waypoints_paths_visualization_functions import *

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

class Renovationrobot_positions_visualization():
    def __init__(self,mat_path,parameterx,parametery,parameterz):
        self.mat_path=mat_path
        self.parameterx=parameterx 
        self.parametery=parametery 
        self.parameterz=parameterz
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    def renovation_planning_source_dict_generation(self):
        planning_source_dict={}
        rbmo=Renovation_BIM_Model_Opreating(self.mat_path,self.parameterx,self.parametery,self.parameterz)
        planning_source_dict=rbmo.get_mat_data_json1()
        return planning_source_dict


    def mobile_platform_visualization(self,visualization_num,mobileplatform_targetjoints):
        # visualization of target mobile platform positions and mobile platform path
        frame = 'map'
        mobileplatform_targepositions=np.array(mobileplatform_targetjoints)
        # print("mobileplatform_targepositions is:",mobileplatform_targepositions)
        scale1=np.array([0.2,0.2,0.2])
        color1=np.array([1.0,0.0,0.0])
        marker1,visualization_num=targetpositions_visualization(mobileplatform_targepositions, frame, visualization_num, scale1, color1)
        self.marker_pub.publish(marker1)
        # rospy.sleep(0.2)
        return visualization_num

    def mobile_platform_planningresults_visualization(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        visualization_num=1

        while not rospy.is_shutdown():
        # for i in range(4):
            rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
            rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))

            "visualization of mobile platform positions"
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]  
            print("mobiledata is:",mobiledata)
            visualization_num=self.mobile_platform_visualization(visualization_num, mobiledata)
            visualization_num=visualization_num+1
            mobile_base_point_count=mobile_base_point_count+1

            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0

            if plane_num_count>=len(planning_source_dict):
                plane_num_count=0
                mobile_base_point_count=0
                visualization_num=1
                rospy.loginfo("painting operation of whole room is over")
                # break
            # rate.sleep()



if __name__ == "__main__":
    rospy.init_node("paintingrobot_planningresults_visualization", anonymous=True)
    ratet=1
    rate = rospy.Rate(ratet)
    
    mat_path="/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_offlineplanner/matlab/scan_data2.mat"
    parameterx=0.2
    parametery=0.0
    parameterz=0.028625

    CUHK_renovationrobot=Renovationrobot_positions_visualization(mat_path,parameterx,parametery,parameterz)
    planning_source_dict=CUHK_renovationrobot.renovation_planning_source_dict_generation()
    CUHK_renovationrobot.mobile_platform_planningresults_visualization(planning_source_dict,rate)



