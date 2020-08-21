#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory
planning_list=[]
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)

def trajectory_plannin_callback(data):
    for i in range(len(data.trajectory[0].joint_trajectory.points)):
        print data.trajectory[0].joint_trajectory.points[i].positions
        temp_list=[]
        for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):
            if abs(data.trajectory[0].joint_trajectory.points[i].positions[j])<0.0001:
                temp_list.append(0.0)
            else:
                temp_list.append(data.trajectory[0].joint_trajectory.points[i].positions[j])
        planning_list.append((0,0,0,0,0,0)+tuple(temp_list))

def aubo_ros_test():
    joint_radian1 = (0.0,0,0,0,0,0,0.0,0,0,0,0,0.)

    pub1 = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    # pub2 = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    # pub3 = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)

    sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, trajectory_plannin_callback, queue_size=1)


    rospy.init_node('aubo_ros_test', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    count=0
    while not rospy.is_shutdown():
        if len(planning_list)!=0:
            if count<len(planning_list):
                rospy.logerr(str(count)+str(planning_list[count]))
                movej_points="movej"+str(planning_list[count])
                #movel_points="movel"+str(joint_radian1)+str(joint_radian2)
                # movet_points="movej"+str(joint_radian1)+str(joint_radian2)+str(joint_radian3)+str(joint_radian4)
                # pub1.publish(movej_points)
                # pub2.publish(movel_points)
                pub1.publish(movej_points)
            else:
                rospy.loginfo("Over")
            count+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        aubo_ros_test()
    except rospy.ROSInterruptException:
        pass