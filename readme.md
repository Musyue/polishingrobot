THe finished works are shown as follows:
1. polishingrobot_yhl/smarteye_embedded_ros package:
the package aims to obtain 3d point cloud document: xx.pcd
2. /home/zy/Desktop/chengwanli/02.处理流程/1.体素滤波
the package aims to  filter the original 3d point cloud to be another pcd document with much less point cloud number 
3. /home/zy/Desktop/chengwanli/02.处理流程/2.区域生长提取平面
the package aims to obtain the plane parameters: a*x+b*y+c*z+d=0
4. /home/zy/Desktop/Point-Cloud-Processing-example/第十三章/3 greedy_projection
the function of this program contains (1) obtaining the plane parameters (2) obtain the mesh files: vtk file 
5. /home/zy/Desktop/Point-Cloud-Processing-example/VTK_writingstl
the function of this program contains obtaining the stl document, however this program has not been written.

The unfinished works are shown as follows: 
In the case of coverage path planning 
1. verifying the function of (5) to write the stl document 
2. path planning based on the stl document
3. calibration between the manipulator base and the camera frame to obtain the waypoints on the manipulator base frame 
4. inverse kinematic solutions of the manipulator to obtain the robot joints value 

In the case of coverage planning of defect points 
1. verifying the function of (5) to write the stl document 
2. detecting defect points positions: (1) the deep learning method (2) the typical PCL solution
3. path planning based on the stl document, such as the greedy algorithm 
4. calibration between the manipulator base and the camera frame to obtain the waypoints on the manipulator base frame 
5. inverse kinematic solutions of the manipulator to obtain the robot joints value

point 2 is the comparision between the case of coverage planning of defect points and the coverage path planning 


## 20200713
1. /home/zy/catkin_ws/src/polishingrobot_yhl/polishing_mobileplatform_planner: the planner for polishing operation to generate mobile platform positions
2. /home/zy/catkin_ws/src/polishingrobot_yhl/polishing_undervibration: the impedance controller for polishing operation
3. /home/zy/catkin_ws/src/polishingrobot_yhl/smarteye_embedded_ros: the package for structured light camera to generate 3D cloud points


## 20200727
1. new packages should be added, including polishingrobot_description, paintingrobot_moveit_config
2. new functions should be added, including:
2.1 mobile platform planner result should be visualized in RVIZ, including the 2d and 3d map, and polishing robot model
3. when step 1 and 2 are finished, other things to be done:
3.1 planning the jackup mechanism positions
3.2 planning camera viewpoints and paths between viewpoints 
    ===============================================================
note: the core one is viewpoints planning, using the octomap methodology.

## 20200804 AM
1. the package of polishingrobot_description and polishingrobot_moveit_config have been built up 
2. ros package of polishingrobot_planner should include below functions:
(1) visualize robot model and construction environment
(2) visualize each planning target, that is, the grid 

when indoor polishing begins, below functions are shown as follows:
(3) plan the mobile platform positions 

when the mobile platform reachs each positions, below functions are shows as follows
(4) plan viewpoint and visualize the viewpoint
(5) plan the collision-avoidance inv-kin solution for climbing mechanism and mainpulator with moveit and visulize it in RVIZ

when inspection ends and polishing begin, below functions are shown as follows:
(6) visualize the infect points 
(7) plan the collision-avoidance inv-kin solution for the manipulator with moveit and visualize it in RVIZ

## 20200804 PM
1. The above two packages have been added the endeffector information
2. another package should be built, the functions contains:
visualize robot model
visualize construction environment 
visualize target positions, 
The package name is: polishingrobot_offlineplanner

## 20200805 AMgo
1. The pacakge named polishingrobot_onlineplanner has been built up.
2. how to apply MOVEIT in real robot planning is still a problem 
3. how to apply OCTOMAP in real viewpoint planning is still a problem 
















