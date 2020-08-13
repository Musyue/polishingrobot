# smarteye_embedded_ros

## This is a ros driver for SMART EYE ROS PKG, you can just run it or roslaunch

`
roslaunch smarteye_embedded_ros smarteye_demo.launch
`
### Take a frame

You can use rosparam to send open flag to take a frame,or you can send the parameter from code or ternimator.

`
rosparam set /smarteye_ros_demo/open_camera_flag 1
`

### Save ASCII mode to a File

First,you need send save parameter and then send open camera to get one frame. Please note,you need to change the launch file,add a abs directory in your linux system.

`
rosparam set /smarteye_ros_demo/save_to_pcd_flag  1
`


`
rosparam set /smarteye_ros_demo/open_camera_flag 1
`

# Results
## rviz show
![State_output](https://github.com/Musyue/smarteye_embedded_ros/blob/master/show/demo.png)
