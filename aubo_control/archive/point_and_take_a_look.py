#! /usr/bin/env python
# coding=utf-8
import math
import os
from aubo_robotcontrol import *
from aubo_kienamatics import *
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def main(test_count):
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))
    try:

        # 链接服务器
        ip = '192.168.1.115'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            # robot.robot_shutdown()
            #
            # # 上电
            robot.robot_startup()
            #
            # # 设置碰撞等级
            # robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            # logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

            # 获取控制柜用户DI
            io_config = robot.get_board_io_config(RobotIOType.User_DI)

            # 输出DI配置
            logger.info(io_config)

            # 获取控制柜用户DO
            io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            # logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                # logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                # robot.set_joint_maxacc((5.5, 5.5, 5.5, 5.5, 5.5, 5.5))
                #
                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((1.5, .5, 2.5, 2.5, 2.5, 2.5))
                # 设置关节最大加速度
                robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

                # 设置关节最大加速度
                robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.5)
                logger.info("-------go-----to-----start-------step--01")
                # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)
                robot.set_end_max_line_velc(0.5)
                
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(3)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                t_save=10
                time.sleep(3)

                joint_radian = deg_to_rad((23.33,59.92,153.939,90.712,69.41,1.07))#((0,0,0,0,0,0))#开始right0
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(3)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #1
                joint_radian = deg_to_rad((102.668,1.7223,142.53,139.6822,-9.2608,-0.687))#left1
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #2

                joint_radian = deg_to_rad((93.004,-31.334,156.41,97.04,1.47,91.056))#left2
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #3

                joint_radian = deg_to_rad((96.16,-33.616,163.94,95.046,-2.4067,-78.19622))#left3
                robot.move_joint(joint_radian)
                time.sleep(3)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #4

                joint_radian = deg_to_rad((90.00,-77.0939,135.78,107.825,-2.406,-78.195))#left4
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #5

                joint_radian = deg_to_rad((85.611,-106.7425,104.4639,105.769,-2.400,-73.97553))#left5
                robot.move_joint(joint_radian)
                time.sleep(5)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #6

                joint_radian = deg_to_rad((90.79,82.13,146.1558,133.0358,-0.3238,110.739))#right1
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #7

                joint_radian = deg_to_rad((96.1967,7.31,-144.93,45.29,-1.429,-18.664))#right2
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #8

                joint_radian = deg_to_rad((93.782,68.15,-130.93,26.358,-5.0317,-47.545))#right3
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #9

                joint_radian = deg_to_rad((103.392,114.308,-81.4896,133.593,-7.04969,-151.7972))#right4
                robot.move_joint(joint_radian)
                time.sleep(4)
                os.system("rosparam set /smarteye_ros_demo/open_camera_flag 1")
                time.sleep(t_save)
                #10


                #back to initial
                joint_radian = deg_to_rad((23.33,59.92,153.939,90.712,69.41,1.07))#((0,0,0,0,0,0))#开始right0
                robot.move_joint(joint_radian)
            # 断开服务器链接
            robot.disconnect()

    except RobotError, e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__=="__main__":
    main(1)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))