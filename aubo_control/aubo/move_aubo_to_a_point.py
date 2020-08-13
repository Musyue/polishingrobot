#! /usr/bin/env python
# coding=utf-8
import math
from aubo_robotcontrol import *
from aubo_kienamatics import *
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def rad_to_degree(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*180/math.pi)
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
                # robot.set_joint_maxacc((0.05, 0.05, 0.05, 0.05, 0.05, 0.05))

                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((0.05, 0.05, 0.05, 0.05, 0.05, 0.05))
                acc_data=0.3
                robot.set_joint_maxacc((acc_data,acc_data,acc_data,acc_data,acc_data,acc_data))

                # 设置关节最大加速度
                velc_data=0.3
                robot.set_joint_maxvelc((velc_data,velc_data,velc_data,velc_data,velc_data,velc_data))
                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.3)
                logger.info("-------go-----to-----start-------step--01")
                # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)
                robot.set_end_max_line_velc(0.3)


                joint_radian = deg_to_rad((6.33,18.66,142.092,120.32,86.375,0.101))#((0,0,0,0,0,0))#开始right0
                # joint_radian=(1.722415465728325, 1.5901199098725218, -1.5249703901706138, -2.635440743255463, -0.11767780271747252, -0.47848867751268553)
                # robot.move_joint(joint_radian)
                kk=[[1.722415465728325, 1.5901199098725218, -1.5249703901706138, -2.635440743255463, -0.11767780271747252, -0.47848867751268553],
                    [-0.4185434149628584, -1.675165425825643, 1.850441910709101, -2.818228191521398, 2.035719756329554, -0.028894140103582266],
                    [0.57454373821649, -1.8404174422226927, 2.0202722387232868, -2.4852054158177, 1.0442258545432503, 0.029872370084840227], 
                    [1.2413384325661792, -1.7273901663007631, 1.9300610374377136, -2.772225022944557, 0.3802728060143217, 0.13448514099151332], 
                    [1.3992598074242188, -1.293568928293749, 1.7902520609308876, -0.3028802125156247, -0.22516899413089586, -2.9041101294795917],
                    [1.4565998910447222, -1.4449120097049963, 1.127975730089747, -0.8944870775391287, -0.17010313588752268, -2.821867837326133],
                    [1.722415465728325, 0.8393032377511815, -1.9955730001285117, -2.3552266810920206, -0.11767780271747252, -0.47848867751268553], 
                    [1.837846719177131, 0.8060853671012884, -2.447121725430698, -3.0094449260334284, -0.22640894367345155, -0.23945044230460866],
                    [1.2413384325661792, -0.283723293871172, 2.532992190810212, -0.4713680884118574, -0.3802728060143217, -3.00710751259828], 
                    [1.3992598074242188, -0.5392341267723282, 2.0309755053697405, -0.8164915695981918, -0.22516899413089586, -2.9041101294795917], 
                    [1.4565998910447222, -0.866713653025375, 1.370205289214275, -1.230455875094222, -0.17010313588752268, -2.821867837326133]]
                for i in range(len(kk)):
                    robot.move_joint(tuple(kk[i]))
                    time.sleep(6)
                    robot.move_joint(joint_radian)
                    time.sleep(6)



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