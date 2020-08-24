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


                joint_radian = deg_to_rad((6.33,18.66,142.092,120.32,86.375,-78.97))#((0,0,0,0,0,0))#开始right0
                print joint_radian
                # joint_radian=(1.722415465728325, 1.5901199098725218, -1.5249703901706138, -2.635440743255463, -0.11767780271747252, -0.47848867751268553)
                robot.move_joint(joint_radian)
                # kk=[[-0.8490710612554864, -0.9361027637850183, 1.9316853102994873, 2.7811342906604644, 2.4651509825499893, -0.06931405769124765], [-0.8500243012442503, -0.14282031020143382, 1.594297632645345, 1.6503612340950298, 2.46610064165111, -0.0694459959018836], [0.14527212543695356, 0.5275344994287883, 2.131272850280343, 1.5492708305093936, 1.4727867997899189, 0.0036603378836455036], [0.14880793395508363, -1.033969577038759, 2.596632865341224, -2.7070696583576277, 1.4692562366799216, 0.003853825605619754], [1.4313029593061373, -0.9134534650901509, 1.7328445540700432, -0.779814157162547, -0.19423664266962337, -2.8638214877060286], [1.431121879473797, -2.2767319080491903, -1.853286139769545, 0.1391843969078037, 0.19441044904351568, 0.27750806252896165], [1.2879128862647917, -1.598204255400434, -2.0442431422431486, -0.6119430065391258, 0.33426298414521405, 0.15520122055424057], [1.431121879473797, -1.581094534886848, -1.4270260490147635, -0.13019288549975627, 0.19441044904351568, 0.27750806252896165]]
                # for i in range(len(kk)):
                #     print kk[i]
                #     robot.move_joint(tuple(kk[i]))
                #     time.sleep(6)
                    # robot.move_joint(joint_radian)
                    # time.sleep(6)



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