#! /usr/bin/env python
# coding=utf-8
import math
from aubo_robotcontrol import *
from aubo_kienamatics import *
import random 
num = random.randint(-100,100)
print("num",num)
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
# def 
def main():
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
        ip = '192.168.1.11'
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
            
            ak47=Aubo_kinematics()
            startpoint=[-85.72,-69.938,94.134,71.518,79.937,-1.1657]
            # print(ak47.aubo_forward(startpoint))
            BTE=ak47.aubo_forward(startpoint)
            print("BTE",BTE)
            tcptrans=(-0.181374,-0.939456,-0.544751)
            Tnew=ak47.trans_2_tcp(tcptrans,BTE)
            print("Tnew",Tnew)
            ETcp=numpy.dot(numpy.matrix((BTE)).reshape((4,4)).I,numpy.matrix((Tnew)).reshape((4,4)))
            # print(ETcp)
            # print(ETcp.I)
            # rpy = (149.38 / 180.0 * pi, 15.64/ 180.0 * pi, -43.91 / 180.0 * pi)
            # print(rpy2r(rpy[0],rpy[1],rpy[2]).tolist())

            Newtt=ak47.rpy_trans_2_new_T((-157.20 / 180.0 * pi, 3.9978/ 180.0 * pi, 6.908 / 180.0 * pi),(-0.181374,-0.939456,-0.544751))
            print(Newtt)
            EtB=numpy.dot(numpy.matrix((Newtt)).reshape((4,4)),ETcp.I)
            print("---EtB---",ak47.martix_to_list(EtB.tolist()))
            EtBnew=ak47.martix_to_list(EtB.tolist())
            # print(ak47.degree_to_rad([-146.8159,-76.32,92.113,143.4119,94.116,-15.47]))
            pjoint=ak47.GetInverseResult(EtBnew,ak47.degree_to_rad(startpoint))
            print("test",tuple(pjoint))
            robot.move_joint(tuple(pjoint))
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
    main()
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))