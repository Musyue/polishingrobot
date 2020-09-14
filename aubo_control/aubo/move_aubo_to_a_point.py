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


                #joint_radian = deg_to_rad((6.33,18.66,142.092,120.32,86.375,-78.97))#((0,0,0,0,0,0))#开始right0
                joint_radian_1=deg_to_rad((15.17,58.218,158.196,102.856,74.259,-91.6399))#32.64651996879213, -99.16010568455614, 86.69431346599761, 9.165040662417706, -56.80744529642997, 87.32765123652544))
                joint_radian_2 = deg_to_rad((23.033,-71.261,143.8354,155.90,77.05,-75.4))#20200826最低
                joint_radian_3 = deg_to_rad((23.033,-71.261,143.8354,155.90,77.05,-75.4))#20200826最低
                robot.move_joint(joint_radian_1)
                # print(joint_radian)
                robot.move_line(joint_radian_3)

                # robot.move_line(joint_radian_1)
                
                # joint_radian = deg_to_rad((15.17,58.218,158.196,102.856,74.259,-91.6399))#20200826最低
                #zuigao 23.033,-3.3075,19.223,-27.29,77.05,-75.4391
                # print(joint_radian)
                # robot.move_joint(joint_radian)
                # joint_radian={
                #     "0th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             -0.75418359041213989,
                #             0.98618650436401367,
                #             1.5586129426956177,
                #             0.57242637872695923,
                #             2.3249800205230713,
                #             -1.5707963705062866
                #         ],
                #         "1th_candidate_joint_solution" : 
                #         [
                #             -0.75418359041213989,
                #             -0.49881339073181152,
                #             -1.5586129426956177,
                #             -1.0597995519638062,
                #             2.3249800205230713,
                #             -1.5707963705062866
                #         ],
                #         "2th_candidate_joint_solution" : 
                #         [
                #             -2.387408971786499,
                #             0.49881336092948914,
                #             1.5586129426956177,
                #             1.0597995519638062,
                #             -2.3249797821044922,
                #             -1.5707962512969971
                #         ],
                #         "3th_candidate_joint_solution" : 
                #         [
                #             -2.387408971786499,
                #             -0.98618650436401367,
                #             -1.5586129426956177,
                #             -0.57242637872695923,
                #             -2.3249797821044922,
                #             -1.5707962512969971
                #         ]
                #     },
                #     "1th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             -0.75418359041213989,
                #             1.7494245767593384,
                #             2.5296814441680908,
                #             0.78025674819946289,
                #             2.3249800205230713,
                #             -1.5707963705062866
                #         ],
                #         "1th_candidate_joint_solution" : 
                #         [
                #             -2.387408971786499,
                #             -1.7494245767593384,
                #             -2.5296814441680908,
                #             -0.78025668859481812,
                #             -2.3249797821044922,
                #             -1.5707962512969971
                #         ]
                #     },
                #     "2th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             -0.69086533784866333,
                #             -0.065814010798931122,
                #             2.9023916721343994,
                #             2.0540239810943604,
                #             2.0382018089294434,
                #             -2.0998377799987793
                #         ],
                #         "1th_candidate_joint_solution" : 
                #         [
                #             -0.69086533784866333,
                #             -2.3666818141937256,
                #             -2.9023916721343994,
                #             -1.4498913288116455,
                #             2.0382018089294434,
                #             -2.0998377799987793
                #         ]
                #     },
                #     "3th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             0.41610381007194519,
                #             -0.080463767051696777,
                #             -2.343209981918335,
                #             0.87884634733200073,
                #             -1.1546924114227295,
                #             -1.5707962512969971
                #         ],
                #         "1th_candidate_joint_solution" : 
                #         [
                #             -1.8134235143661499,
                #             0.080463789403438568,
                #             2.343209981918335,
                #             -0.87884622812271118,
                #             2.89896559715271,
                #             -1.5707961320877075
                #         ]
                #     },
                #     "4th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             1.0012625455856323,
                #             1.7611805200576782,
                #             -1.9380122423171997,
                #             1.5077406167984009,
                #             0.93291878700256348,
                #             -0.73586559295654297
                #         ]
                #     },
                #     "5th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             -2.4507274627685547,
                #             0.065814025700092316,
                #             -2.9023916721343994,
                #             -2.0540242195129395,
                #             -2.0382015705108643,
                #             -1.0417548418045044
                #         ]
                #     },
                #     "6th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             -1.3281692266464233,
                #             -0.080463774502277374,
                #             -2.343209981918335,
                #             0.87884622812271118,
                #             -2.8989653587341309,
                #             -1.5707964897155762
                #         ],
                #         "1th_candidate_joint_solution" : 
                #         [
                #             2.7254889011383057,
                #             0.080463752150535583,
                #             2.343209981918335,
                #             -0.87884634733200073,
                #             1.154692530632019,
                #             -1.5707963705062866
                #         ],
                #         "2th_candidate_joint_solution" : 
                #         [
                #             2.7254889011383057,
                #             -2.0864806175231934,
                #             -2.343209981918335,
                #             2.8848631381988525,
                #             1.154692530632019,
                #             -1.5707963705062866
                #         ]
                #     },
                #     "7th_selected_viewpoint" : 
                #     {
                #         "0th_candidate_joint_solution" : 
                #         [
                #             2.1403300762176514,
                #             -1.7611805200576782,
                #             1.9380122423171997,
                #             -1.5077404975891113,
                #             -0.9329187273979187,
                #             -2.4057271480560303
                #         ]
                #     }
                # }
                # print(len(joint_radian))
                # for i in range(len(joint_radian)):
                #     print "=============================i==========="+str(i)
                #     # print(str(i)+"th_selected_viewpoint------->",joint_radian[str(i)+"th_selected_viewpoint"])
                #     for j in range(len(joint_radian[str(i)+"th_selected_viewpoint"])):
                #         #print(str(j)+"th_candidate_joint_solution------>",joint_radian[str(i)+"th_selected_viewpoint"][str(j)+"th_candidate_joint_solution" ])
                #         print(rad_to_degree(joint_radian[str(i)+"th_selected_viewpoint"][str(j)+"th_candidate_joint_solution"]))



                
                # joint_radian_00 = deg_to_rad((-93.748,-67.127,102.608491,70.617551,88.338668,-81.216891))
                # joint_radian_01 = deg_to_rad((-60.219,-52.997,124.22,78.62,93.657,-48.098))
                # joint_radian_02 = deg_to_rad((-7.199,5.678,128.549,30.61,99.071,5.017))
                # count=0
                # while count<100:
                    
                #     robot.move_joint(joint_radian_00)
                #     time.sleep(3)
                #     robot.move_joint(joint_radian_01)
                #     time.sleep(3)
                #     robot.move_joint(joint_radian_02)
                #     time.sleep(3)            
                #     robot.move_joint(joint_radian_00)
                #     time.sleep(3)      
                #     count+=1
                    # time.sleep(6)
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
            # robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__=="__main__":
    main(1)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))8