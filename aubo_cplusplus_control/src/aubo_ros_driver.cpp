#include "aubo_cplusplus_control/aubo_ros_driver.h"

using aubo10_ros_driver::AuboRosDriver;

AuboRosDriver::AuboRosDriver()
{

}
void AuboRosDriver::init_aubo_driver()
{
    ROS_INFO("=================Start init Aubo===================");
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
        /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录成功."<<std::endl;
    }

     std::cerr<<"机械臂初始化....."<<std::endl;


    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }

    /** 业务块 **/
    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();
    ROS_INFO("You can Enjoy the fun to control aubo use ros");

}
void AuboRosDriver::shutdown_aubo()
{
    sleep(10);
    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();
}
void AuboRosDriver::logout_aubo()
{
    sleep(10);
    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}
//for joint
void AuboRosDriver::aubo_joint_set_acc(double joint_acc)
{
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = joint_acc/180.0*M_PI;
    jointMaxAcc.jointPara[1] = joint_acc/180.0*M_PI;
    jointMaxAcc.jointPara[2] = joint_acc/180.0*M_PI;
    jointMaxAcc.jointPara[3] = joint_acc/180.0*M_PI;
    jointMaxAcc.jointPara[4] = joint_acc/180.0*M_PI;
    jointMaxAcc.jointPara[5] = joint_acc/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

}
void AuboRosDriver::aubo_joint_set_vel(double joint_vel)
{
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = joint_vel/180.0*M_PI;
    jointMaxVelc.jointPara[1] = joint_vel/180.0*M_PI;
    jointMaxVelc.jointPara[2] = joint_vel/180.0*M_PI;
    jointMaxVelc.jointPara[3] = joint_vel/180.0*M_PI;
    jointMaxVelc.jointPara[4] = joint_vel/180.0*M_PI;
    jointMaxVelc.jointPara[5] = joint_vel/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


}
//for end
void AuboRosDriver::aubo_end_set_acc(double line_acc,double angle_acc)
{
    double endMoveMaxAcc;
    endMoveMaxAcc = line_acc;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(angle_acc);
}//line and angle
void AuboRosDriver::aubo_end_set_vel(double line_acc,double angle_acc)
{
    double endMoveMaxVelc;
    endMoveMaxVelc = line_acc;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(angle_acc);
}//line and angle
/*
paramlist :(jointacc,jointvel,type)
type=0,typeslfag
type=1,movej
*/
bool AuboRosDriver::aubo_movej_one(double jointdeg[],double paramlist[])
{
    /** 接口调用: 设置关节型运动的最大加速度 ***/
    int typeslfag=-1;

    if ( paramlist[0] < 0.0 || paramlist[0] > 180.0)
    {
        ROS_ERROR("Aubo movej joint control acc can not more than 180.0 in degree,we will use default acc=20.0");
        this->aubo_joint_set_acc(20.0);
    }else
    {
        this->aubo_joint_set_acc(paramlist[0]);
    }
    
    if (paramlist[1] < 0.0 || paramlist[1] > 180.0)
    {
        ROS_ERROR("Aubo movej joint control vel can not more than 180.0 in degree,we will use default acc=20.0");
        /** 接口调用: 设置关节型运动的最大速度 ***/
        this->aubo_joint_set_vel(20.0);
    }else
    {
        this->aubo_joint_set_vel(20.0);
    }
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    typeslfag=paramlist[2];
    // printf("typeslfag--%d---%f\n",typeslfag,paramlist[2]);
    if(typeslfag==0)
    {
        /** 机械臂运动到零位姿态 **/
        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        ROS_ERROR("====Aubo will go back to the stand up state=======");
        ret = robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) fail,Please Check=======");
            return false;
        }
    }else
    {
        ROS_INFO("====Aubo will move to (%f,%f,%f,%f,%f,%f)=======",jointdeg[0],jointdeg[1],jointdeg[2],jointdeg[3],jointdeg[4],jointdeg[5]);
        Util::initJointAngleArray(jointAngle, jointdeg[0]/180.0*M_PI,  jointdeg[1]/180.0*M_PI,  jointdeg[2]/180.0*M_PI, jointdeg[3]/180.0*M_PI,jointdeg[4]/180.0*M_PI, jointdeg[5]/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (%f,%f,%f,%f,%f,%f) fail,Please Check=======",jointdeg[0],jointdeg[1],jointdeg[2],jointdeg[3],jointdeg[4],jointdeg[5]);
            return false;
        }
    }
    usleep(100);
}

/*
paramlist :(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type,blendradius,circularlooptimes)
type=0,typeslfag
type=1,movej
jointdeg[][6],a path 
*/
bool AuboRosDriver::aubo_movej_path(double jointdeg[][6],double paramlist[],int joint_path_count)
{
    /** 接口调用: 设置关节型运动的最大加速度 ***/
    int typeslfag;
    if ( paramlist[0] < 0.0 || paramlist[0] > 180.0)
    {
        ROS_ERROR("Aubo movej joint control acc can not more than 180.0 in degree,we will use default acc=20.0");
        this->aubo_joint_set_acc(20.0);
    }else
    {
        this->aubo_joint_set_acc(paramlist[0]);
    }
    
    if (paramlist[1] < 0.0 || paramlist[1] > 180.0)
    {
        ROS_ERROR("Aubo movej joint control vel can not more than 180.0 in degree,we will use default acc=20.0");
        /** 接口调用: 设置关节型运动的最大速度 ***/
        this->aubo_joint_set_vel(20.0);
    }else
    {
        this->aubo_joint_set_vel(20.0);
    }
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    typeslfag=paramlist[2];
    if(typeslfag==0)
    {
        /** 机械臂运动到零位姿态 **/
        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        ROS_ERROR("====Aubo will go back to the stand up state=======");
        ret = robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) fail,Please Check=======");
            return false;
        }
    }else
    {
        // for (int i = 0; i < 2; i++)
        // {
        //     printf("Q %d\n",i);
        //    for (int j = 0; j < 6; j++)
        //    {
        //        printf(" %f",jointdeg[i][j]);
        //    }
        //    printf("\n");
           
        // }
        // printf("paralist\n");
        // for (int i = 0; i < 3; i++)
        // {
        //     printf(" ,%f ",paramlist[i]);
        // }
        // printf("\n");
        
        
        int jointpath_num=joint_path_count;
        // printf("jointpath_num-----%d\n",jointpath_num);
        for (int i = 0; i < jointpath_num; i++)
        {
            ROS_INFO("====Aubo will move to Path Point[%d] (%f,%f,%f,%f,%f,%f)=======",i,jointdeg[i][0],jointdeg[i][1],jointdeg[i][2],jointdeg[i][3],jointdeg[i][4],jointdeg[i][5]);
            Util::initJointAngleArray(jointAngle, jointdeg[i][0]/180.0*M_PI,  jointdeg[i][1]/180.0*M_PI,  jointdeg[i][2]/180.0*M_PI, jointdeg[i][3]/180.0*M_PI,jointdeg[i][4]/180.0*M_PI, jointdeg[i][5]/180.0*M_PI);
            robotService.robotServiceJointMove(jointAngle, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("====Aubo move to (%f,%f,%f,%f,%f,%f) fail,Please Check=======",jointdeg[i][0],jointdeg[i][1],jointdeg[i][2],jointdeg[i][3],jointdeg[i][4],jointdeg[i][5]);
                return false;
            }
            usleep(100);
        }
        // delete[] jointdeg;
    }
    
}

void AuboRosDriver::MoveJ_One_Callback(const std_msgs::String::ConstPtr& msg)
 {
    ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movej_str="movej";
    std::string::size_type idx;
    idx=recv_str.find(movej_str);
    double jointdeg[6];
    double string2double_array[100];
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");
        // ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type,blendradius,circularlooptimes)}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        std::regex reg("-?(([0-9]\\d*\\.\\d*)|(0\\.\\d*[0-9]\\d*))");
        std::string strTemp = msg->data.c_str();
        const std::sregex_iterator end;
        int count_double=0;
        int error_test=0;
        for (std::sregex_iterator iter(std::cbegin(strTemp), std::cend(strTemp), reg); iter != end;++iter)
        {
            // std::cout << atof(iter->str().c_str()) << std::endl;
            if(count_double<6)
            {
                jointdeg[count_double]=atof(iter->str().c_str());
            }else
            {
                // printf("count_double%d",count_double);
                string2double_array[count_double-6]=atof(iter->str().c_str());
            }
            count_double++;
            error_test++;
        }
        double paramlist[3];
        for (int i = 0; i < 3; i++)
        {
            paramlist[i]=string2double_array[i];
        }
        if(error_test==9)
        {
            this->aubo_movej_one(jointdeg,paramlist);
        }else
        {
            ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");
        }
    }
 }

void AuboRosDriver::MoveJ_Path_Callback(const std_msgs::String::ConstPtr& msg)
 {
    // ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movej_str="movejp";
    std::string::size_type idx;
    idx=recv_str.find(movej_str);
    // double jointdeg[6];
    
    double string2double_array[3000];
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movejp,Command Like this movejp{joint=[(0,0,0,0,0,0),(0,0,0,0,0,0)...,(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");
        // ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type,blendradius,circularlooptimes)}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        std::regex reg("-?(([0-9]\\d*\\.\\d*)|(0\\.\\d*[0-9]\\d*))");
        std::string strTemp = msg->data.c_str();
        const std::sregex_iterator end;
        int count_double=0;

        for (std::sregex_iterator iter(std::cbegin(strTemp), std::cend(strTemp), reg); iter != end;++iter)
        {
            // std::cout << atof(iter->str().c_str()) << std::endl;
            string2double_array[count_double]=atof(iter->str().c_str());
            count_double++;
        }
        int count_double_temp=0;
        int joint_path_count=count_double/6;
        // printf("count_double/6 %d\n",count_double/6);
        if(joint_path_count>1 && count_double%6==3)
        {
            // double (* jointdeg)[6]=new double[count_double/6][6];
            double jointdeg[joint_path_count][6];
            for (int i = 0; i < joint_path_count; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    jointdeg[i][j]=string2double_array[count_double_temp];
                    count_double_temp++;
                }
            }
            double paramlist[3];
            paramlist[2]=string2double_array[count_double-1];
            paramlist[1]=string2double_array[count_double-2];
            paramlist[0]=string2double_array[count_double-3];
            this->aubo_movej_path(jointdeg,paramlist,joint_path_count);
            //for debug
            // for (int i = 0; i < count_double/6; i++)
            // {
            //     printf("JointQ %d\n",i);
            //     for (int j = 0; j < 6; j++)
            //     {
            //         printf(" ,%f ",jointdeg[i][j]);
            //     }
            //     printf("\n");
                
            // }
            // printf("paralist\n");
            // for (int i = 0; i < 3; i++)
            // {
            //     printf(" ,%f ",paramlist[i]);
            // }
            // printf("\n");
            
            
            
        }else
        {
            ROS_ERROR("Please publish the Right movejp,Command Like this movejp{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");
        }
    }
 }
 void AuboRosDriver::MoveL_Callback(const std_msgs::String::ConstPtr& msg)
 {
    // ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movel_str="movel";
    std::string::size_type idx;
    idx=recv_str.find(movel_str);
    // double jointdeg[6];
    
    double string2double_array[3000];
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movel,Command Like this movel{joint=[(0,0,0,0,0,0),(0,0,0,0,0,0)...,(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type)}");
        // ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type,blendradius,circularlooptimes)}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        std::regex reg("-?(([0-9]\\d*\\.\\d*)|(0\\.\\d*[0-9]\\d*))");
        std::string strTemp = msg->data.c_str();
        const std::sregex_iterator end;
        int count_double=0;

        for (std::sregex_iterator iter(std::cbegin(strTemp), std::cend(strTemp), reg); iter != end;++iter)
        {
            // std::cout << atof(iter->str().c_str()) << std::endl;
            string2double_array[count_double]=atof(iter->str().c_str());
            count_double++;
        }
        int count_double_temp=0;
        int joint_path_count=count_double/6-1;
        // printf("count_double/6 %d\n",count_double/6);
        if(joint_path_count>1 && count_double%6==1)
        {
            // double (* jointdeg)[6]=new double[count_double/6][6];
            double jointdeg[joint_path_count][6];
            for (int i = 0; i < joint_path_count; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    jointdeg[i][j]=string2double_array[count_double_temp];
                    count_double_temp++;
                }
            }
            double paramlist[7];
            paramlist[6]=string2double_array[count_double-1];
            paramlist[5]=string2double_array[count_double-2];
            paramlist[4]=string2double_array[count_double-3];
            paramlist[3]=string2double_array[count_double-4];
            paramlist[2]=string2double_array[count_double-5];
            paramlist[1]=string2double_array[count_double-6];
            paramlist[0]=string2double_array[count_double-7];
            this->aubo_movel(jointdeg,paramlist,joint_path_count);
        }else{
            ROS_ERROR("Please publish the Right movel,Command Like this movel{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");

        }
        
    }
 }
 void AuboRosDriver::MoveT_Callback(const std_msgs::String::ConstPtr& msg)
 {
    // ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movel_str="movet";
    std::string::size_type idx;
    idx=recv_str.find(movel_str);
    // double jointdeg[6];
    
    double string2double_array[3000];
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movet Command Like this movet{joint=[(0,0,0,0,0,0),(0,0,0,0,0,0)...,(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,blendradius,circularlooptimes,type)}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        std::regex reg("-?(([0-9]\\d*\\.\\d*)|(0\\.\\d*[0-9]\\d*))");
        std::string strTemp = msg->data.c_str();
        const std::sregex_iterator end;
        int count_double=0;

        for (std::sregex_iterator iter(std::cbegin(strTemp), std::cend(strTemp), reg); iter != end;++iter)
        {
            // std::cout << atof(iter->str().c_str()) << std::endl;
            string2double_array[count_double]=atof(iter->str().c_str());
            count_double++;
        }
        int count_double_temp=0;
        int joint_path_count=count_double/6-1;
        // printf("count_double/6 %d\n",count_double/6);
        if(joint_path_count>1 && count_double%6==3)
        {
            double jointdeg[joint_path_count][6];
            for (int i = 0; i < joint_path_count; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    jointdeg[i][j]=string2double_array[count_double_temp];
                    count_double_temp++;
                }
            }
            double paramlist[9];
            paramlist[8]=string2double_array[count_double-1];
            paramlist[7]=string2double_array[count_double-2];
            paramlist[6]=string2double_array[count_double-3];
            paramlist[5]=string2double_array[count_double-4];
            paramlist[4]=string2double_array[count_double-5];
            paramlist[3]=string2double_array[count_double-6];
            paramlist[2]=string2double_array[count_double-7];
            paramlist[1]=string2double_array[count_double-8];
            paramlist[0]=string2double_array[count_double-9];//type
            this->aubo_movet(jointdeg,paramlist,joint_path_count);
        }else{
            ROS_ERROR("Please publish the Right movel,Command Like this movel{joint=[(0,0,0,0,0,0)],paramlist=(jointacc,jointvel,jointveltype)}");

        }
        
    }
 }
/*
paramlist :(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,type)
type=0,typeslfag
type=1,movel
*/
bool AuboRosDriver::aubo_movel(double jointdeg[][6],double paramlist[],int joint_path_count)
{
    /** 接口调用: 设置关节型运动的最大加速度 ***/
    int typeslfag;
    if ( paramlist[0] < 0.0 || paramlist[0] > 180.0)
    {
        ROS_ERROR("Aubo movel joint control acc can not more than 180.0 in degree,we will use default acc=20.0");
        this->aubo_joint_set_acc(20.0);
    }else
    {
        this->aubo_joint_set_acc(paramlist[0]);
    }
    
    if (paramlist[1] < 0.0 || paramlist[1] > 180.0)
    {
        ROS_ERROR("Aubo movel joint control vel can not more than 180.0 in degree,we will use default acc=20.0");
        /** 接口调用: 设置关节型运动的最大速度 ***/
        this->aubo_joint_set_vel(20.0);
    }else
    {
        this->aubo_joint_set_vel(20.0);
    }
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    typeslfag=paramlist[6];
    if(typeslfag==0)
    {
        /** 机械臂运动到零位姿态 **/
        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        ROS_ERROR("====Aubo will go back to the stand up state=======");
        ret = robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) fail,Please Check=======");
            return false;
        }
    }else
    {
        ROS_INFO("====Aubo will start to move line =======");
        ROS_INFO("====First :---> Aubo will move to (%f,%f,%f,%f,%f,%f)=======",jointdeg[0][0],jointdeg[0][1],jointdeg[0][2],jointdeg[0][3],jointdeg[0][4],jointdeg[0][5]);
        Util::initJointAngleArray(jointAngle, jointdeg[0][0]/180.0*M_PI,  jointdeg[0][1]/180.0*M_PI,  jointdeg[0][2]/180.0*M_PI, jointdeg[0][3]/180.0*M_PI,jointdeg[0][4]/180.0*M_PI, jointdeg[0][5]/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (%f,%f,%f,%f,%f,%f) fail,Please Check=======",jointdeg[0][0],jointdeg[0][1],jointdeg[0][2],jointdeg[0][3],jointdeg[0][4],jointdeg[0][5]);
            return false;
        }
        robotService.robotServiceInitGlobalMoveProfile();
        if((paramlist[2]<0.0 || paramlist[2]>3.0 ) || (paramlist[4]<0.0 || paramlist[4]>180.0 ))
        {
            ROS_ERROR("Aubo movel endeffector line and angle acc can not more than 3.0 and 180,we will use default lineacc=1.0,angleacc=1.0");
            this->aubo_end_set_acc(1.0,1.0);
        }else{
            this->aubo_end_set_acc(paramlist[2],paramlist[4]);
        }
        if((paramlist[3]<0.0 || paramlist[3]>3.0 ) || (paramlist[5]<0.0 || paramlist[5]>180.0 ))
        {
            ROS_ERROR("Aubo movel endeffector line and angle vel can not more than 3.0 and 180,we will use default linevel=1.0,anglevel=1.0");
            this->aubo_end_set_vel(1.0,1.0);
        }else{
            this->aubo_end_set_vel(paramlist[3],paramlist[5]);
        }

        int jointpath_num=joint_path_count;
        for (int i = 0; i < jointpath_num; i++)
        {
            ROS_INFO("====Aubo will move to Path Point[%d] (%f,%f,%f,%f,%f,%f)=======",i,jointdeg[i][0],jointdeg[i][1],jointdeg[i][2],jointdeg[i][3],jointdeg[i][4],jointdeg[i][5]);
            Util::initJointAngleArray(jointAngle, jointdeg[i][0]/180.0*M_PI,  jointdeg[i][1]/180.0*M_PI,  jointdeg[i][2]/180.0*M_PI, jointdeg[i][3]/180.0*M_PI,jointdeg[i][4]/180.0*M_PI, jointdeg[i][5]/180.0*M_PI);
            robotService.robotServiceLineMove(jointAngle, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("====Aubo movel to (%f,%f,%f,%f,%f,%f) fail,Please Check=======",jointdeg[i][0],jointdeg[i][1],jointdeg[i][2],jointdeg[i][3],jointdeg[i][4],jointdeg[i][5]);
                return false;
            }
            usleep(100);
        }
    }
    
}
/*
                0       1       2           3           4           5              6               7        8
paramlist :(jointacc,jointvel,lineendacc,lineendvel,angleendacc,angleendvel,blendradius,circularlooptimes,type)
type=0,typeslfag
type=1,ARC_CIR
type=2,CARTESIAN_MOVEP
type=3,CARTESIAN_CUBICSPLINE
type=4,CARTESIAN_UBSPLINEINTP
type=5,JIONT_CUBICSPLINE
type=6,JOINT_UBSPLINEINTP
1 、subMoveMode 当 subMoveMode==ARC_CIR, CARTESIAN_MOVEP, CARTESIAN_CUBICSPLINE,
CARTESIAN_UBSPLINEINTP 时，该运动属于末端型运动；
当 subMoveMode==JIONT_CUBICSPLINE, JOINT_UBSPLINEINTP 时，该运动属于关节型运动；
当 subMoveMode==ARC_CIR 表示圆或者圆弧
当圆的圈数属性（ CircularLoopTimes）为 0 时，表示圆弧轨迹，
当圆的圈数属性（ CircularLoopTimes）大于 0 时，表示圆轨迹。
当 subMoveMode==CARTESIAN_MOVEP 表示 MOVEP 轨迹，需要用户这只交融半径的属性。
*/
bool AuboRosDriver::aubo_movet(double jointdeg[][6],double paramlist[],int joint_path_count)
{
    /** 接口调用: 设置关节型运动的最大加速度 ***/
    int typeslfag;
    if ( paramlist[0] < 0.0 || paramlist[0] > 180.0)
    {
        ROS_ERROR("Aubo movel joint control acc can not more than 180.0 in degree,we will use default acc=20.0");
        this->aubo_joint_set_acc(20.0);
    }else
    {
        this->aubo_joint_set_acc(paramlist[0]);
    }
    
    if (paramlist[1] < 0.0 || paramlist[1] > 180.0)
    {
        ROS_ERROR("Aubo movel joint control vel can not more than 180.0 in degree,we will use default acc=20.0");
        /** 接口调用: 设置关节型运动的最大速度 ***/
        this->aubo_joint_set_vel(20.0);
    }else
    {
        this->aubo_joint_set_vel(20.0);
    }
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    typeslfag=paramlist[8];
    if(typeslfag==0)
    {
        /** 机械臂运动到零位姿态 **/
        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        ROS_ERROR("====Aubo will go back to the stand up state=======");
        ret = robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) fail,Please Check=======");
            return false;
        }
    }else
    {
        ROS_INFO("====Aubo will start to move line =======");
        ROS_INFO("====Aubo will movej to the first Point (%f,%f,%f,%f,%f,%f)=======",jointdeg[0][0],jointdeg[0][1],jointdeg[0][2],jointdeg[0][3],jointdeg[0][4],jointdeg[0][5]);
        Util::initJointAngleArray(jointAngle, jointdeg[0][0]/180.0*M_PI,  jointdeg[0][1]/180.0*M_PI,  jointdeg[0][2]/180.0*M_PI, jointdeg[0][3]/180.0*M_PI,jointdeg[0][4]/180.0*M_PI, jointdeg[0][5]/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("====Aubo move to (%f,%f,%f,%f,%f,%f) fail,Please Check=======",jointdeg[0][0],jointdeg[0][1],jointdeg[0][2],jointdeg[0][3],jointdeg[0][4],jointdeg[0][5]);
            return false;
        }
        robotService.robotServiceInitGlobalMoveProfile();
        if((paramlist[2]<0.0 || paramlist[2]>3.0 ) || (paramlist[4]<0.0 || paramlist[4]>180.0 ))
        {
            ROS_ERROR("Aubo movet endeffector line and angle acc can not more than 3.0 and 180,we will use default lineacc=1.0,angleacc=1.0");
            this->aubo_end_set_acc(1.0,1.0);
        }else{
            this->aubo_end_set_acc(paramlist[2],paramlist[4]);
        }
        if((paramlist[3]<0.0 || paramlist[3]>3.0 ) || (paramlist[5]<0.0 || paramlist[5]>180.0 ))
        {
            ROS_ERROR("Aubo movet endeffector line and angle vel can not more than 3.0 and 180,we will use default linevel=1.0,anglevel=1.0");
            this->aubo_end_set_vel(1.0,1.0);
        }else{
            this->aubo_end_set_vel(paramlist[3],paramlist[5]);
        }

        int jointpath_num=joint_path_count;
        for (int i = 0; i < jointpath_num; i++)
        {
            ROS_INFO("====Aubo will add to Path Point [%d] (%f,%f,%f,%f,%f,%f)=======",i,jointdeg[i][0],jointdeg[i][1],jointdeg[i][2],jointdeg[i][3],jointdeg[i][4],jointdeg[i][5]);
            Util::initJointAngleArray(jointAngle, jointdeg[i][0]/180.0*M_PI,  jointdeg[i][1]/180.0*M_PI,  jointdeg[i][2]/180.0*M_PI, jointdeg[i][3]/180.0*M_PI,jointdeg[i][4]/180.0*M_PI, jointdeg[i][5]/180.0*M_PI);
            robotService.robotServiceAddGlobalWayPoint(jointAngle);
        }
        if(typeslfag==1)
        {   
            ROS_INFO("===Aubo will run with circular path=========");
            robotService.robotServiceSetGlobalCircularLoopTimes(paramlist[7]);
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);

            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed.=======Please check====");
            }
        }else if(typeslfag==2){
            ROS_INFO("===Aubo will run with CARTESIAN_MOVEP path=========");
            robotService.robotServiceSetGlobalBlendRadius(paramlist[6]);
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_MOVEP, true);

            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed=======Please check====");
            }
        }else if(typeslfag==3)
        {
            ROS_INFO("===Aubo will run with CARTESIAN_CUBICSPLINE path=========");
            robotService.robotServiceSetGlobalBlendRadius(paramlist[6]);
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_CUBICSPLINE, true);

            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed=======Please check====");
            }

        }else if(typeslfag==4)
        {
            ROS_INFO("===Aubo will run with CARTESIAN_UBSPLINEINTP path=========");
            robotService.robotServiceSetGlobalBlendRadius(paramlist[6]);
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_UBSPLINEINTP, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed=======Please check====");
            }

        }else if(typeslfag==5)
        {
            ROS_INFO("===Aubo will run with JIONT_CUBICSPLINE path=========");
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::JIONT_CUBICSPLINE, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed=======Please check====");
            }

        }else if(typeslfag==6)
        {
            ROS_INFO("===Aubo will run with JOINT_UBSPLINEINTP path=========");
            ret = robotService.robotServiceTrackMove(aubo_robot_namespace::JOINT_UBSPLINEINTP, true);
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_ERROR("=====TrackMove failed=======Please check====");
            }
        }
        else
        {
            ROS_ERROR("=====Movet type is error,Please check!!===");
        }
    }
    
}