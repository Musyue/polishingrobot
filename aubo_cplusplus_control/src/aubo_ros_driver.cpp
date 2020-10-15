#include "aubo_cplusplus_control/aubo_ros_driver.h"

using aubo10_ros_driver::AuboRosDriver;
namespace qi = boost::spirit::qi;
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
bool AuboRosDriver::aubo_movej(double jointdeg[],float acc,float vel,bool gozero)
// bool AuboRosDriver::aubo_movej(double jointdeg[],float jointacc,float jointvel,float endacc,float endvel,float ,bool gozero)
{
    
    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    if ( acc < 0.0 || acc > 180.0)
    {
        ROS_ERROR("Aubo movej joint control acc can not more than 180.0 in degree,we will use default acc=20.0");
        jointMaxAcc.jointPara[0] = 20.0/180.0*M_PI;
        jointMaxAcc.jointPara[1] = 20.0/180.0*M_PI;
        jointMaxAcc.jointPara[2] = 20.0/180.0*M_PI;
        jointMaxAcc.jointPara[3] = 20.0/180.0*M_PI;
        jointMaxAcc.jointPara[4] = 20.0/180.0*M_PI;
        jointMaxAcc.jointPara[5] = 20.0/180.0*M_PI;   //接口要求单位是弧度
        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    }else
    {
        jointMaxAcc.jointPara[0] = acc/180.0*M_PI;
        jointMaxAcc.jointPara[1] = acc/180.0*M_PI;
        jointMaxAcc.jointPara[2] = acc/180.0*M_PI;
        jointMaxAcc.jointPara[3] = acc/180.0*M_PI;
        jointMaxAcc.jointPara[4] = acc/180.0*M_PI;
        jointMaxAcc.jointPara[5] = acc/180.0*M_PI;   //接口要求单位是弧度
        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    }
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    if (vel < 0.0 || vel > 180.0)
    {
        ROS_ERROR("Aubo movej joint control vel can not more than 180.0 in degree,we will use default acc=20.0");
        /** 接口调用: 设置关节型运动的最大速度 ***/
        
        jointMaxVelc.jointPara[0] = 20.0/180.0*M_PI;
        jointMaxVelc.jointPara[1] = 20.0/180.0*M_PI;
        jointMaxVelc.jointPara[2] = 20.0/180.0*M_PI;
        jointMaxVelc.jointPara[3] = 20.0/180.0*M_PI;
        jointMaxVelc.jointPara[4] = 20.0/180.0*M_PI;
        jointMaxVelc.jointPara[5] = 20.0/180.0*M_PI;   //接口要求单位是弧度
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    }else
    {
        /** 接口调用: 设置关节型运动的最大速度 ***/
        jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
        jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
        jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
        jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
        jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
        jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    }
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    if(gozero)
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

    sleep(100);
}

void AuboRosDriver::MoveJ_Callback(const std_msgs::String::ConstPtr& msg)
 {
    ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movej_str="movej";
    std::string::size_type idx;
    idx=recv_str.find(movej_str);
    double jointdeg[6];
    float acc,vel;
    int gozero;
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movej,Command Like this movej{joint=(0,0,0,0,0,0),acc=0.0,vel=0.0,gozero=0}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        using boost::spirit::ascii::space;
        typedef std::string::const_iterator iterator_type;
        typedef aubo10_ros_driver::aubo_movej_parser<iterator_type> aubo_movej_parser;
        aubo_movej_parser g; // Our grammar
        std::string str;
        str=msg->data.c_str();
        aubo10_ros_driver::aubo_movej aubomovej;
        std::string::const_iterator iter = str.begin();
        std::string::const_iterator end = str.end();
        bool r = phrase_parse(iter, end, g, space, aubomovej);

        if (r && iter == end)
        {
            std::cout << boost::fusion::tuple_open('[');
            std::cout << boost::fusion::tuple_close(']');
            std::cout << boost::fusion::tuple_delimiter(", ");

            std::cout << "-------------------------\n";
            std::cout << "parser got data: " << boost::fusion::as_vector(aubomovej) << std::endl;

            // std::cout<<aubomovej.joint0<<std::endl;
            jointdeg[0]=aubomovej.joint0;
            jointdeg[1]=aubomovej.joint1;
            jointdeg[2]=aubomovej.joint2;
            jointdeg[3]=aubomovej.joint3;
            jointdeg[4]=aubomovej.joint4;
            jointdeg[5]=aubomovej.joint5;

            acc=aubomovej.acc;
            vel=aubomovej.vel;
            gozero=aubomovej.gozero;
            this->aubo_movej(jointdeg,acc,vel,gozero);
            // std::cout << typeid(aubomovej.joint0).name() << std::endl;
            // std::cout << typeid(aubomovej.gozero).name() << std::endl;

            std::cout << "\n-------------------------\n";
        }
        else
        {
            std::cout << "-------------------------\n";
            std::cout << "Parsing failed\n";
            std::cout << "-------------------------\n";
        }
    }
    

 }
void AuboRosDriver::MoveL_Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("GOT A Control CMD: [%s]", msg->data.c_str());
    std::string recv_str=msg->data.c_str();
    std::string movej_str="movel";
    std::string::size_type idx;
    idx=recv_str.find(movej_str);
    double jointdeg[6];
    float acc,vel;
    int gozero;
    if(idx==std::string::npos)
    {
        ROS_ERROR("Please publish the Right movel,Command Like this movel{joint=(0,0,0,0,0,0),acc=0.0,vel=0.0,gozero=0}");
    }
    else
    {
        ROS_INFO("We will move to [%s]",msg->data.c_str());
        using boost::spirit::ascii::space;
        typedef std::string::const_iterator iterator_type;
        typedef aubo10_ros_driver::aubo_movej_parser<iterator_type> aubo_movej_parser;
        aubo_movej_parser g; // Our grammar
        std::string str;
        str=msg->data.c_str();
        aubo10_ros_driver::aubo_movej aubomovej;
        std::string::const_iterator iter = str.begin();
        std::string::const_iterator end = str.end();
        bool r = phrase_parse(iter, end, g, space, aubomovej);

        if (r && iter == end)
        {
            std::cout << boost::fusion::tuple_open('[');
            std::cout << boost::fusion::tuple_close(']');
            std::cout << boost::fusion::tuple_delimiter(", ");

            std::cout << "-------------------------\n";
            std::cout << "parser got data: " << boost::fusion::as_vector(aubomovej) << std::endl;

            // std::cout<<aubomovej.joint0<<std::endl;
            jointdeg[0]=aubomovej.joint0;
            jointdeg[1]=aubomovej.joint1;
            jointdeg[2]=aubomovej.joint2;
            jointdeg[3]=aubomovej.joint3;
            jointdeg[4]=aubomovej.joint4;
            jointdeg[5]=aubomovej.joint5;

            acc=aubomovej.acc;
            vel=aubomovej.vel;
            gozero=aubomovej.gozero;
            this->aubo_movej(jointdeg,acc,vel,gozero);
            // std::cout << typeid(aubomovej.joint0).name() << std::endl;
            // std::cout << typeid(aubomovej.gozero).name() << std::endl;

            std::cout << "\n-------------------------\n";
        }
        else
        {
            std::cout << "-------------------------\n";
            std::cout << "Parsing failed\n";
            std::cout << "-------------------------\n";
        }
    }
    

}