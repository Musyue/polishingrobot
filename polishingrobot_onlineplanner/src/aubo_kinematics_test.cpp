#include "ros/ros.h"
#include "../include/aubo_kinematics.h"
#include <iostream>


#include <dynamic_reconfigure/server.h>
#include <polishing_pkg/Polishing_Config.h>
int count=0;
void callback(polishing_pkg::Polishing_Config &config, uint32_t level) {
  count=config.int_param;
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "aubo_kinematics_test");

    dynamic_reconfigure::Server<polishing_pkg::Polishing_Config> server;
    dynamic_reconfigure::Server<polishing_pkg::Polishing_Config>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    double y_temp_max=0.0;
    double x_temp_max=0.0;
    double z_temp_max=0.0;
    double r_max_from_aubo=1.5;
    double D_distance=0.31;

    double r_cut = sqrt(r_max_from_aubo*r_max_from_aubo+D_distance*D_distance);
    VectorXd circle_center(3);
    circle_center<<0,D_distance,0;
    double o1[3];
    double o2[3];
    double o3[3];
    double o4[3];
    double o5[3];
    double o6[3];
    o1[0]=-1*sqrt(2)*r_cut/3;
    o1[1]=D_distance;
    o1[2]=sqrt(2)*r_cut/4;

    o2[0]=0.0;
    o2[1]=D_distance;
    o2[2]=sqrt(2)*r_cut/4;

    o3[0]=sqrt(2)*r_cut/3;
    o3[1]=D_distance;
    o3[2]=sqrt(2)*r_cut/4;

    o4[0]=-1*sqrt(2)*r_cut/3;
    o4[1]=D_distance;
    o4[2]=-sqrt(2)*r_cut/4;

    o5[0]=0.0;
    o5[1]=D_distance;
    o5[2]=-sqrt(2)*r_cut/4;

    o6[0]=sqrt(2)*r_cut/3;
    o6[1]=D_distance;
    o6[2]=sqrt(2)*r_cut/4;
    // // printf("haha");
    ros::Rate loop_rate(1);
    VectorXd q(6);
    q<<6.33,18.66,142.092,120.32,86.375,0.101;
    VectorXd q_result(6);
    deg_to_rad(q_result,q);
    std::cout<<q_result<<std::endl;
    MatrixXd PosIn_Matrix(4,4);
    aubo_forward(PosIn_Matrix,q_result);
    
    // std::cout<<PosIn_Matrix<<std::endl;
    // MatrixXd T_target(4,4);
    // T_target(0,0) =  -0.991144;
    // T_target(0,1) = -0.0291793;
    // T_target(0,2) = -0.129545;
    // T_target(0,3) = o1[0];
    // T_target(1,0) = -0.131647;
    // T_target(1,1) = 0.0881837;
    // T_target(1,2) =  0.987367;
    // T_target(1,3) = o1[1];
    // T_target(2,0) = -0.0173869;
    // T_target(2,1) = 0.995677;
    // T_target(2,2) = -0.0912442;
    // T_target(2,3) = o1[2];
    // T_target.row(3) << 0, 0, 0, 1;
    // VectorXd q_last(6);
    // GetInverseResult(T_target,q_result,q_last);
    // std::cout<<q_last<<std::endl;
    // -0.991144 -0.0291793  -0.129545   0.176876
    // -0.131647  0.0881837   0.987367   0.302485
    // -0.0173869   0.995677 -0.0912442   0.796422
    // Eigen::MatrixXd Matrix_bTc(4,4);
    // Matrix_bTc<<0.06787 ,-0.03428, 0.99710 ,0.24361234,
    //     -0.99755 ,0.01493, 0.06842, -0.25666829,
    //     -0.01723 ,-0.99930, -0.03318, 0.75667707
    //     ,0,0,0,1; 
    // Eigen::MatrixXd Matrix_eTc(4,4);
    // Matrix_eTc<<-0.99960561, -0.01773356,  0.0215622 , -0.09214288,
    //      0.01817358, -0.99962585,  0.0204784 ,  0.21627011,
    //      0.0211837 ,  0.02085993,  0.99956265, -0.01512569,
    //      0.        ,  0.        ,  0.        ,  1.  ;
    // Eigen::MatrixXd Matrix_pcl_one_point(4,1);
    // Matrix_pcl_one_point<<-0.36174914,-0.20874575, 0.97912359,1;
    // Eigen::MatrixXd Matrix_pcl_one_point_after(4,1);
    // Matrix_pcl_one_point_after=Matrix_bTc*Matrix_pcl_one_point;
    // std::cout<<Matrix_bTc*Matrix_pcl_one_point<<std::endl;
    // std::cout<<Matrix_pcl_one_point_after(0,0)<<" "<<Matrix_pcl_one_point_after(1,0);
    // for (size_t i = 0; i < q_result.size(); i++)
    // {
    //     /* code */
    //     std::cout<<q_result(i)<<" ";
    // }
    // std::cout<<std::endl;
    while (ros::ok()){

      std::cout<<count<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();	
      }
      
    
    
    return 0;
}