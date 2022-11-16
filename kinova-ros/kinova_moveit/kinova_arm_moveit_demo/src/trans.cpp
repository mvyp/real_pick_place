#include <tf_conversions/tf_eigen.h>
#include <iostream>
int main()
{
    Eigen::Quaterniond base_color(0.506, -0.495, 0.501, -0.499);//四元数

    Eigen::Isometry3d T_1=Eigen::Isometry3d::Identity(); //变换矩阵
    T_1.rotate ( base_color.toRotationMatrix() );  
    T_1.pretranslate ( Eigen::Vector3d ( 0,0.015,0 ) ); 
    Eigen::Quaterniond end_color       
                    ( 0.9055217233172039
  , -0.12933246089993763
  , 0.30784678177960656
  , 0.26178976700044443); //四元数
    Eigen::Isometry3d T_2=Eigen::Isometry3d::Identity(); //变换矩阵
    T_2.rotate (end_color.toRotationMatrix() );  
    T_2.pretranslate ( Eigen::Vector3d ( -0.0006463342107878509
  ,0.01924570383925011
  ,0.09918770345072835) );
    
    Eigen::Isometry3d T = T_2 * T_1.inverse();
    Eigen::Matrix3d rotation_matrix;//旋转矩阵
    rotation_matrix = T.rotation();
    Eigen::Vector3d e_b_p = T.translation();//t_odom_curr_now 平移向量
    Eigen::Quaterniond e_b_q = Eigen::Quaterniond ( rotation_matrix );//转为四元数

    std::cout << "qw: " << e_b_q.w() << std::endl; 
    std::cout << "qx: " << e_b_q.x() << std::endl; 
    std::cout << "qy: " << e_b_q.y() << std::endl; 
    std::cout << "qz: " << e_b_q.z() << std::endl; 

    std::cout << "x: " << e_b_p[0] << std::endl;
    std::cout << "y: " << e_b_p[1] << std::endl;
    std::cout << "z: " << e_b_p[2] << std::endl;


}

