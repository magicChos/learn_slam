#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


int main(int argc, char **argv)
{
    Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
    std::cout << "Tlc_initial: " << Tlc_initial << std::endl;
    Eigen::Matrix4d Tlc = Tlc_initial.inverse();
    std::cout << "Tlc: " << Tlc << std::endl;


    Eigen::Vector3f v;
    v << 1.0 , 2.0 , 3.0;

    std::cout << v << std::endl;

    std::cout << "3 v: " << 3 * v << std::endl;
    return 1;
}