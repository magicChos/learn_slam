#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


int main(int argc, char **argv)
{
    Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
    std::cout << "Tlc_initial: " << Tlc_initial << std::endl;
    Eigen::Matrix4d Tlc = Tlc_initial.inverse();
    std::cout << "Tlc: " << Tlc << std::endl;
    return 1;
}