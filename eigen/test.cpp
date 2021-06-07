#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


int main(int argc, char **argv)
{
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(1 , 1) = 2;

    Eigen::Vector3d n;
    n << 1.0 , 2.0 , 3.0;
    std::cout << n << std::endl;
    std::cout << "----------------------------" << std::endl;
    n = n / n(2);
    std::cout << n << std::endl;
    

    return 1;
}