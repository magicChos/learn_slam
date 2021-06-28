#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


int main(int argc, char **argv)
{
    Eigen::Vector3d m;
    m << 0 , 1 , 0;

    Eigen::Vector3d n;
    n << 0 , 0 , 1;

    std::cout << "向量积： " << m.dot(n) << std::endl;
    std::cout << "叉积: " << m.cross(n).norm() << std::endl;

    Eigen::Vector3d mn;
    mn << 1 , 2 , 3;
    mn.normalize();
    std::cout << mn << std::endl;
    

    return 1;
}