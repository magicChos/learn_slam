#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main(int argc , char **argv)
{
    Quaterniond q1(0.0 , 0.0 , 0.0 , 1.0) , q2(0.0 , 0.0 , 0.0 , 1.0);
    q1.normalize();
    q2.normalize();

    Vector3d t1(10 , 10 , 10) , t2(10 , 0 , 10);
    Vector3d p1(10 , 0 , 0);

    Isometry3d T1w(q1) , T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);
    std::cout << T1w.rotation() << std::endl;

    std::cout << T1w.matrix() << std::endl;
    std::cout << T2w.matrix() << std::endl;


    return 1;
}

