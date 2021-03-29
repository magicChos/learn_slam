#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main(int argc, char **argv)
{
    Quaterniond q1(0.0, 0.0, 0.0, 1.0), q2(0.0, 0.0, 0.0, 1.0);
    q1.normalize();
    q2.normalize();

    Vector3d t1(10, 10, 10), t2(10, 0, 10);
    Vector3d p1(10, 0, 0);

    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);
    std::cout << T1w.rotation() << std::endl;

    std::cout << T1w.matrix() << std::endl;
    std::cout << T2w.matrix() << std::endl;

    Eigen::Vector3d pt;
    pt << 1, 2, 3;
    std::cout << "pt: " << pt << std::endl;

    Eigen::Matrix4d tof_2_base_matrix_;

    tof_2_base_matrix_ << 1.0501503000000001e-02, 2.0494568000000001e-03,
        9.9994278000000003e-01, 2.0858668689999998e-01,
        -9.9994123000000001e-01, 2.7064331999999999e-03,
        1.0495938999999999e-02, -1.6197355000000000e-02,
        -2.6847673999999999e-03, -9.9999422000000004e-01,
        2.0777580999999999e-03, 5.2435359199999997e-02, 0., 0., 0., 1.;
    std::cout << tof_2_base_matrix_ << std::endl;

    std::cout << "-------------------------------\n";
    std::cout << tof_2_base_matrix_(0,0) << std::endl;

    Eigen::Matrix4d base_map_matrix = Eigen::Matrix4d::Identity();
    std::cout << base_map_matrix << std::endl;


    Eigen::Vector3d d3 = Eigen::Vector3f(1 , 2 , 3);
    Eigen::Vector3f f3 = d3;
    std::cout << f3 << std::endl;

    return 1;
}
