#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;

/**
 * @brief 构建切向空间
 *
 * @param g0
 * @return MatrixXd
 */
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    std::cout << "a = " << a << std::endl;
    Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

int main(int argc, char **argv)
{
    Eigen::Vector3d m;
    m << 0, 1, 0;

    Eigen::Vector3d n;
    n << 0, 0, 1;

    std::cout << "向量积： " << m.dot(n) << std::endl;
    std::cout << "叉积: " << m.cross(n).norm() << std::endl;

    Eigen::Vector3d mn;
    mn << 1, 2, 3;
    mn.normalize();
    std::cout << mn << std::endl;

    Eigen::Vector3d g0(0, 0, 9.81);
    MatrixXd res = TangentBasis(g0);
    std::cout << "res = " << res << std::endl;

    Eigen::Isometry3f tran_isometry = Eigen::Isometry3f::Identity();
    std::cout << "trans_isometry = " << tran_isometry.matrix() << std::endl;

    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    std::cout << init_guess << std::endl;

    Eigen::Matrix<double, 3, 4> P0;
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    std::cout << "P0 = " << P0 << std::endl;

    return 1;
}
