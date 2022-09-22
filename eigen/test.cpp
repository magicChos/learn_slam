#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <ostream>
#include <math.h>
#include <algorithm>
#include <functional>

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

using TrajectoryType = std::vector<Eigen::Isometry3d>;

TrajectoryType ReadTrajectory(const std::string &file_name)
{
    std::ifstream  fin(file_name);
    TrajectoryType trajectory;
    if (!fin)
    {
        std::cerr << "trajectory " << file_name << "not found" << std::endl;
        return trajectory;
    }
    while (!fin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d p1;
        p1.translation().x() = tx;
        p1.translation().y() = ty;
        p1.translation().z() = tz;
        p1.linear()          = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        trajectory.push_back(p1);
    }
    return trajectory;
}

void SVD(Eigen::Matrix3d &matrix_A)
{
    // Matrix3d A;
    // A(0, 0) = 1, A(0, 1) = 0, A(0, 2) = 1;
    // A(1, 0) = 0, A(1, 1) = 1, A(1, 2) = 1;
    // A(2, 0) = 0, A(2, 1) = 0, A(2, 2) = 0;

    JacobiSVD<Eigen::MatrixXd> svd(matrix_A, ComputeThinU | ComputeThinV);
    Matrix3d                   V = svd.matrixV(), U = svd.matrixU();

    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (U.determinant() * V.transpose().determinant() < 0)
    {
        S(2, 2) = -1;
    }

    matrix_A = U * S * V.transpose();
    // std::cout << "rot = " << matrix_A << std::endl;
}

void calculateRPE(const TrajectoryType &gt, const TrajectoryType &esti)
{
    
    // double delta = 1; 
    // for (size_t i = 0; i < gt.size() - delta; i++)
    // {
    //     double error_trans =
    //         ((gt[i].inverse() * gt[i + delta]).inverse() * (esti[i].inverse() * esti[i + delta])).translation().norm();
    //     rmse_trans += error_trans * error_trans;
    // }

    // // rmse_all   = sqrt(rmse_all / double(gt.size()));
    // rmse_trans = sqrt(rmse_trans / double(gt.size()));
    // std::vector<double> RPE;
    // // RPE.push_back(rmse_all);
    // RPE.push_back(rmse_trans);
    // std::cout << " RPE_trans = " << rmse_trans << std::endl;

    // return RPE;
}

std::vector<double> calculateATE(const TrajectoryType &gt, const TrajectoryType &esti)
{
    // step1 计算轨迹中心点
    // step2 每个点减去中心点
    // step3 计算向量外积

    Eigen::MatrixXd gt_translation(gt.size(), 3);
    Eigen::MatrixXd et_translation(esti.size(), 3);
    for (size_t i = 0; i < gt.size(); ++i)
    {
        gt_translation.row(i) = gt[i].translation();
        et_translation.row(i) = esti[i].translation();
    }

    Eigen::Vector3d gt_centered = gt_translation.colwise().mean();
    Eigen::Vector3d et_centered = et_translation.colwise().mean();

    std::cout << "gt_centered = " << gt_centered << std::endl;
    std::cout << "et_centered = " << et_centered << std::endl;

    Eigen::MatrixXd gt_translation_centered(gt.size(), 3);
    Eigen::MatrixXd et_translation_centered(esti.size(), 3);

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < gt.size(); ++i)
    {
        gt_translation_centered.row(i) = gt[i].translation() - gt_centered;
        et_translation_centered.row(i) = esti[i].translation() - et_centered;

        W += gt_translation_centered.row(i).matrix().transpose() * et_translation_centered.row(i);
    }

    // Eigen::Matrix3d WT = W.transpose();
    SVD(W);

    std::cout << "W = " << W << std::endl;

    Eigen::Vector3d trans = gt_centered - W * et_centered;
    std::cout << "trans = " << trans << std::endl;

    Eigen::MatrixXd et_aligned = (W * et_translation.transpose()).transpose();

    for (int i = 0; i < esti.size(); i++)
    {
        et_aligned.row(i) = et_aligned.row(i) + trans.transpose();
    }

    Eigen::MatrixXd alignment_error = et_aligned - gt_translation;
    std::cout << "@test 0 row = " << alignment_error.row(0) << std::endl;

    std::cout << "@test ??? = " << alignment_error.row(0).norm() << std::endl;

    std::vector<double> trans_error;
    for (int i = 0; i < gt.size(); ++i)
    {
        trans_error.push_back(alignment_error.row(i).norm());
    }

    // double rmse_all, rmse_trans;
    // for (size_t i = 0; i < gt.size(); i++)
    // {
    //     // // ATE旋转+平移
    //     // double error_all = (gt[i].inverse() * esti[i]).log().norm();
    //     // rmse_all += error_all * error_all;
    //     // ATE平移
    //     double error_trans = (gt[i].inverse() * esti[i]).translation().norm();
    //     rmse_trans += error_trans * error_trans;
    // }
    // // rmse_all   = sqrt(rmse_all / double(gt.size()));
    // rmse_trans = sqrt(rmse_trans / double(gt.size()));
    // std::vector<double> ATE;
    // // ATE.push_back(rmse_all);
    // ATE.push_back(rmse_trans);
    // // std::cout << " ATE_trans = " << rmse_trans << std::endl;

    // return ATE;
}

void test_cross()
{
    Eigen::Vector3d a(1, 2, 3);
    Eigen::Vector3d b(3, 2, 1);

    Eigen::MatrixXd rot(2, 3);
    rot.row(0) = a;
    rot.row(1) = b;

    // std::cout << rot.row(0) <<std::endl;

    Eigen::Matrix3d W = rot.row(0).matrix().transpose() * rot.row(1).matrix();

    Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
    W                 = W + V;
    std::cout << W << std::endl;

    std::cout << W.rowwise().mean() << std::endl;
    std::cout << W.colwise().mean() << std::endl;

    // Eigen::MatrixXd fuck_rot(2 , 3);
    // fuck_rot.row(0) = a;
    // fuck_rot.row(1) = a;

    // std::cout << "fuck_rot = " << fuck_rot << std::endl;
    // rot = rot - fuck_rot;

    // rot.row(0) = rot.row(0) - a.transpose();
    // rot = rot - a.transpose();
    std::cout << rot << std::endl;

    // auto W = rot.row(0).cross(rot.row(1));
    // std::cout << W << std::endl;
}

std::vector<float> matrix2angle(Eigen::Matrix3d rotateMatrix)
{
	float sy = (float)sqrt(rotateMatrix(0,0) * rotateMatrix(0,0) + rotateMatrix(1,0)*rotateMatrix(1,0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = (float)atan2(rotateMatrix(2,1), rotateMatrix(2,2));
		y = (float)atan2(-rotateMatrix(2,0), sy);
		z = (float)atan2(rotateMatrix(1, 0), rotateMatrix(0, 0));
	}
	else
	{
		x = (float)atan2(-rotateMatrix(1, 2), rotateMatrix(1, 1));
		y = (float)atan2(-rotateMatrix(2, 0), sy);
		z = 0;
	}
	std::vector<float> i;
	i.push_back((float)(x * (180.0f / M_PI)));
	i.push_back((float)(y * (180.0f / M_PI)));
	i.push_back((float)(z * (180.0f / M_PI)));
	return i;
}

double matrix2RotAngle(const Eigen::Matrix3d &m)
{
    double sum = m(0 , 0) + m(1 , 1) + m(2 , 2);
    return std::acos(std::min(1.0 , std::max(-1.0 , (sum - 1.0)/2)));
}

int main(int argc, char **argv)
{
    // Eigen::Vector3d m;
    // m << 0, 1, 0;

    // Eigen::Vector3d n;
    // n << 0, 0, 1;

    // std::cout << "向量积： " << m.dot(n) << std::endl;

    // double cos_theta = m.dot(n) / (m.norm() * n.norm());
    // std::cout << "夹角 = " << std::acos(cos_theta) << std::endl;

    // Eigen::Vector3d cross_pt = m.cross(n);
    // std::cout << "叉积: " << m.cross(n) << std::endl;

    // Eigen::Vector3d mn;
    // mn << 1, 2, 3;
    // mn.normalize();
    // std::cout << mn << std::endl;

    // Eigen::Vector3d g0(0, 0, 9.81);
    // MatrixXd        res = TangentBasis(g0);
    // std::cout << "res = " << res << std::endl;

    // Eigen::Isometry3f tran_isometry = Eigen::Isometry3f::Identity();
    // std::cout << "trans_isometry = " << tran_isometry.matrix() << std::endl;

    // Eigen::AngleAxisf    init_rotation(0, Eigen::Vector3f::UnitZ());
    // Eigen::Translation3f init_translation(0, 0, 0);
    // Eigen::Matrix4f      init_guess = (init_translation * init_rotation).matrix();

    // std::cout << init_guess << std::endl;

    // Eigen::Matrix<double, 3, 4> P0;
    // P0.leftCols<3>()  = Eigen::Matrix3d::Identity();
    // P0.rightCols<1>() = Eigen::Vector3d::Zero();

    // std::cout << "P0 = " << P0 << std::endl;

    // Eigen::Matrix<double, 6 , 3> P1;
    // P1.row(0) = Eigen::Matrix3d::Identity();

    // Eigen::MatrixXd rot(6, 3);
    // for (int i = 0; i < 6; ++i)
    // {
    //     for (int j = 0; j < 3; ++j)
    //     {
    //         rot(i, j) = 2;
    //     }
    // }

    // Eigen::MatrixXd rot2(6, 3);
    // for (int i = 0; i < 6; ++i)
    // {
    //     // for (int j = 0; j < 3; ++j)
    //     // {
    //     //     rot2(i, j) = 3;
    //     // }

    //     rot2.row(i) << 1, 2, 3;
    // }

    // std::cout << rot2 << std::endl;

    // std::cout << rot2.norm() << std::endl;

    // // std::cout << rot << std::endl;

    // // auto d = rot2 - rot;
    // // std::cout << d << std::endl;

    // Eigen::Isometry3d T1;
    // T1.setIdentity();
    // T1.linear() = Eigen::Matrix3d::Identity();
    // T1.translation() << 1, 2, 3;
    // std::cout << T1.matrix() << std::endl;
    // auto q_matrix = T1.linear();
    // std::cout << "q = " << q_matrix << std::endl;

    // Eigen::Quaterniond q(q_matrix.matrix());
    // std::cout << "q = " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

    // std::string groundtruth_file = "/home/han/project/learn_slam/eigen/gt_pose_align.txt";
    // std::string estimated_file   = "/home/han/project/learn_slam/eigen/et_pose_align.txt";
    // //读取高博slam14讲中的groundtruth.txt和estimated.txt的数据
    // auto gi   = ReadTrajectory(groundtruth_file);
    // auto esti = ReadTrajectory(estimated_file);

    // std::vector<double> ATE, RPE;
    // // calculateATE(gi, esti); //计算ATE

    // calculateRPE(gi, esti);

    // test_cross();

    Eigen::Matrix3d d;
    d << 0.999993 , -0.000994105 , -0.00369948 ,0.00104245  ,   0.999914  ,  0.0130885 ,0.00368615  , -0.0130923  ,  0.999907;

    // double sum = d(0 , 0) + d(1 , 1) + d(2 , 2);
    // // double angle = std::acos(std::min(1, std::max(-1 , (sum - 1)/2)));
    // double val = (sum - 1)/2;

    // double angle = std::acos(std::min(1.0 , std::max(-1.0 , val)));
    std::cout << "angle: " << matrix2RotAngle(d) << std::endl;

    return 1;
}



