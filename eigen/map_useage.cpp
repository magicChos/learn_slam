// /*
//  * @Author: your name
//  * @Date: 2020-03-15 20:42:13
//  * @LastEditTime: 2020-04-09 19:40:58
//  * @LastEditors: Please set LastEditors
//  * @Description: In User Settings Edit
//  * @FilePath: /learn_slam/eigen/map_useage.cpp
//  */
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>
// #include <iostream>

// void test()
// {
//     std::cerr << __PRETTY_FUNCTION__ << std::endl;
//     std::cerr << __FUNCTION__ << std::endl;
// }

// int main()
// {
//     int array[8];
//     for (int i = 0; i < 8; ++i)
//     {
//         array[i] = i;
//     }

//     std::cout << "column-major: " << Eigen::Map<Eigen::Matrix<int, 2, 4>>(array) << std::endl;
//     std::cout << "row-major: " << Eigen::Map<Eigen::Matrix<int, 2, 4>>(array) << std::endl;
//     std::cout << "Row-major using stride:\n"
//               << Eigen::Map<Eigen::Matrix<int, 2, 4>, Eigen::Unaligned, Eigen::Stride<1, 4>>(array) << std::endl;

//     double *update = new double[3];
//     for (int i = 0; i < 3; ++i)
//     {
//         update[i] = i;
//     }

//     Eigen::Vector3d d = Eigen::Vector3d(update);
//     std::cout << "d: \n"
//               << std::endl;
//     std::cout << d << std::endl;

//     double data[9];
//     for (int i = 0; i < 9; ++i)
//     {
//         data[i] = i;
//     }

//     Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matrix33 = Eigen::Map<Eigen::Matrix<double, 3, 3>>(data);
//     std::cout << "matrix33 = \n";
//     std::cout << matrix33 << std::endl;

//     std::cout << Eigen::Matrix<double, 1, 1>::Identity() * 1 / (1.0 * 1.0) << std::endl;

//     test();

//     Eigen::Matrix3d rot_mat;

//     rot_mat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ());
//     std::cout << "rot_mat: \n"
//               << rot_mat << std::endl;

//     Eigen::Vector3d v(1, 2, 3);
//     Eigen::Vector3d w(0, 1, 2);

//     std::cout << v.cross(w);

//     Eigen::Vector3d v_test;
//     std::cout << "-----: " << v_test(0, 0) << std::endl;
//     std::cout << "-----: " << v_test(1, 0) << std::endl;
//     std::cout << "-----: " << v_test(2, 0) << std::endl;

//     return 0;
// }

#include <fstream>
#include <iostream>
using namespace std;
int main()
{
    double *gpx = new double[3];
    double *gpy = new double[3];
    double *gpz = new double[3]; //存储控制点物方坐标
    double *ipx = new double[3];
    double *ipy = new double[3]; //存储控制点像方坐标
    ifstream filein("1.txt", ios::in);
    // 以输入方式打开一个文件，ios::nocreate表示原文件不存在时，不创建
    if (!filein)
    {
        cout << "文件不存在";
        exit(1);
    }
    for (int i = 0; i < 3; i++)
        filein >> ipx[i] >> ipy[i] >> gpx[i] >> gpy[i] >> gpz[i];
    filein.close();
    for (int j = 0; j < 3; j++)
        cout << ipx[j] << "\t" << ipy[j] << "\t" << gpx[j] << "\t" << gpy[j] << "\t" << gpz[j] << endl;

    return 0;
}
