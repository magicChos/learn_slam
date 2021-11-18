/************************************************************************************************
@filename    :QR.cpp
@brief       :QR分解解算AX=B方程
@time        :2021/11/18 11:33:10
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <Eigen/Dense>
#include <iostream>

int main(int argc, char *argv[])
{
    Eigen::Matrix2d A;
    A << 1, 2, 4, 5;
    Eigen::Vector2d b(3, 4);
    Eigen::Vector2d C = A.colPivHouseholderQr().solve(b);

    std::cout << C << std::endl;
    return 0;
}
