/************************************************************************************************
@filename    :transformAssocateToMap.cpp
@brief       :legoloam中transformAssociateToMap函数讨论
@link        :https://blog.csdn.net/l898985121/article/details/114844864?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-1.highlightwordscore&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-1.highlightwordscore
@time        :2021/12/25 22:48:33
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/



#include <Eigen/Dense>
#include <cmath>
#include <iostream>


float transformLast[6];
// 记录当前时刻的里程计位姿
float transformSum[6];

float transformIncre[6];

// 记录当前时刻经过优化后的位姿
float transformTobeMapped[6];

// 记录上一时刻的里程计位姿
float transformBefMapped[6];

// 记录上一时刻经过优化后的位姿
float transformAftMapped[6];

Eigen::Matrix4f rotx(float angle)
{
    Eigen::Matrix4f b1;
    b1 << 1, 0, 0, 0,
        0, cos(angle), -sin(angle), 0,
        0, sin(angle), cos(angle), 0,
        0, 0, 0, 1;
    return b1;
}
Eigen::Matrix4f roty(float angle)
{
    Eigen::Matrix4f b1;
    b1 << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;
    return b1;
}
Eigen::Matrix4f rotz(float angle)
{
    Eigen::Matrix4f b1;
    b1 << cos(angle), -sin(angle), 0, 0,
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return b1;
}
Eigen::Matrix4f translation();
void transformAssociateToMap()
{
    // t_12: 表示2到1的平移，R_10：表示0到1的旋转
    // t_12 = R_10(t_02 - t_01)
    // R_10 = R(-rz , -rx , -ry) = R(-rz) * R(-rx) * R(-ry)
    // 如下表示的是R(-ry)
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    // R(-rx)
    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    // R(-rz)
    // transformIncre表示的t_12
    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;
    //-----------------------------------------end-------------------------------------------

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
    float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                   crycrx / cos(transformTobeMapped[0]));

    float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                   crzcrx / cos(transformTobeMapped[0]));

    /****************************************************************/
    // t_04 = R_03*t_12 + t_03
    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] = transformAftMapped[3] - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5] - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
    /******************************end********************************/

}

int main(int argc, char **argv)
{
    transformSum[0] = 0.0;
    transformSum[1] = 0.0;
    transformSum[2] = M_PI / 6.0;
    transformSum[3] = 10.0;
    transformSum[4] = 20.0;
    transformSum[5] = 0.0;

    transformBefMapped[0] = 0.0;
    transformBefMapped[1] = 0.0;
    transformBefMapped[2] = M_PI / 4.0;
    transformBefMapped[3] = 8.0;
    transformBefMapped[4] = 17.0;
    transformBefMapped[5] = 0.0;

    transformAftMapped[0] = 0;
    transformAftMapped[1] = 0;
    transformAftMapped[2] = M_PI / 2.0;
    transformAftMapped[3] = 0;
    transformAftMapped[4] = 15.0;
    transformAftMapped[5] = 0;

    transformAssociateToMap();
    std::cout << transformTobeMapped[0] << " , "
              << transformTobeMapped[1] << " , "
              << transformTobeMapped[2] << " , "
              << transformTobeMapped[3] << " , "
              << transformTobeMapped[4] << " , "
              << transformTobeMapped[5] << std::endl;
    std::cout << "angle : " << M_PI / 2.0 - (M_PI / 4 - M_PI / 6) << std::endl;

    Eigen::Matrix4f a = roty(transformTobeMapped[0]) * rotx(transformTobeMapped[1]) * rotz(transformTobeMapped[2]);
    a(0, 3) = transformTobeMapped[3];
    a(1, 3) = transformTobeMapped[4];
    a(2, 3) = transformTobeMapped[5];
    std::cout << a << std::endl;
    std::cout << "------------------" << std::endl;

    // 代表当前里程计位姿
    Eigen::Matrix4f t1 = roty(transformSum[0]) * rotx(transformSum[1]) * rotz(transformSum[2]);
    t1(0, 3) = transformSum[3];
    t1(1, 3) = transformSum[4];
    t1(2, 3) = transformSum[5];
    // 上一时刻里程计位姿
    Eigen::Matrix4f t2 = roty(transformBefMapped[0]) * rotx(transformBefMapped[1]) * rotz(transformBefMapped[2]);
    t2(0, 3) = transformBefMapped[3];
    t2(1, 3) = transformBefMapped[4];
    t2(2, 3) = transformBefMapped[5];
    // 上一时刻里程计位姿经过优化之后的位姿
    Eigen::Matrix4f t3 = roty(transformAftMapped[0]) * rotx(transformAftMapped[1]) * rotz(transformAftMapped[2]);
    t3(0, 3) = transformAftMapped[3];
    t3(1, 3) = transformAftMapped[4];
    t3(2, 3) = transformAftMapped[5];

    Eigen::Matrix4f c = t3 * t2.inverse() * t1;
    std::cout << c << std::endl;

    return 0;
}
