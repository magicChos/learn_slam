#include <iostream>
#include <cmath>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

template <typename T>
inline T DotProduct(const T x[3], const T y[3])
{
    return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

int main(int argc, char *argv[])
{

    double angle_axis[3] = {0.0925186, 0.201963, 0.172691};
    double theta2 = DotProduct(angle_axis, angle_axis);

    double theta = std::sqrt(theta2);
    std::cout << "theta: " << theta << std::endl;

    double angle_axis_norm[3];
    for (int i = 0; i < 3; ++i)
    {
        angle_axis_norm[i] = angle_axis[i] / theta;
    }

    Eigen::AngleAxisd rotaion_vector(theta, Eigen::Vector3d(angle_axis_norm[0], angle_axis_norm[1], angle_axis_norm[2]));
    std::cout << rotaion_vector.matrix() << std::endl;

    VectorRef angle_axis_ref(angle_axis, 3);
    Eigen::VectorXd inverse_rotation = -angle_axis_ref;
    std::cout << "inverse_rotation: " << inverse_rotation << std::endl;

    double *angle_axis_inverse = inverse_rotation.data();
    double theta2_inverse = DotProduct(inverse_rotation.data() , inverse_rotation.data());
    double theta_inverse = std::sqrt(theta2_inverse);
    std::cout << "theta_inverse: " << theta2_inverse << std::endl;

    double angle_axis_norm_inverse[3];
    for(int i = 0 ; i < 3 ; ++i)
    {
        angle_axis_norm_inverse[i] = angle_axis_inverse[i]/theta_inverse;
    }
    Eigen::AngleAxisd rotaion_vector_inverse(theta_inverse , Eigen::Vector3d(angle_axis_norm_inverse));
    std::cout << rotaion_vector_inverse.matrix() << std::endl;
    
    // Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // std::cout << "rotation_matrix: \n" << rotation_matrix << std::endl;

    // // 旋转向量 , 沿z轴旋转45度
    // Eigen::AngleAxisd rotation_vector(M_PI/4 , Eigen::Vector3d(0 , 0 , 1));
    // std::cout.precision(3);

    // // 旋转向量转旋转矩阵
    // std::cout << "rotation matrix=\n" << rotation_vector.matrix()<<std::endl;

    // // 也可以直接赋值
    // rotation_matrix = rotation_vector.toRotationMatrix();
    // std::cout<<"赋值后的rotation matrix=\n" << rotation_matrix<<std::endl;

    // // 利用AngleAxis进行坐标变换
    // Eigen::Vector3d v(1 , 0 ,0);
    // Eigen::Vector3d v_rotated = rotation_vector * v;
    // std::cout << "(1 , 0 ,0) after rotation = \n" << v_rotated.transpose() << std::endl;

    // // 或者利用旋转矩阵
    // v_rotated = rotation_matrix * v;
    // std::cout << "(1 , 0 ,0) after rotation = \n" << v_rotated.transpose() << std::endl;

    // // 欧拉角：将旋转矩阵直接转换为欧拉角
    // Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2 , 1 , 0); // ZYX顺序
    // std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;

    // // 旋转向量转欧拉角（Z-Y-X）
    // Eigen::Vector3d eulerAngle = rotation_vector.matrix().eulerAngles(2 , 1, 0);
    // std::cout << "yaw pitch roll = " << eulerAngle.transpose() << std::endl;

    // // 欧式变换矩阵使用Eigen::Isometry
    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // // 按照rotation_vector进行旋转
    // T.rotate(rotation_vector);
    // std::cout << "欧式旋转之后的矩阵 = \n" << T.matrix() << std::endl;
    // // 把平移向量设为(1 , 3 , 4)
    // T.pretranslate(Eigen::Vector3d(1 , 3 , 4));
    // std::cout << " Transform matrix = \n" << T.matrix() << std::endl;

    // // 用变换矩阵进行坐标变换
    // Eigen::Vector3d v_transformed = T * v;
    // std::cout << "v transform = \n" << v_transformed.transpose() << std::endl;

    // // 旋转向量转四元数
    // Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    // std::cout << "quaternion = \n" << q.coeffs() << std::endl;

    // // 四元数转旋转向量
    // rotation_vector = Eigen::AngleAxisd(q);

    // // 四元数转旋转矩阵
    // rotation_matrix = q.matrix();
    // // rotation_matrix = q.toRotationMatrix()

    // // 旋转矩阵转四元数
    // q = Eigen::Quaterniond(rotation_matrix);
    // std::cout << "quaternion = \n" << q.coeffs() << std::endl;

    // // 旋转矩阵转旋转向量
    // Eigen::AngleAxisd r_vector(rotation_matrix);

    // // 或者 r_vecotr.fromRotationMatrix(rotation_matrix)

    // // 使用四元数旋转一个向量
    // v_rotated = q * v;
    // std::cout << "(1 , 0 ,0) after rotation = \n" << v_rotated.transpose() << std::endl;

    return 0;
}
