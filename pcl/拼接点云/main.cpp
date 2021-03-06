#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


int main(int argc, char *argv[]){
    std::cout << "hello pcl" <<std::endl;
    std::vector<cv::Mat> colorImgs , depthImgs;
    // 相机位姿
    std::vector<Eigen::Isometry3d> poses;

    std::ifstream fin("../pose.txt");
    if(!fin){
        std::cerr << "打开pose文件失败" << std::endl;
        return 0;
    }

    for(int i = 0 ; i < 5 ; i++){
        boost::format fmt("../%s/%d.%s");

        colorImgs.push_back(cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back(cv::imread( (fmt%"depth"%(i+1)%"pgm").str() , -1));

        double data[7] = {0};
        for(auto & d : data) {
            fin >> d;
        }

        // 四元数
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        // 转旋转矩阵
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }

    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 

        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) 
                    continue; // 为0表示没有测量到
                Eigen::Vector3d point; 

                //  ||u||   | fx  0  cx|    |x|
                // z||v|| = | 0  fy  cy| x  |y|
                //  ||1||   | 0   0   1|    |z|
                // (x,y,z)为空间点再相机坐标系下坐标
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 

                // T:表示相机坐标系到世界坐标系的变换
                Eigen::Vector3d pointWorld = T*point;
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                p.b = int(color.at<cv::Vec3b>(u , v)[0]);
                p.g = int(color.at<cv::Vec3b>(u , v)[1]);
                p.r = int(color.at<cv::Vec3b>(u , v)[2]);
                pointCloud->points.push_back( p );
            }
    }
    pointCloud->height = 1;
    pointCloud->width = pointCloud->points.size();
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );

    return 1;
}