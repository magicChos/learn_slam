#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
using namespace std;
using namespace cv;

cv::Mat img_1;
cv::Mat img_2;

std::vector<cv::Point> left_pts;
std::vector<cv::Point> right_pts;

void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1,
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches );

void pose_estimation_2d2d (
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        Mat& R, Mat& t );

void triangulation (
        const vector<KeyPoint>& keypoint_1,
        const vector<KeyPoint>& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector<Point3d>& points
        );

// 像素坐标转相机归一化坐标
Point2f pixel2cam( const Point2d& p, const Mat& K );

void On_mouse_left(int event, int x, int y, int flags, void*)//每次点击左键，将将当前点坐标存储到txt文件中，并在相应位置画红点
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Point recent_Point = cv::Point(x, y);
        left_pts.push_back(recent_Point);
        
        cv::circle(img_1, recent_Point, 10, Scalar(0, 0, 255), 2);
        
    }
}

void On_mouse_right(int event , int x , int y , int flags , void *)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Point recent_Point = cv::Point(x, y);
        right_pts.push_back(recent_Point);

        cv::circle(img_2, recent_Point, 10, Scalar(0, 0, 255), 2);

    }
}

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: triangulation img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    img_1 = imread ( argv[1], 1 );
    img_2 = imread ( argv[2], 1 );

    cv::namedWindow ("image1");
    cv::setMouseCallback("image1" , On_mouse_left);

    while (true)
    {
        cv::imshow("image1" , img_1);
        char key = cv::waitKey(25);
        if(key == 'q')
            break;


    }

    cv::destroyAllWindows();

    cv::namedWindow("image2");
    cv::setMouseCallback("image2" , On_mouse_right);

    while (true)
    {
        cv::imshow("image2" , img_2);
        char key = cv::waitKey(25);
        if(key == 'q')
            break;

    }
    cv::destroyAllWindows();

    std::cout << "left image 选点 \n";
    for(auto pt : left_pts)
    {
        std::cout << pt;
    }
    std::cout << "right image 选点 \n";
    for(auto pt: right_pts)
    {
        std::cout << pt << std::endl;
    }


    int height = img_1.rows;
    int width = img_1.cols;

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 三角化
    vector<Point3d> points;
//    triangulation( keypoints_1, keypoints_2, matches, R, t, points );

    Mat T1 = (Mat_<float> (3,4) <<
              1,0,0,0,
              0,1,0,0,
              0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
              R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
              R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
              R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
              );
    Mat K = ( Mat_<double> ( 3,3 ) << 500, 0, 960, 0, 500, 443, 0, 0, 1 );
    std::vector<cv::Point2f> pts_1 , pts_2;
    for (int i = 0 ; i < left_pts.size(); ++i)
    {
        pts_1.push_back(pixel2cam(left_pts[i] , K));
        pts_2.push_back(pixel2cam(right_pts[i] , K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1 , T2 , pts_1 , pts_2 , pts_4d);
    // 转换成非齐次坐标
    // 每一列代表一个三维点

    std::vector<cv::Point3d> points_3d;
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        // (X , Y , Z , 1)
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        // 三维点
        Point3d p (
                    x.at<float>(0,0),
                    x.at<float>(1,0),
                    x.at<float>(2,0)
                    );
        points_3d.push_back( p );
    }

    std::vector<double> vec1_dst;
    for(int i = 0 ; i < 4 ; ++i)
    {
        cv::Point3d start_pt = points_3d[i];
        cv::Point3d end_pt   = points_3d[(i + 1) % 4];

        double dist = std::sqrt((end_pt.x - start_pt.x) * (end_pt.x - start_pt.x) + (end_pt.y - start_pt.y) * (end_pt.y - start_pt.y) + (end_pt.z - start_pt.z) * (end_pt.z - start_pt.z));
        vec1_dst.push_back(dist);
        std::cout << "dist: " << dist << std::endl;
    }

    double avg1 = (vec1_dst[1] + vec1_dst[3])/2.;


    std::vector<double> vec2_dist;
    for(int i = 4 ; i < 8 ; ++i)
    {
        cv::Point3d start_pt = points_3d[i];
        cv::Point3d end_pt   = points_3d[(i + 1) % 4 + 4];

        double dist = std::sqrt((end_pt.x - start_pt.x) * (end_pt.x - start_pt.x) + (end_pt.y - start_pt.y) * (end_pt.y - start_pt.y) + (end_pt.z - start_pt.z) * (end_pt.z - start_pt.z));
        vec2_dist.push_back(dist);
        std::cout << "dist: " << dist << std::endl;
    }

    double avg2 = (vec2_dist[0] + vec2_dist[1] + vec2_dist[2] + vec2_dist[3])/4.;

    double template_size = 600;
    std::cout << "招牌尺寸： " << avg1 * template_size/avg2 << std::endl;


//    //-- 验证三角化点与特征点的重投影关系
//    Mat K = ( Mat_<double> ( 3,3 ) << 500, 0, 960, 0, 500, 443, 0, 0, 1 );

//    std::cout << "K : " << K << std::endl;

//    for ( int i=0; i<matches.size(); i++ )
//    {
//        Point2d pt1_cam = pixel2cam( keypoints_1[ matches[i].queryIdx ].pt, K );
//        Point2d pt1_cam_3d(
//                    points[i].x/points[i].z,
//                    points[i].y/points[i].z
//                    );
        
//        cv::Point pix1_cam;
//        pix1_cam.x = (int)(K.at<double>(0 , 0) * points[i].x + K.at<double>(0 , 2) * points[i].z)/points[i].z;
//        pix1_cam.y = (int)(K.at<double>(1 , 1) * points[i].y + K.at<double>(1 , 2) * points[i].z)/points[i].z;

//        cv::circle(img_1 , pix1_cam , 5 , cv::Scalar(0 , 0 , 255));
        

//        // cout<<"point in the first camera frame: "<<pt1_cam<<endl;
//        // cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;

//        // 第二个图
//        Point2f pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, K );
//        cv::Point pix2_cam;
//        cv::Point3d cam_pt2;
//        cam_pt2.x = T2.at<float>(0 , 0) * points[i].x + T2.at<float>(0 , 1) * points[i].y + T2.at<float>(0 , 2) * points[i].z + T2.at<float>(0 , 3);
//        cam_pt2.y = T2.at<float>(1 , 0) * points[i].x + T2.at<float>(1 , 1) * points[i].y + T2.at<float>(1 , 2) * points[i].z + T2.at<float>(1 , 3);
//        cam_pt2.z = T2.at<float>(2 , 0) * points[i].x + T2.at<float>(2 , 1) * points[i].y + T2.at<float>(2 , 2) * points[i].z + T2.at<float>(2 , 3);


//        std::cout << T2.at<float>(0 , 0) << " , " << T2.at<float>(0 , 1) << " , " << T2.at<float>(0 , 2) << std::endl;

//        pix2_cam.x = (int)(K.at<double>(0 , 0) * cam_pt2.x + K.at<double>(0 , 2) * cam_pt2.z)/cam_pt2.z;
//        pix2_cam.y = (int)(K.at<double>(1 , 1) * cam_pt2.y + K.at<double>(1 , 2) * cam_pt2.z)/cam_pt2.z;

//        std::cout << "pix2_cam: " << pix2_cam << std::endl;

//        cv::circle(img_2 , pix2_cam , 5 , cv::Scalar(0 , 0 , 255));

//        std::cout << "R : " << R << std::endl;
//        Mat pt2_trans = R*( Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
//        pt2_trans /= pt2_trans.at<double>(2,0);
//        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
//        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
//        // cout<<endl;
//    }

//    cv::imshow("img1" , img_1);
//    cv::imshow("img2" , img_2);
//    cv::waitKey(0);

    return 0;
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

void pose_estimation_2d2d (
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 500, 0, 960, 0, 500, 443, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2 );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 960, 443 );				//相机主点, TUM dataset标定值
    int focal_length = 521;						//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}

void triangulation (
        const vector< KeyPoint >& keypoint_1,
        const vector< KeyPoint >& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
              1,0,0,0,
              0,1,0,0,
              0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
              R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
              R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
              R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
              );

    Mat K = ( Mat_<double> ( 3,3 ) << 500, 0, 960, 0, 500, 443, 0, 0, 1 );
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
    }

    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

    // 转换成非齐次坐标
    // 每一列代表一个三维点
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        // (X , Y , Z , 1)
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        // 三维点
        Point3d p (
                    x.at<float>(0,0),
                    x.at<float>(1,0),
                    x.at<float>(2,0)
                    );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
            (
                ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
                ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
                );
}

