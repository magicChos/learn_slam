#include "DBoW3/DBoW3.h"
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <stlplus/filesystemSimplified/file_system.hpp>
#include <stlplus/filesystemSimplified/portability_fixes.hpp>
#include <stlplus/filesystemSimplified/wildcard.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <SelfFileOperation/include/FileOperator.h>

using namespace cv;
using namespace std;
using namespace FileOperator;

/***************************************************
 * 本节演示了如何根据data/目录下的十张图训练字典
 * ************************************************/

int main(int argc, char **argv)
{
    // read the image
    cout << "reading images... " << endl;
    vector<Mat> images;

    std::vector<std::string> image_lists;
    list_images(image_lists, "/Users/han/Opencv_Project/dbow3/build/data", "*.png");
    for (auto name : image_lists)
    {
        images.emplace_back(cv::imread(name));
        std::cout << name << std::endl;
    }

    // detect ORB features
    cout<<"detecting ORB features ... "<<endl;
    Ptr< Feature2D > detector = ORB::create();
    vector<Mat> descriptors;

    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
    }

    // create vocabulary
    cout<<"creating vocabulary ... "<<endl;
    DBoW3::Vocabulary vocab;
    vocab.create( descriptors );
    cout<<"vocabulary info: "<<vocab<<endl;
    vocab.save( "vocabulary.yml.gz" );
    cout<<"done"<<endl;

    return 0;
}

