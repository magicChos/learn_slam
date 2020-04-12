#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std;

void panorama_to_Cloud(const std::string &panoramaImageName , std::string output_CloudName);

void write_ply(const std::string output_plyFile , std::vector<std::tuple<double , double , double , int , int , int> > &clouds);
int main()
{
    cout << "Hello World!" << endl;
    std::string imageName = "/Users/han/Desktop/0166.jpg";
    std::string savePlyName = "save.ply";

    auto start_time = std::chrono::steady_clock::now();

    panorama_to_Cloud(imageName , savePlyName);
    auto end_time = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "cost time: " << cost_time << " 毫秒" << std::endl;
    return 0;
}

void panorama_to_Cloud(const std::string &panoramaImageName , std::string output_CloudName)
{
    cv::Mat img = cv::imread(panoramaImageName);
    if(img.empty())
    {
        std::cerr << "open image Fail" << std::endl;
        return;
    }

    int row = img.rows;
    int col = img.cols;

    double R = (double)col / (2 * CV_PI);

    std::map<int , double> map_sinx , map_cosx , map_siny , map_cosy;

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
    for(int c = 0 ; c < col ; ++c)
    {
        map_sinx.insert(std::pair<int , double>(c , sin(c/R)));
        map_cosx.insert(std::pair<int , double>(c , cos(c/R)));
    }

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
    for (int r = 0; r < row ; ++r)
    {
        map_siny.insert(std::pair<int , double>(r , sin(r/R)));
        map_cosy.insert(std::pair<int , double>(r , cos(r/R)));
    }

    std::vector<std::tuple<double , double , double , int , int , int> > all_clouds;

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
    for (int r = 0; r < row; ++r)
    {
        for (int c = 0; c < col; ++c)
        {
            double x = (double)c;
            double y = (double)r;
            double theta = x/R;
            double phi = y/R;

            double X = R * sin(phi) * sin(theta);
            double Y = R*sin(phi)*cos(theta);
            double Z = R*cos(phi);

            int color_b = img.at<cv::Vec3b>(r, c)[0];
            int color_g = img.at<cv::Vec3b>(r, c)[1];
            int color_r = img.at<cv::Vec3b>(r, c)[2];

            std::tuple<double , double , double , int , int , int> per_point;
            std::get<0>(per_point) = X;
            std::get<1>(per_point) = Y;
            std::get<2>(per_point) = Z;
            std::get<3>(per_point) = color_r;
            std::get<4>(per_point) = color_g;
            std::get<5>(per_point) = color_b;

            all_clouds.emplace_back(per_point);
        }
    }


    std::cout << "一共有：" << all_clouds.size() << "个点" << std::endl;
    write_ply(output_CloudName , all_clouds);
}

void write_ply(const std::string output_plyFile , std::vector<std::tuple<double , double , double , int , int , int> > &clouds)
{
    std::ofstream outfile(output_plyFile.c_str() , std::ios::binary| std::ios::out);
    if(!outfile.is_open())
        return;
    outfile << "ply"
            << '\n'
            << "format ascii 1.0"
            << '\n'
            << "element vertex " << clouds.size()
            << '\n'
            << "property double x"
            << '\n'
            << "property double y"
            << '\n'
            << "property double z"
            << '\n'
            << "property uchar red"
            << '\n'
            << "property uchar green"
            << '\n'
            << "property uchar blue"
            << '\n'
            << "end_header" << std::endl;

    outfile << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);

    for (auto p : clouds)
    {
        outfile << std::get<0>(p) << ' ' << std::get<1>(p) << ' ' << std::get<2>(p) << ' ' << std::get<3>(p) << ' ' << std::get<4>(p) << " " << std::get<5>(p) << "\n";
    }

    outfile.flush();
    outfile.close();
}
