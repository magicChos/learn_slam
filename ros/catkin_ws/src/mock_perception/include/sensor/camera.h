#ifndef ACE_SENSOR_CAEMRA_H_
#define ACE_SENSOR_CAEMRA_H_

#include "Vzense/Vzense_api_710.h"
#include <Eigen/Core>
#include <cstdint>
#include <gmock/gmock.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace ace
{
    namespace sensor
    {

        class CameraInterface
        {
        public:
            virtual ~CameraInterface() {}
            virtual bool ReadFrame(cv::Mat &rgb,
                                   std::vector<Eigen::Vector3d> &pointCloud) = 0;
            virtual void GetCalibration(Eigen::Matrix3f &params) = 0;
            virtual void GetExtrinsic(Eigen::Matrix3f &rotation,
                                      Eigen::Vector3f &translation) = 0;
        };

        class CameraOption
        {
        public:
            bool spatialFilter = true;
            bool depthDistortionCorrection = true;
            bool rgbDistortionCorrection = true;
            bool mappingDepthToRGB = false;
            bool mappingRGBToDepth = false;
            bool synchronize = true;
            PsDataMode dataMode = PsDepthAndRGB_30;
            PsDepthRange depthRange = PsNearRange;
            PsPixelFormat pixelFormat = PsPixelFormatBGR888;
        };

        class Camera : public CameraInterface
        {
        public:
            Camera(int32_t deviceIndex) : deviceIndex(deviceIndex){};
            ~Camera();
            bool Initialize(CameraOption option = CameraOption{});
            bool
            ReadFrame(cv::Mat &rgb,
                      std::vector<Eigen::Vector3d> &pointCloud); // TODO: use our vector3

            // get rgb image
            bool ReadRGB(cv::Mat &rgb);

            // get depth image
            bool ReadDepth(cv::Mat &depth);

            // get rgb and depth
            bool ReadRGBandDepth(cv::Mat &rgb, cv::Mat &depth);

            // get rgb and align depth
            bool ReadRGBandAlignDepth(cv::Mat &rgb, cv::Mat &align_depth);

            // export camera intrinsic params
            bool exportCameraIntrinsicParams(cv::Mat &camera_mat, cv::Mat &dist_mat);

            bool exportCameraExtrinsicParams(cv::Mat &extrinsic_mat);

            void GetCalibration(Eigen::Matrix3f &params) { params = C; }
            void GetExtrinsic(Eigen::Matrix3f &rotation, Eigen::Vector3f &translation)
            {
                rotation = R;
                translation = T;
            }

        private:
            int32_t deviceIndex;
            PsDeviceHandle deviceHandle;
            uint32_t sessionIndex;

            Eigen::Matrix3f C, R;
            Eigen::Vector3f T;
        };

        class MockCamera : public CameraInterface
        {
        public:
            MOCK_METHOD(bool, ReadFrame,
                        (cv::Mat & rgb, std::vector<Eigen::Vector3d> &pointCloud),
                        (override));
            MOCK_METHOD(void, GetCalibration, (Eigen::Matrix3f & params), (override));
            MOCK_METHOD(void, GetExtrinsic,
                        (Eigen::Matrix3f & rotation, Eigen::Vector3f &translation),
                        (override));
        };

        class VirtualCamera : public CameraInterface
        {
        public:
            bool ReadFrame(cv::Mat &rgb,
                           std::vector<Eigen::Vector3d> &pointCloud)
            {
                return true;
            }
            bool Initialize(CameraOption option = CameraOption{});

            // export camera intrinsic params
            bool exportCameraIntrinsicParams(cv::Mat &camera_mat, cv::Mat &dist_mat);

            bool exportCameraExtrinsicParams(cv::Mat &extrinsic_mat);

            void GetCalibration(Eigen::Matrix3f &params);
            void GetExtrinsic(Eigen::Matrix3f &rotation, Eigen::Vector3f &translation)
            {
                rotation = R;
                translation = T;
            }

        private:
            Eigen::Matrix3f C, R;
            Eigen::Vector3f T;
        };

    } // namespace sensor
} // namespace ace

#endif
