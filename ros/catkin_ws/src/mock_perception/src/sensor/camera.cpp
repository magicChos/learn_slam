// #include "ace/sensor/camera.h"
// #include "ace/common/log.h"

#include "sensor/camera.h"
#include "common/log.h"
#include "utils/readyaml.h"
#include "utils/file_system.hpp"
#include "utils/portability_fixes.hpp"
#include "utils/wildcard.hpp"
#include <chrono>
#include <fstream>
#include <glog/logging.h>
#include <thread>
#include "utils/debug_utils.h"

namespace ace
{
    namespace sensor
    {

        Camera::~Camera()
        {
            return;
            PsReturnStatus status;
            status = Ps2_StopStream(deviceHandle, sessionIndex);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_StopStream failed");
                return;
            }

            status = Ps2_CloseDevice(deviceHandle);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_CloseDevice failed");
            }
        }

        bool Camera::Initialize(CameraOption option)
        {
            PsReturnStatus status;
            uint32_t slope = 1450;
            uint32_t deviceCount;

            sessionIndex = 0;

            status = Ps2_GetDeviceCount(&deviceCount);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_GetDeviceCount failed");
                return false;
            }
            // LOG(INFO) << "Found device count: " << deviceCount << "\n";
            
            LogInfo("Found device count: ");

            if (deviceIndex >= deviceCount)
            {
                LogError("Invalid device index");
                return false;
            }

            status = Ps2_Initialize();
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_Initialize failed");
                return false;
            }

            PsDeviceInfo *pDeviceListInfo = new PsDeviceInfo[deviceCount];
            status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_GetDeviceListInfo failed");
                return false;
            }

            status = Ps2_OpenDevice(pDeviceListInfo[deviceIndex].uri, &deviceHandle);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_OpenDevice failed");
                return false;
            }

            status = Ps2_StartStream(deviceHandle, sessionIndex);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_StartStream failed");
                return false;
            }

            status = Ps2_SetDepthRange(deviceHandle, sessionIndex, option.depthRange);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_SetDepthRange failed");
                return false;
            }

            status =
                Ps2_SetSynchronizeEnabled(deviceHandle, sessionIndex, option.synchronize);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_SetSynchronizeEnabled failed");
                return false;
            }

            status =
                Ps2_SetColorPixelFormat(deviceHandle, sessionIndex, option.pixelFormat);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_SetColorPixelFormat failed");
                return false;
            }

            // hach. wait for camera to be ready
            std::this_thread::sleep_for(std::chrono::seconds(10));

            status = Ps2_SetDataMode(deviceHandle, sessionIndex, option.dataMode);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("PsSetDataMode failed");
                return false;
            }

            status = Ps2_SetDepthDistortionCorrectionEnabled(
                deviceHandle, sessionIndex, option.depthDistortionCorrection);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_SetDepthDistortionCorrectionEnabled failed");
                return false;
            }

            status = Ps2_SetRGBDistortionCorrectionEnabled(
                deviceHandle, sessionIndex, option.rgbDistortionCorrection);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_SetRGBDistortionCorrectionEnabled failed");
                return false;
            }

            status = Ps2_SetSpatialFilterEnabled(deviceHandle, sessionIndex,
                                                 option.spatialFilter);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("PsSetSpatialFilterEnabled failed");
                return false;
            }

            status = Ps2_SetMapperEnabledDepthToRGB(deviceHandle, sessionIndex,
                                                    option.mappingDepthToRGB);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("PsSetMapperEnabledDepthToRGB failed");
                return false;
            }

            status = Ps2_SetMapperEnabledRGBToDepth(deviceHandle, sessionIndex,
                                                    option.mappingRGBToDepth);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("PsSetMapperEnabledRGBToDepth failed");
                return false;
            }

            PsCameraExtrinsicParameters extrinsicParams;
            status = Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex,
                                                      &extrinsicParams);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Ps2_GetCameraExtrinsicParameters failed");
                return false;
            }

            Eigen::Matrix3d Rd{extrinsicParams.rotation};
            Eigen::Vector3d Td{extrinsicParams.translation};
            R = Rd.cast<float>();
            T = Td.cast<float>();

            PsCameraParameters depthParams, rgbParams;
            status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor,
                                             &depthParams);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Get depth parameters failed");
                return false;
            }

            status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor,
                                             &rgbParams);
            if (status != PsReturnStatus::PsRetOK)
            {
                LogError("Get rgb parameters failed");
                return false;
            }
            C << rgbParams.cx, 0, rgbParams.fx, 0, rgbParams.cy, rgbParams.fy, 0, 0, 1;

            delete[] pDeviceListInfo;

            return true;
        }

        bool Camera::ReadFrame(cv::Mat &rgb, std::vector<Eigen::Vector3d> &pointCloud)
        {
            PsReturnStatus status;
            PsFrameReady frameReady = {0};

            status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
            if (status != PsRetOK)
            {
                LogError("PsReadNextFrame error");
                return false;
            }

            PsFrame depthFrame{0};
            PsFrame rgbFrame{0};
            if (frameReady.rgb == 1)
            {
                status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsRGBFrame error");
                    return false;
                }

                if (rgbFrame.pFrameData != NULL)
                {
                    rgb = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                  rgbFrame.pFrameData);
                }
            }
            else
            {
                LogError("RGB frame is not ready");
                return false;
            }
            if (frameReady.depth == 1)
            {
                status =
                    Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsDepthFrame error");
                    return false;
                }
                if (depthFrame.pFrameData != NULL)
                {
                    auto len = depthFrame.width * depthFrame.height;
                    std::unique_ptr<PsVector3f[]> data(new PsVector3f[len]);
                    // pointCloud.resize(len);

                    cv::Mat depth = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                                            depthFrame.pFrameData);

                    status = Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex,
                                                                depthFrame, data.get());
                    if (status != PsRetOK)
                    {
                        LogError("Ps2_ConvertDepthFrameToWorldVector error");
                        return false;
                    }
                    // pointCloud.resize(len);
                    // for (int i = 0; i < len; ++i) {
                    //   if (data[i].z == 0xFFFF)
                    //     data[i].z = 0;
                    //   pointCloud[i] = Eigen::Vector3d(data[i].x, data[i].z, -data[i].y);
                    // }

                    int skip_row = 0;
                    int skip_col = 0;

                    for (int i = 0; i < depthFrame.height; i = i + 1 + skip_row)
                    {
                        for (int j = 0; j < depthFrame.width; j = j + 1 + skip_col)
                        {
                            size_t index = i * depthFrame.width + j;

                            if (data[index].x < -490 || data[index].x > 450 || data[index].z > 1200 || data[index].z < 300 || data[index].y > 20 || data[index].y < -70)
                            {
                                continue;
                            }

                            if (data[index].x != 0 || data[index].y != 0 || data[index].z != 0)
                            {
                                pointCloud.emplace_back(Eigen::Vector3d(data[index].x, data[index].z, -data[index].y));
                            }
                        }
                    }
                }
            }
            else
            {
                LogError("Depth frame is not ready");
                return false;
            }
            return true;
        }

        bool Camera::ReadRGB(cv::Mat &rgb)
        {
            PsReturnStatus status;
            PsFrameReady frameReady = {0};

            status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
            if (status != PsRetOK)
            {
                LogError("PsReadNextFrame error");
                return false;
            }
            status = Ps2_SetRGBResolution(deviceHandle, sessionIndex,
                                          PsRGB_Resolution_640_480);
            if (status != PsRetOK)
            {
                LogError("Ps2_SetRGBResolution error");
                return false;
            }

            PsFrame rgbFrame{0};
            if (frameReady.rgb == 1)
            {
                status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsMappedRGBFrame error");
                    return false;
                }

                if (rgbFrame.pFrameData != NULL)
                {
                    rgb = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                  rgbFrame.pFrameData);
                }
            }
            else
            {
                return false;
            }

            return true;
        }

        bool Camera::ReadDepth(cv::Mat &depth)
        {
            PsReturnStatus status;
            PsFrameReady frameReady = {0};

            status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
            if (status != PsRetOK)
            {
                LogError("PsReadNextFrame error");
                return false;
            }

            PsFrame depthFrame{0};
            if (frameReady.depth == 1)
            {
                status =
                    Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsDepthFrame error");
                    return false;
                }

                if (depthFrame.pFrameData != NULL)
                {
                    depth = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                                    depthFrame.pFrameData);
                }
            }
            else
            {
                return false;
            }

            return true;
        }

        bool Camera::ReadRGBandDepth(cv::Mat &rgb, cv::Mat &depth)
        {
            PsReturnStatus status;
            PsFrameReady frameReady = {0};

            status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
            if (status != PsRetOK)
            {
                LogError("PsReadNextFrame error");
                return false;
            }

            status = Ps2_SetRGBResolution(deviceHandle, sessionIndex,
                                          PsRGB_Resolution_640_480);
            if (status != PsRetOK)
            {
                LogError("Ps2_SetRGBResolution error");
                return false;
            }

            PsFrame depthFrame{0};
            PsFrame rgbFrame{0};

            // start read rgb
            if (frameReady.rgb == 1)
            {
                status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsMappedRGBFrame error");
                    return false;
                }

                if (rgbFrame.pFrameData != NULL)
                {
                    rgb = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                  rgbFrame.pFrameData);
                }
            }
            else
            {
                return false;
            }
            // end read rgb

            // start read depth
            if (frameReady.depth == 1)
            {
                status =
                    Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsDepthFrame error");
                    return false;
                }

                if (depthFrame.pFrameData != NULL)
                {
                    depth = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                                    depthFrame.pFrameData);
                }
            }
            // end read depth
            return true;
        }

        bool Camera::ReadRGBandAlignDepth(cv::Mat &rgb, cv::Mat &align_depth)
        {
            PsReturnStatus status;
            PsFrameReady frameReady = {0};

            status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
            if (status != PsRetOK)
            {
                LogError("PsReadNextFrame error");
                return false;
            }

            status = Ps2_SetRGBResolution(deviceHandle, sessionIndex,
                                          PsRGB_Resolution_640_480);
            if (status != PsRetOK)
            {
                LogError("Ps2_SetRGBResolution error");
                return false;
            }

            status = Ps2_SetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, true);
            if (status != PsRetOK)
            {
                LogError("Ps2_SetMapperEnabledRGBToDepth error");
                return false;
            }

            PsFrame aligndepthFrame{0};
            PsFrame rgbFrame{0};

            // start read rgb
            if (frameReady.rgb == 1)
            {
                status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsMappedRGBFrame error");
                    return false;
                }

                if (rgbFrame.pFrameData != NULL)
                {
                    rgb = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                  rgbFrame.pFrameData);
                }
            }
            else
            {
                return false;
            }
            // end read rgb

            // start read align depth
            if (frameReady.mappedDepth == 1)
            {
                status = Ps2_GetFrame(deviceHandle, sessionIndex, PsMappedDepthFrame,
                                      &aligndepthFrame);
                if (status != PsRetOK)
                {
                    LogError("PsGetFrame PsMappedDepthFrame error");
                    return false;
                }

                if (aligndepthFrame.pFrameData != NULL)
                {
                    align_depth = cv::Mat(aligndepthFrame.height, aligndepthFrame.width,
                                          CV_16UC1, aligndepthFrame.pFrameData);
                    // for (int i = 0; i < align_depth.rows; ++i)
                    // {

                    //     for (int j = 0; j < align_depth.cols; ++j)
                    //     {
                    //         align_depth.at<short>(i, j) = (short)align_depth.at<short>(i,
                    //         j) * 255;
                    //     }
                    // }
                }
            }
            else
            {
                return false;
            }
            // end read align depth

            return true;
        }

        bool VirtualCamera::exportCameraIntrinsicParams(cv::Mat &camera_mat, cv::Mat &dist_mat)
        {

            return true;
        }

        void VirtualCamera::GetCalibration(Eigen::Matrix3f &params)
        {
            std::string current_dir = stlplus::folder_up(__FILE__);
            std::string calibration_file = current_dir + "../../config/camera_param.yaml";


            std::shared_ptr<YamlReader> yaml_reader_obj = std::make_shared<YamlReader>(calibration_file);
            cv::Mat camera_matrix;
            cv::Mat dist_matrix;
            cv::Mat extrinsic_matrix;
            yaml_reader_obj->getNodeMatrix("camera_matrix:" , camera_matrix);
            yaml_reader_obj->getNodeMatrix("dist_matrix:" , dist_matrix);
            yaml_reader_obj->getNodeMatrix("extrinsic_matrix" , extrinsic_matrix);

            C = Eigen::Matrix3f::Identity(3 , 3);
            C(0 , 0) = camera_matrix.at<float>(0 , 2);
            C(1 , 1) = camera_matrix.at<float>(1 , 2);
            C(0 , 2) = camera_matrix.at<float>(0 , 0);
            C(1 , 2) = camera_matrix.at<float>(1 , 1);
            params = C;


            cv::Mat R_matrix =  extrinsic_matrix.rowRange(0 , 3).colRange(0 , 3);
            R = Mat2MatrixXd(R_matrix).cast<float>();
            T << extrinsic_matrix.at<float>(0 , 3) , extrinsic_matrix.at<float>(1 , 3) , extrinsic_matrix.at<float>(2 , 3);
        }

        bool Camera::exportCameraExtrinsicParams(cv::Mat &extrinsic_mat)
        {
            return true;
        }

        bool Camera::exportCameraIntrinsicParams(cv::Mat &camera_mat,
                                                 cv::Mat &dist_mat)
        {
            PsReturnStatus status;
            status = Ps2_SetRGBResolution(deviceHandle, sessionIndex,
                                          PsRGB_Resolution_640_480);
            if (status != PsRetOK)
            {
                LogError("Ps2_SetRGBResolution error");
                return false;
            }

            PsCameraParameters camera_params;
            status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor,
                                             &camera_params);
            if (status != PsRetOK)
            {
                LogError("Ps2_GetCameraParameters error");
                return false;
            }

            camera_mat.at<float>(0, 0) = camera_params.fx;
            camera_mat.at<float>(1, 1) = camera_params.fy;
            camera_mat.at<float>(0, 2) = camera_params.cx;
            camera_mat.at<float>(1, 2) = camera_params.cy;

            dist_mat.at<float>(0, 0) = camera_params.k1;
            dist_mat.at<float>(0, 1) = camera_params.k2;
            dist_mat.at<float>(0, 2) = camera_params.p1;
            dist_mat.at<float>(0, 3) = camera_params.p2;
            dist_mat.at<float>(0, 4) = camera_params.k3;

            return true;
        }

        // bool Camera::exportCameraExtrinsicParams(cv::Mat &extrinsic_mat)
        // {
        //     PsReturnStatus status;
        //     status = Ps2_SetRGBResolution(deviceHandle, sessionIndex,
        //                                   PsRGB_Resolution_640_480);
        //     if (status != PsRetOK)
        //     {
        //         LogError("Ps2_SetRGBResolution error");
        //         return false;
        //     }

        //     PsCameraExtrinsicParameters extrinsic_params;
        //     status = Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex,
        //                                               &extrinsic_params);

        //     extrinsic_mat.at<float>(0, 0) = extrinsic_params.rotation[0];
        //     extrinsic_mat.at<float>(0, 1) = extrinsic_params.rotation[1];
        //     extrinsic_mat.at<float>(0, 2) = extrinsic_params.rotation[2];
        //     extrinsic_mat.at<float>(0, 3) = extrinsic_params.translation[0];

        //     extrinsic_mat.at<float>(1, 0) = extrinsic_params.rotation[3];
        //     extrinsic_mat.at<float>(1, 1) = extrinsic_params.rotation[4];
        //     extrinsic_mat.at<float>(1, 2) = extrinsic_params.rotation[5];
        //     extrinsic_mat.at<float>(1, 3) = extrinsic_params.translation[1];

        //     extrinsic_mat.at<float>(2, 0) = extrinsic_params.rotation[6];
        //     extrinsic_mat.at<float>(2, 1) = extrinsic_params.rotation[7];
        //     extrinsic_mat.at<float>(2, 2) = extrinsic_params.rotation[8];
        //     extrinsic_mat.at<float>(2, 3) = extrinsic_params.translation[2];

        //     return true;
        // }

    } // namespace sensor
} // namespace ace
