
#ifndef ACE_PERCEPTION_OBSTACLE_DETECTION_H_
#define ACE_PERCEPTION_OBSTACLE_DETECTION_H_

#include "object_detection/object_detection.h"
#include "utils/perception_time.h"
#include "sensor/camera.h"
#include <Eigen/Core>
#include <cstdint>
#include "common/log.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "utils/file_system.hpp"
#include "utils/portability_fixes.hpp"
#include "utils/wildcard.hpp"

namespace ace
{
    namespace perception
    {

        class ObjectDetectionWrapper
        {

        public:
            static ObjectDetectionWrapper &get()
            {
                static ObjectDetectionWrapper wrapper;
                return wrapper;
            }

            ObjectDetectionWrapper(ObjectDetectionWrapper const &) = delete;
            void operator=(ObjectDetectionWrapper const &) = delete;
            bool Detect(cv::Mat image, std::vector<OBJECT> &objects);

        private:
            ObjectDetectionWrapper()
            {
                std::string current_dir = stlplus::folder_up(__FILE__);
                std::string model_path = current_dir + "../../weights/object_detection/";
                if (sric_detection_init(model_path) < 0)
                {
                    // LOG(ERROR) << "Object detection initialization failed\n";
                    LogError("Object detection initialization failed");
                }
            }
            ~ObjectDetectionWrapper() { sric_detection_release(); }
        };

        class ObstacleDetectOption
        {
        public:
            double resolution = 10.0; // in mm
            int width = 200, height = 200;
            double lidarTop = 70.0, baseTop = 30.0, armTop = 0.0, armBottom = -20.0,
                   baseBottom = -30.0;
            bool detectObjects = false;
            bool debug = false;
            double minimumVisibleDistance = 350.0;
            double maximumVisibleDistance = 1200.0;
            double hfov = 70.0 / 180.0 * atan(1) * 4;
            bool useTof = true;
            int64_t elapse_time = 30000;
            int pix_thresh = 127;
        };

        class ObstacleDetector
        {
        public:
            ObstacleDetector(ace::sensor::CameraInterface *camera,
                             ObstacleDetectOption &option);

            bool GenerateLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, cv::Mat &localmap);

            bool getSingleLevelLocalMap(const std::vector<Eigen::Vector3d> &pointCloud,
                                        cv::Mat &map);

            bool GetLocalMap(cv::Mat &map);
            bool GetLocalMap(uint8_t *map);

        private:
            bool detectObjects(cv::Mat &map, const cv::Mat rgb);
            bool getThreeLevelLocalMap(const std::vector<Eigen::Vector3d> &pointCloud,
                                       cv::Mat &map);

            bool drawDetects(const std::vector<OBJECT> &dets, cv::Mat &input_img);

            bool radisuFilter(const std::vector<Eigen::Vector3d> &input_pointCloud, std::vector<Eigen::Vector3d> &output_pointCloud, int size = 20, float radius = 0.1);

            void cameraParamsInit();

            int contourFilter(cv::Mat &input_map);

        private:
            ace::sensor::CameraInterface *m_camera;
            ObstacleDetectOption option;
            std::shared_ptr<Timer> m_timer;
            Eigen::Matrix3f m_C;
            Eigen::Vector3f m_rgbOrigin3D;

            double m_blockProb;
        };

    } // namespace perception
} // namespace ace

#endif
