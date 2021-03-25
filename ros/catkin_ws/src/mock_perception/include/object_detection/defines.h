#pragma once

#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp>
#include "object_types.h"


typedef float track_t;
typedef cv::Point_<track_t> Point_t;
#define El_t CV_32F
#define Mat_t CV_32FC

typedef std::vector<int> assignments_t;
typedef std::vector<track_t> distMatrix_t;

/*
* \brief config_t
*/
typedef std::multimap<std::string, std::string> config_t;

/*
* \brief The CRegion class
*/
class CRegion
{
public:
    CRegion() = default;

    CRegion(const cv::Rect &rect)
        : m_brect(rect)
    {
        B2RRect();
    }

    CRegion(const cv::RotatedRect &rrect)
        : m_rrect(rrect)
    {
        R2BRect();
    }

    CRegion(const cv::Rect &rect, objtype_t type, float confidence)
        : m_type(type), m_brect(rect), m_confidence(confidence)
    {
        B2RRect();
    }

    objtype_t m_type = bad_type;
    cv::RotatedRect m_rrect;
    cv::Rect m_brect;
    float m_confidence = -1;

private:
    cv::Rect R2BRect()
    {
        m_brect = m_rrect.boundingRect();
        return m_brect;
    }

    cv::RotatedRect B2RRect()
    {
        m_rrect = cv::RotatedRect(m_brect.tl(), cv::Point2f(static_cast<float>(m_brect.x + m_brect.width), static_cast<float>(m_brect.y)), m_brect.br());
        return m_rrect;
    }
};

struct OBJECT
{
    cv::Rect_<float> rect;
    int label;
    float prob;

    int track_id;
};

typedef std::vector<OBJECT> objects_t;
typedef std::vector<CRegion> regions_t;

namespace tracking
{
    /*
* \brief The Detectors enum
*/
    enum Detectors
    {
        ACE_YOLO_FASTEST = 0,
        DetectorsCount
    };

    /*
* \brief The DistType enum
*/
    enum DistType
    {
        DistCenters, // Euclidean distance between centers, pixels
        DistRects,   // Euclidean distance between bounding rectangles, pixels
        DistJaccard, // Intersection over Union, IoU, [0, 1]
        DistHist,    // Bhatacharia distance between histograms, [0, 1]
        DistsCount
    };

    /*
* \brief The FilterGoal enum
*/
    enum FilterGoal
    {
        FilterCenter,
        FilterRect,
        FiltersCount
    };

    /*
* \brief The KalmanType enum
*/
    enum KalmanType
    {
        KalmanLinear,
        KalmanUnscented,
        KalmanAugmentedUnscented,
        KalmanCount
    };

    /*
* \brief The MatchType enum
*/
    enum MatchType
    {
        MatchHungrian,
        MatchBipart,
        MatchCount
    };

    /*
* \brief The LostTrackType enum
*/
    enum LostTrackType
    {
        TrackNone,
        TrackKCF,
        TrackMIL,
        TrackMedianFlow,
        TrackGOTURN,
        TrackMOSSE,
        TrackCSRT,
        TrackDAT,
        TrackSTAPLE,
        TrackLDES,
        SingleTracksCount
    };
}
