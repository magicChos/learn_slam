//
// Created by dyq on 2020/10/28.
//


#include "ace/common/fixed_ratio_sampler.h"

#include "glog/logging.h"

namespace ace {
namespace common {

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio)
{
    CHECK_GE(ratio, 0.);
    LOG_IF(WARNING, ratio == 0.) << "FixedRatioSampler is dropping all data.";
    CHECK_LE(ratio, 1.);
}

FixedRatioSampler::~FixedRatioSampler() {}

bool FixedRatioSampler::Pulse()
{
    ++num_pulses_;
    if (static_cast<double>(num_samples_) / num_pulses_ < ratio_)
    {
        ++num_samples_;
        return true;
    }

    return false;
}

std::string FixedRatioSampler::DebugString()
{
    return std::to_string(num_samples_) + " (" +
         std::to_string(100. * num_samples_ / num_pulses_) + "%)";
}

}  // namespace common
}  // namespace ace

