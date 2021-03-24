#pragma once

#include "module/base_module.h"
#include "subscriber/color_subscriber.h"
#include "subscriber/cloud_subscriber.h"
#include "subscriber/map_subscriber.h"
#include "tf_listener/tf_listener.hpp"
#include <memory>

class MockSystem
{
public:
    MockSystem(std::shared_ptr<BaseModule> module);
    ~MockSystem();

protected:
    std::shared_ptr<BaseModule> m_module;

private:
    std::shared_ptr<ColorSubscriber> m_color_sub;
    std::shared_ptr<CloudSubscriber> m_cloud_sub;
    std::shared_ptr<MapSubscriber> m_map_sub;
    std::shared_ptr<TFListener> m_listener;

    ros::NodeHandle m_nh;
};