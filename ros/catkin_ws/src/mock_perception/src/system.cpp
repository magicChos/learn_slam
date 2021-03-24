#include "system.h"

MockSystem::MockSystem(std::shared_ptr<BaseModule> module)
{
    std::cout << "step into mock system" << std::endl;

    m_module = module;

    m_color_sub = std::make_shared<ColorSubscriber>(m_nh, "/pico_camera/color_image", 10000);
    m_cloud_sub = std::make_shared<CloudSubscriber>(m_nh, "/pico_camera/point_cloud", 10000);
    m_map_sub = std::make_shared<MapSubscriber>(m_nh, "/map", 10000);
    m_listener = std::make_shared<TFListener>(m_nh, "map", "base_footprint");
}

MockSystem::~MockSystem()
{
}