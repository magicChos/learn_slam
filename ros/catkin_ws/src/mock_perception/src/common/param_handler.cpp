//
// Created by dyq on 2020/10/23.
//

#include "ace/common/param_handler.h"
#include <iostream>

namespace ace {
namespace common {

ParamHandler::ParamHandler(const std::string &file_name)
{
    try
    {
        config_ = YAML::LoadFile(file_name);
        fileLoaded = true;
    }
    catch (std::exception& e)
    {
        fileLoaded = false;
    }
}

ParamHandler::~ParamHandler() {}

bool ParamHandler::GetString(const std::string &key, std::string &str_value)
{
    try
    {
        str_value = config_[key].as<std::string>();
    }
    catch (std::exception &e)
    {
        return false;
    }
    return true;
}

bool ParamHandler::GetString(const std::string &category, const std::string &key, std::string &str_value)
{
    try
    {
        str_value = config_[category][key].as<std::string>();
    }
    catch(std::exception &e)
    {
        return false;
    }
    return true;
}

bool ParamHandler::GetBoolean(const std::string &key, bool &bool_value)
{
    try
    {
        bool_value = config_[key].as<bool>();
    }
    catch (std::exception &e)
    {
        return false;
    }
    return true;
}

bool ParamHandler::GetInteger(const std::string &key, int &int_value)
{
    try
    {
        int_value = config_[key].as<int>();
    } catch (std::exception &e) {
        return false;
    }

    return true;
}

}  // namespace common
}  // namespace ace
