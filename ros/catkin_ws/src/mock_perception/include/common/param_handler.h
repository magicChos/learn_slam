//
// Created by dyq on 2020/10/23.
//

#ifndef ACE_COMMON_PARAMETER_HANDLER_H_
#define ACE_COMMON_PARAMETER_HANDLER_H_

#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>

namespace ace {
namespace common {

class ParamHandler {
public:
    ParamHandler(const std::string &file_name);
    virtual ~ParamHandler();

    bool GetString(const std::string &key, std::string &str_value);
    bool GetString(const std::string &category, const std::string &key, std::string & str_value);

    template<typename T>
    bool GetVector(const std::string &key, std::vector<T> &vec_value)
    {
        try
        {
            vec_value = config_[key].as<std::vector<T> >();
        }
        catch (std::exception &e)
        {
            return false;
        }

        return true;
    }

    template<typename T>
    bool GetVector(const std::string &category, const std::string &key, std::vector<T> &vec_value)
    {
        try
        {
            vec_value = config_[category][key].as<std::vector<T>>();
        }
        catch (std::exception &e)
        {
            return false;
        }
        return true;
    }

    template<typename T>
    bool Get2DArray(const std::string &category, const std::string &key, std::vector<std::vector<T> > &vec_value)
    {
        try {
            vec_value = config_[category][key].as<std::vector<std::vector<T> > >();
        } catch (std::exception &e) {
            return false;
        }
        return true;
    }

    template<typename T>
    bool GetValue(const std::string &key, T &T_value)
    {
        try {
            T_value = config_[key].as<T>();
        } catch (std::exception &e) {
            return false;
        }
        return true;
    }

    template<typename T>
    bool GetValue(const std::string &category, const std::string &key, T &T_value)
    {
        try {
            T_value = config_[category][key].as<T>();
            return true;
        } catch (std::exception &e) {
            return false;
        }
        return true;
    }

    bool GetBoolean(const std::string & category, const std::string &key, bool &bool_value)
    {
        try
        {
            bool_value = config_[category][key].as<bool>();
            return true;
        }
        catch (std::exception &e)
        {
            return false;
        }
        return true;
    }


    std::vector<std::string> KetKeys()
    {
        std::vector<std::string> v;
        v.reserve(config_.size());

        for(auto it = config_.begin(); it != config_.end(); it++)
        {
            v.push_back(it->first.as<std::string>());
        }
        return v;
    }


    bool GetBoolean(const std::string &key, bool &bool_value);
    bool GetInteger(const std::string &key, int &int_value);

    bool FileOpenedSuccessfully()
    {
        return fileLoaded;
    }

protected:
    YAML::Node config_;

private:
    bool fileLoaded = false;
};

}  // namespace common
}  // namespace ace

#endif // ACE_COMMON_PARAMETER_HANDLER_H_
