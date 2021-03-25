//
// Created by dyq on 2020/10/23.
//


#ifndef ACE_COMMON_UTILS_H_
#define ACE_COMMON_UTILS_H_

#include <string>
#include <stdlib.h>

namespace ace {
namespace common {

template <typename MapType, typename KeyType = typename MapType::key_type,
          typename ValueType = typename MapType::mapped_type>
ValueType* FindOrNull(MapType& map, const KeyType& key)
{
    auto it = map.find(key);
    if (it == map.end()) return nullptr;

    return &(it->second);
}


std::string GetConfigurationDirectory();


}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_UTILS_H_
