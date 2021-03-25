//
// Created by dyq on 2020/10/29.
//


#ifndef ACE_COMMON_CONFIG_H_
#define ACE_COMMON_CONFIG_H_

namespace ace {
namespace common {

constexpr char kConfigurationFilesDirectory[] =
    "@CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY@";
constexpr char kSourceDirectory[] = "@PROJECT_SOURCE_DIR@";

}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_CONFIG_H_
