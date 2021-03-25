//
// Created by dyq on 2020/10/29.
//

#ifndef ACE_COMMON_CONFIGURATION_FILE_RESOLVER_H_
#define ACE_COMMON_CONFIGURATION_FILE_RESOLVER_H_

#include <vector>

#include "ace/common/lua_parameter_dictionary.h"
#include "ace/common/port.h"

namespace ace {
namespace common {

class ConfigurationFileResolver : public FileResolver
{
public:
    explicit ConfigurationFileResolver(
      const std::vector<std::string>& configuration_files_directories);

    std::string GetFullPathOrDie(const std::string& basename) override;
    std::string GetFileContentOrDie(const std::string& basename) override;

private:
    std::vector<std::string> configuration_files_directories_;
};

}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_CONFIGURATION_FILE_RESOLVER_H_
