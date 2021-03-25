//
// Created by dyq on 2020/11/27.
//


#include "ace/common/log.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ace/common/utils.h"

#include <string>

namespace ace {
namespace common {
namespace {

TEST(LogTest, test)
{
    int count = 0;

    std::string str = "Helle log start.";

    LogInfo("This info message: %d", count++);
    LogDebug("This info message: %d", count++);
    LogWarn("This info message: %d", count++);
    LogError("This info message: %d", count++);
    LogFatal("This info message: %d", count++);

    for (int i = 0; i < 100; i++)
    {
        LogInfo("This info message: %s", str.c_str());
        sleep(0.5);
    }

}

}
}  // namespace common
}  // namespace ace
