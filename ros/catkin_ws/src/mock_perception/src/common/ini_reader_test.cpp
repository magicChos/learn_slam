//
// Created by dyq on 2020/12/1.
//

#include "ace/common/ini_reader.h"
#include "ace/common/log.h"
#include "ace/common/utils.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace ace {
namespace common {
namespace {


#define TEST_FILE "test.ini"

TEST(INIReader, test)
{
    INIReader reader(GetConfigurationDirectory() + TEST_FILE);

    if (reader.ParseError() < 0)
    {
        LogInfo("Can't load 'test.ini");
         return;
    }

    LogInfo("Config loaded from 'test.ini': version= %d",
        reader.GetInteger("protocol", "version", -1));

    LogInfo("name  = %s", reader.Get("user", "name", "UNKNOWN").c_str());
    LogInfo("email = %s", reader.Get("user", "email", "UNKNOWN").c_str());
    LogInfo("pi    = %lf", reader.GetReal("user", "pi", -1));
    LogInfo("pi    = %d", reader.GetBoolean("user", "active", true));
}

}
}  // namespace common
}  // namespace ace