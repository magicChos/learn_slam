//
// Created by dyq on 2020/12/8.
//

#include "gtest/gtest.h"
#include "ace/common/time.h"
#include "ace/common/log.h"
#include <chrono>

namespace ace {
namespace common {

TEST(Time, test01)
{
    common::Time time1;
    auto t1 = time1.time_since_epoch().count();

    sleep(1);
    auto t2 = time1.time_since_epoch().count();

    LogInfo("count : %d", t2 - t1);



}


}  // namespace common
}  // namespace ace

