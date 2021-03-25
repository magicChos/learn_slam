//
// Created by dyq on 2020/10/29.
//

#include "ace/common/param_handler.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <string>

namespace ace {
namespace common {

std::string GetConfigDirectoryPath() { return "./../../configuration_files/"; }

TEST(YAMLTest, yaml_category_read) {
#if 0
    ParamHandler paramHandler(GetConfigDirectoryPath() + "test-yaml.yaml");

    EXPECT_TRUE(paramHandler.FileOpenedSuccessfully());
    double c1p1, c2p1;
    std::vector<double> c1p2, c2p2;


    EXPECT_TRUE(paramHandler.GetValue("category-1", "parameter-1", c1p1));
    EXPECT_TRUE(paramHandler.GetValue("category-2", "parameter-1", c2p1));
    EXPECT_TRUE(paramHandler.GetValue("category-1", "parameter-2", c1p2));
    EXPECT_TRUE(paramHandler.GetValue("category-2", "parameter-2", c2p2));

  // EXPECT_TRUE(paramHandler.FileOpenedSuccessfully());
  // double c1p1, c2p1;
  // std::vector<double> c1p2, c2p2;
  // EXPECT_TRUE(paramHandler.GetValue("category-1", "parameter-1", c1p1));
  // EXPECT_TRUE(paramHandler.GetValue("category-2", "parameter-1", c2p1));
  // EXPECT_TRUE(paramHandler.GetValue("category-1", "parameter-2", c1p2));
  // EXPECT_TRUE(paramHandler.GetValue("category-2", "parameter-2", c2p2));

  // EXPECT_TRUE(c1p1 == 12.);
  // EXPECT_TRUE(c1p2[0] == 1.);
  // EXPECT_TRUE(c1p2[1] == 2.);
  // EXPECT_TRUE(c1p2[2] == 3.);
  // EXPECT_TRUE(c1p2.size() == 3);

    EXPECT_FALSE(paramHandler.GetValue("notARealKey", c1p1));
    ParamHandler badHandler("not-a-real-file.potato");
    EXPECT_FALSE(badHandler.FileOpenedSuccessfully());
#endif
}

} // namespace common
} // namespace ace
