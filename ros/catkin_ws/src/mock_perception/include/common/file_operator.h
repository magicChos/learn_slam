#pragma once

#include "ace/perception/utils/file_system.hpp"
#include "ace/perception/utils/portability_fixes.hpp"
#include "ace/perception/utils/wildcard.hpp"
#include <iostream>

bool get_file_lst(std::vector<std::string> &files_lst, const std::string &suffix, const std::string &dir_path);

//字符串分割函数
std::vector<std::string> str_split(std::string str, std::string pattern);