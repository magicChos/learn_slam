#include "file_operator.h"

bool get_file_lst(std::vector<std::string> &files_lst, const std::string &suffix, const std::string &dir_path)
{
    if (suffix.empty() || dir_path.empty())
    {
        std::cerr << "please check input suffix or dir_path\n";
        return false;
    }

    files_lst = stlplus::folder_wildcard(dir_path, suffix, false, true);
    return true;
}

//字符串分割函数
std::vector<std::string> str_split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern; //扩展字符串以方便操作
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}