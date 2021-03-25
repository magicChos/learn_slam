//
// Created by dyq on 2020/12/1.
//

#include "ace/common/ini.h"
#include "ace/common/ini_reader.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace ace {
namespace common {

using std::string;

INIReader::INIReader(const std::string& filename)
{
    error_ = INIParse(filename.c_str(), ValueHandler, this);
}

INIReader::~INIReader()
{
    // Clean up the field sets
    std::map<std::string, std::set<std::string>*>::iterator fieldSetsIt;
    for (fieldSetsIt = fields_.begin(); fieldSetsIt != fields_.end(); ++fieldSetsIt)
        delete fieldSetsIt->second;
}

int INIReader::ParseError()
{
    return error_;
}

string INIReader::Get(string section, string name, string default_value)
{
    string key = MakeKey(section, name);
    return values_.count(key) ? values_[key] : default_value;
}

long INIReader::GetInteger(string section, string name, long default_value)
{
    string valstr = Get(section, name, "");
    const char* value = valstr.c_str();
    char* end;
    // This parses "1234" (decimal) and also "0x4D2" (hex)
    long n = strtol(value, &end, 0);
    return end > value ? n : default_value;
}

double INIReader::GetReal(string section, string name, double default_value)
{
    string valstr = Get(section, name, "");
    const char* value = valstr.c_str();
    char* end;
    double n = strtod(value, &end);
    return end > value ? n : default_value;
}

bool INIReader::GetBoolean(string section, string name, bool default_value)
{
    string valstr = Get(section, name, "");
    // Convert to lower case to make string comparisons case-insensitive
    std::transform(valstr.begin(), valstr.end(), valstr.begin(), ::tolower);
    if (valstr == "true" || valstr == "yes" || valstr == "on" || valstr == "1")
        return true;
    else if (valstr == "false" || valstr == "no" || valstr == "off" || valstr == "0")
        return false;
    else
        return default_value;
}

std::set<std::string> INIReader::GetSections() const
{
    return sections_;
}

std::set<std::string> INIReader::GetFields(std::string section) const
{
    string sectionKey = section;
    std::transform(sectionKey.begin(), sectionKey.end(), sectionKey.begin(), ::tolower);
    // std::map<std::string, std::set<std::string>*>::const_iterator fieldSetIt = fields_.find(sectionKey);
    auto fieldSetIt = fields_.find(sectionKey);
    if(fieldSetIt==fields_.end())
        return std::set<std::string>();
    return *(fieldSetIt->second);
}

string INIReader::MakeKey(string section, string name)
{
    string key = section + "=" + name;
    // Convert to lower case to make section/name lookups case-insensitive
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);
    return key;
}

int INIReader::ValueHandler(void* user, const char* section, const char* name, const char* value)
{
    INIReader* reader = (INIReader*)user;

    // Add the value to the lookup map
    string key = MakeKey(section, name);
    if (reader->values_[key].size() > 0)
        reader->values_[key] += "\n";
    reader->values_[key] += value;

    // Insert the section in the sections set
    reader->sections_.insert(section);

    // Add the value to the values set
    string sectionKey = section;
    std::transform(sectionKey.begin(), sectionKey.end(), sectionKey.begin(), ::tolower);

    std::set<std::string>* fieldsSet;
    std::map<std::string, std::set<std::string>*>::iterator fieldSetIt = reader->fields_.find(sectionKey);
    if(fieldSetIt==reader->fields_.end())
    {
        fieldsSet = new std::set<std::string>();
        reader->fields_.insert ( std::pair<std::string, std::set<std::string>*>(sectionKey,fieldsSet) );
    } else {
        fieldsSet=fieldSetIt->second;
    }
    fieldsSet->insert(name);

    return 1;
}
}  // namespace common
}  // namespace ace
