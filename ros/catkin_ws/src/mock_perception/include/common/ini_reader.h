//
// Created by dyq on 2020/12/1.
//

#ifndef ACE_COMMON_INI_READER_H
#define ACE_COMMON_INI_READER_H

#include <map>
#include <set>
#include <string>

namespace ace {
namespace common {

// Read an INI file into easy-to-access name/value pairs. (Note that I've gone
// for simplicity here rather than speed, but it should be pretty decent.)
class INIReader
{
public:
    // Construct INIReader and parse given filename. See ini.h for more info
    // about the parsing.
    INIReader(const std::string& filename);
    ~INIReader();

    // Return the result of ini_parse(), i.e., 0 on success, line number of
    // first error on parse error, or -1 on file open error.
    int ParseError();

    // Get a string value from INI file, returning default_value if not found.
    std::string Get(std::string section, std::string name, std::string default_value);

    // Get an integer (long) value from INI file, returning default_value if
    // not found or not a valid integer (decimal "1234", "-1234", or hex "0x4d2").
    long GetInteger(std::string section, std::string name, long default_value);

    // Get a real (floating point double) value from INI file, returning
    // default_value if not found or not a valid floating point value
    // according to strtod().
    double GetReal(std::string section, std::string name, double default_value);

    // Get a boolean value from INI file, returning default_value if not found or if
    // not a valid true/false value. Valid true values are "true", "yes", "on", "1",
    // and valid false values are "false", "no", "off", "0" (not case sensitive).
    bool GetBoolean(std::string section, std::string name, bool default_value);

    // Returns all the section names from the INI file, in alphabetical order, but in the
    // original casing
    std::set<std::string> GetSections() const;

    // Returns all the field names from a section in the INI file, in alphabetical order,
    // but in the original casing. Returns an empty set if the field name is unknown
    std::set<std::string> GetFields(std::string section) const;

private:
    int error_;
    std::map<std::string, std::string> values_;

    // Because we want to retain the original casing in _fields, but
    // want lookups to be case-insensitive, we need both _fields and _values
    std::set<std::string> sections_;
    std::map<std::string, std::set<std::string>*> fields_;
    static std::string MakeKey(std::string section, std::string name);
    static int ValueHandler(void* user, const char* section, const char* name, const char* value);
};
}  // namespace common
}  // namespace ace

#endif // ACE_COMMON_INI_READER_H
