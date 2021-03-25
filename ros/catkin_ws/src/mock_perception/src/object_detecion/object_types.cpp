// #include "object_types.h"

#include "object_detection/object_types.h"


std::string TypeConverter::m_typeNames[(size_t)ObjectTypes::TypesCount] =
    {
        "background",
        "bottle",
        "shoes",
        "wire"};

std::string TypeConverter::m_badTypeName = "unknown";