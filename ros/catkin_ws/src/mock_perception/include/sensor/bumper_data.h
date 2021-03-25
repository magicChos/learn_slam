//
// Created by dyq on 2020/12/21.
//

#ifndef ACE_SUPERBUILD_BUMPER_DATA_H
#define ACE_SUPERBUILD_BUMPER_DATA_H

#include "ace/common/time.h"

namespace ace
{
    namespace sensor
    {

        struct BumperData
        {
            unsigned char id;
            unsigned char state;
        };

    } // namespace sensor
} // namespace ace

#endif //ACE_SUPERBUILD_BUMPER_DATA_H
