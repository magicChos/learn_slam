/*
 * Copyright 2021/01/29
 * The Computational Geometry Authors @Avanta
 *
 * Computational Geometry Library
 */

#include "ace/common/computation_geometry/cg.hpp"
#include "ace/common/computation_geometry/cg_algorithm.hpp"

namespace cg {
namespace algorithm {

template <typename T>
struct remove_duplicates
{
public:
    template <typename InputIterator, typename OutputIterator>
    remove_duplicates(InputIterator begin, InputIterator end, OutputIterator out)
    {
        std::sort(begin,end);

        T previous = (*begin);
        (*out++) = (*begin);

        for (InputIterator it = (begin + 1); it != end; ++it)
        {
            if ((*it) > previous)
            {
                (*out++) = (*it);
                previous = (*it);
            }
        }
    }
};

} // namespace algorithm
} // namespace wykobi
