/*
 * Copyright 2021/01/29
 * The Computational Geometry Authors @Avanta
 *
 * Computational Geometry Library
 */

#include <algorithm>


#include "ace/common/computation_geometry/cg.hpp"
#include "ace/common/computation_geometry/cg_algorithm.hpp"

namespace cg
{
namespace algorithm
{
template <typename T>
struct polygon_triangulate < point2d<T> >
{
public:

    template <typename OutputIterator>
    polygon_triangulate(const polygon<T,2>& polygon, OutputIterator out)
    {
        if (polygon.size() < 3)
            return;
        else if (polygon.size() == 3)
        {
            (*out++) = make_triangle(polygon[0], polygon[1], polygon[2]);
            return;
        }

        cg::polygon<T,2> internal_polygon;

        internal_polygon.reserve(polygon.size());

        std::copy(polygon.begin(),polygon.end(),std::back_inserter(internal_polygon));

        if (cg::polygon_orientation(internal_polygon) != Clockwise)
        {
            internal_polygon.reverse();
        }

        while (internal_polygon.size() > 3)
        {
            for (std::size_t i = 0; i < internal_polygon.size(); ++i)
            {
                if (convex_vertex(i,internal_polygon,Clockwise) && vertex_is_ear(i,internal_polygon))
                {
                    (*out++) = vertex_triangle(i,internal_polygon);

                    internal_polygon.erase(i);

                    break;
                }
            }
        }

        (*out++) = vertex_triangle(1,internal_polygon);
    }
};

} // namespace algorithm

} // namespace cg
