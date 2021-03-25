/*
 * Copyright 2021/01/29
 * The Computational Geometry Authors @Avanta
 *
 * Computational Geometry Library
 */


#ifndef COMPUTATIONAL_GEOMETRY_ALGORITHM
#define COMPUTATIONAL_GEOMETRY_ALGORITHM


#include <algorithm>

#include "ace/common/computation_geometry/cg.hpp"

namespace cg {
namespace algorithm {

    template <typename T> struct isotropic_normalization;
    template <typename T> struct isotropic_normalization< point2d<T> >;
    template <typename T> struct isotropic_normalization< point3d<T> >;

    template <typename T> struct convex_hull_graham_scan;
    template <typename T> struct convex_hull_graham_scan< point2d<T> >;

    template <typename T> struct convex_hull_jarvis_march;
    template <typename T> struct convex_hull_jarvis_march< point2d<T> >;

    template <typename T> struct convex_hull_melkman;
    template <typename T> struct convex_hull_melkman< point2d<T> >;

    template <typename T> struct covariance_matrix;
    template <typename T> struct covariance_matrix< point2d<T> >;
    template <typename T> struct covariance_matrix< point3d<T> >;

    template <typename T> struct ordered_polygon;
    template <typename T> struct ordered_polygon< point2d<T> >;

    template <typename T> struct remove_duplicates;

    template <typename T> struct naive_group_intersections;
    template <typename T> struct naive_group_intersections< segment<T,2> >;
    template <typename T> struct naive_group_intersections< segment<T,3> >;
    template <typename T> struct naive_group_intersections< circle<T> >;

    template <typename T> struct naive_minimum_bounding_ball;
    template <typename T> struct naive_minimum_bounding_ball< point2d<T> >;

    template <typename T> struct naive_minimum_bounding_ball_with_ch_filter;
    template <typename T> struct naive_minimum_bounding_ball_with_ch_filter< point2d<T> >;

    template <typename T> struct randomized_minimum_bounding_ball;
    template <typename T> struct randomized_minimum_bounding_ball< point2d<T> >;

    template <typename T> struct randomized_minimum_bounding_ball_with_ch_filter;
    template <typename T> struct randomized_minimum_bounding_ball_with_ch_filter < point2d<T> >;

    template <typename T> struct ritter_minimum_bounding_ball;
    template <typename T> struct ritter_minimum_bounding_ball< point2d<T> >;
    template <typename T> struct ritter_minimum_bounding_ball< point3d<T> >;
    //template <typename T, std::size_t D> struct ritter_minimum_bounding_ball< pointnd<T,D> >;

    template <typename T> struct ritter_minimum_bounding_ball_with_ch_filter;
    template <typename T> struct ritter_minimum_bounding_ball_with_ch_filter< point2d<T> >;

    template <typename T> struct generate_axis_projection_descriptor;

    template <typename T> struct sutherland_hodgman_polygon_clipper;
    template <typename T> struct sutherland_hodgman_polygon_clipper< point2d<T> >;

    template <typename T> struct sutherland_hodgman_polygon_clipper_engine;
    template <typename T> struct sutherland_hodgman_polygon_clipper_engine< point2d<T> >;

    template <typename T> struct polygon_triangulate;
    template <typename T> struct polygon_triangulate< point2d<T> >;
} // namespace algorithm
} // namespace cg


#include "ace/common/computation_geometry/internal/cg_normalization.inl.h"
#include "ace/common/computation_geometry/internal/cg_hull.inl.h"
#include "ace/common/computation_geometry/internal/cg_ordered_polygon.inl.h"
#include "ace/common/computation_geometry/internal/cg_duplicates.inl.h"
#include "ace/common/computation_geometry/internal/cg_naive_group_intersections.inl.h"
#include "ace/common/computation_geometry/internal/cg_axis_projection_descriptor.inl.h"
#include "ace/common/computation_geometry/internal/cg_clipping.inl.h"
#include "ace/common/computation_geometry/internal/cg_earclipping.inl.h"


#endif
