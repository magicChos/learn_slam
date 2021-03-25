/*
 * Copyright 2021/01/29
 * The Computational Geometry Authors @Avanta
 *
 * Computational Geometry Library
 */


#ifndef COMPUTATIONAL_GEOMETRY_UTILTIIES
#define COMPUTATIONAL_GEOMETRY_UTILTIIES


#include <iostream>
#include <iomanip>
#include <vector>


#include "ace/common/computation_geometry/cg.hpp"
#include "ace/common/computation_geometry/cg_matrix.hpp"

namespace cg {
   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const point2d<Type>& point)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << point.x << "," << point.y << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const point3d<Type>& point)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << point.x << "," << point.y << "," << point.z << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const vector2d<Type>& v)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << v.x << "," << v.y << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const vector3d<Type>& v)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << v.x << "," << v.y << "," << v.z << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const ray<Type,2>& ray)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << ray.origin.x << "," << ray.origin.y << "," << ray.direction.x << "," << ray.direction.y << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const ray<Type,3>& ray)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << ray.origin.x    << "," << ray.origin.y    << "," << ray.origin.z    << ","
                << ray.direction.x << "," << ray.direction.y << "," << ray.direction.z << ")";

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const pointnd<Type,Dimension>& point)
   {
      os << "(";

      for (std::size_t i = 0; i < Dimension - 1; ++i)
      {
         os << std::scientific
            << std::showpoint
            << std::setprecision(6)
            << point[i] << ",";
      }

      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << point[Dimension - 1] << ")";

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const segment<Type,Dimension>& segment)
   {
      for (unsigned int i = 0; i < cg::segment<Type,Dimension>::PointCount; ++i)
      {
         os << segment[i];
      }

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const line<Type,Dimension>& line)
   {
      for (unsigned int i = 0; i < cg::line<Type,Dimension>::PointCount; ++i)
      {
         os << line[i];
      }

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const triangle<Type,Dimension>& triangle)
   {
      for (unsigned int i = 0; i < cg::triangle<Type,Dimension>::PointCount; ++i)
      {
         os << triangle[i];
      }

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const rectangle<Type>& rectangle)
   {
      for (unsigned int i = 0; i < cg::rectangle<Type>::PointCount; ++i)
      {
         os << rectangle[i];
      }

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const box<Type,Dimension>& box)
   {
      for (unsigned int i = 0; i < cg::box<Type,Dimension>::PointCount; ++i)
      {
         os << box[i];
      }

      return os;
   }

   template <typename Type, std::size_t Dimension>
   inline std::ostream& operator<<(std::ostream& os, const quadix<Type,Dimension>& quadix)
   {
      for (unsigned int i = 0; i < cg::quadix<Type,Dimension>::PointCount; ++i)
      {
         os << quadix[i];
      }

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const circle<Type>& circle)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(10)
         << "(" << circle.x << "," << circle.y << "," << circle.radius << ")";

      return os;
   }

   template <typename Type>
   inline std::ostream& operator<<(std::ostream& os, const sphere<Type>& sphere)
   {
      os << std::scientific
         << std::showpoint
         << std::setprecision(6)
         << "(" << sphere.x << "," << sphere.y << "," << sphere.z << "," << sphere.radius << ")";

      return os;
   }

   template <typename Type, std::size_t M, std::size_t N>
   inline std::ostream& operator<<(std::ostream& os, const matrix<Type,M,N>& matrix)
   {
      for (std::size_t x = 0; x < M; x++)
      {
         for (std::size_t y = 0; y < N; y++)
         {
            os << matrix(x,y) << "\t";
         }

         os << std::endl;
      }

      return os;
   }

} // namespace cg

#endif // COMPUTATIONAL_GEOMETRY_UTILTIIES
