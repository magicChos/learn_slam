/*
 * Copyright 2021/01/29
 * The Computational Geometry Authors @Avanta
 *
 * Computational Geometry Library
 */
#ifndef COMPUTATIONAL_GEOMETRY_MATRIX
#define COMPUTATIONAL_GEOMETRY_MATRIX

#include <vector>
#include <limits>
#include <cstdlib>
#include <cassert>
#include <algorithm>

#include "ace/common/computation_geometry/cg.hpp"
#include "ace/common/computation_geometry/cg_math.hpp"

namespace cg {

template <typename T, std::size_t M, std::size_t N>
class matrix
{
public:
    matrix() : dptr(reinterpret_cast<T*>(&data))
    {
        zero();
    }

    ~matrix(){}

    matrix(const matrix<T,M,N>& m);

    // column major
    const T& operator()(std::size_t x, std::size_t y) const
    {
        return data[y][x];
    }

    T& operator()(std::size_t x, std::size_t y)
    {
        return data[y][x];
    }

    const T& operator()(std::size_t i) const
    {
    return dptr[i];
    }

    T& operator()(std::size_t i)
    {
        return dptr[i];
    }

    const T& operator[](std::size_t i) const
    {
        return dptr[i];
    }

    T& operator[](std::size_t i)
    {
        return dptr[i];
    }

    matrix<T,M,N>& operator=(const matrix<T,M,N>& m);
    matrix<T,M,N>& operator+=(const T& value);
    matrix<T,M,N>& operator-=(const T& value);
    matrix<T,M,N>& operator*=(const T& value);
    matrix<T,M,N>& operator/=(const T& value);
    matrix<T,M,N>& operator+=(const matrix<T,M,N>& _matrix);
    matrix<T,M,N>& operator-=(const matrix<T,M,N>& _matrix);

    void zero();
    void identity();
    void swap(const unsigned int& x1,const unsigned int& y1,
        const unsigned int& x2,const unsigned int& y2);

    std::size_t size() const
    {
        return M * N;
    }

private:
    T  data[M][N];
    T* dptr;
};

template <typename T>
inline T det(const matrix<T,1,1>& matrix)
{
    return matrix(0,0);
}


template <typename T>
inline T det(const matrix<T,2,2>& m)
{
    return m[0] * m[3] - m[1] * m[2];
}

template <typename T>
inline T det(const matrix<T,3,3>& m)
{
    return (m(0,0) * (m(1,1) * m(2,2) - m(1,2) * m(2,1)) -
            m(1,0) * (m(0,1) * m(2,2) - m(0,2) * m(2,1)) +
            m(2,0) * (m(0,1) * m(1,2) - m(0,2) * m(1,1)));
}

template <typename T>
inline T det(const matrix<T,4,4>& m)
{
    T A0 = m[ 0] * m[ 5] - m[ 1] * m[ 4];
    T A1 = m[ 0] * m[ 6] - m[ 2] * m[ 4];
    T A2 = m[ 0] * m[ 7] - m[ 3] * m[ 4];
    T A3 = m[ 1] * m[ 6] - m[ 2] * m[ 5];
    T A4 = m[ 1] * m[ 7] - m[ 3] * m[ 5];
    T A5 = m[ 2] * m[ 7] - m[ 3] * m[ 6];
    T B0 = m[ 8] * m[13] - m[ 9] * m[12];
    T B1 = m[ 8] * m[14] - m[10] * m[12];
    T B2 = m[ 8] * m[15] - m[11] * m[12];
    T B3 = m[ 9] * m[14] - m[10] * m[13];
    T B4 = m[ 9] * m[15] - m[11] * m[13];
    T B5 = m[10] * m[15] - m[11] * m[14];

    return A0 * B5 - A1 * B4 + A2 * B3 + A3 * B2 - A4 * B1 + A5 * B0;
}

template <typename T>
inline void transpose(matrix<T,1,1>& matrix);

template <typename T>
inline void transpose(matrix<T,2,2>& matrix);

template <typename T>
inline void transpose(matrix<T,3,3>& matrix);

template <typename T>
inline void transpose(matrix<T,4,4>& matrix);

template <typename T>
inline void inverse(matrix<T,2,2>& out_matrix, const matrix<T,2,2>& in_matrix);

template <typename T>
inline void inverse(matrix<T,3,3>& out_matrix, const matrix<T,3,3>& in_matrix);

template <typename T>
inline void inverse(matrix<T,4,4>& out_matrix, const matrix<T,4,4>& in_matrix);

template <typename T, std::size_t N>
inline void inverse(matrix<T,N,N>& out_matrix, const matrix<T,N,N>& in_matrix);

template <typename T>
inline void eigen_values(const matrix<T,2,2>& matrix, T& eigen_value1, T& eigen_value2);

template <typename T>
inline void eigenvector(const matrix<T,2,2>& matrix, vector2d<T>& eigenvector1, vector2d<T>& eigenvector2);

} // namespace cg

#include "ace/common/computation_geometry/internal/cg_matrix.inl.h"


#endif
