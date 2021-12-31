#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

// Commonly used vector or matrix declaration
template <typename TYPE, int32_t row>
using Vec = Eigen::Matrix<TYPE, row, 1>; ///< Vector with fixed size

template <typename TYPE, int32_t row, int32_t col>
using Mat = Eigen::Matrix<TYPE, row, col>; ///< Matrix with fixed size

using Vec2I = Vec<int32_t, 2>;   ///< Vec 2 int32_t
using Vec2UI = Vec<uint32_t, 2>; ///< Vec 2 uint32_t
using Vec2F = Vec<float, 2>;     ///< Vec 2 float
using Vec2D = Vec<double, 2>;    ///< Vec 2 double
using Vec3I = Vec<int32_t, 3>;   ///< Vec 3 int32_t
using Vec3F = Vec<float, 3>;     ///< Vec 3 float
using Vec3D = Vec<double, 3>;    ///< Vec 3 double
using Vec4I = Vec<int32_t, 4>;   ///< Vec 4 int32_t
using Vec4F = Vec<float, 4>;     ///< Vec 4 float
using Vec4D = Vec<double, 4>;    ///< Vec 4 double
using Vec5I = Vec<int32_t, 5>;   ///< Vec 5 int32_t
using Vec5F = Vec<float, 5>;     ///< Vec 5 float
using Vec5D = Vec<double, 5>;    ///< Vec 5 double

using Mat2x2I = Mat<int32_t, 2, 2>; ///< Mat 2x2 int32_t
using Mat2x2F = Mat<float, 2, 2>;   ///< Mat 2x2 float
using Mat2x2D = Mat<double, 2, 2>;  ///< Mat 2x2 double
using Mat3x3I = Mat<int32_t, 3, 3>; ///< Mat 3x3 int32_t
using Mat3x3F = Mat<float, 3, 3>;   ///< Mat 3x3 float
using Mat3x3D = Mat<double, 3, 3>;  ///< Mat 3x3 double
using Mat4x4I = Mat<int32_t, 4, 4>; ///< Mat 4x4 int32_t
using Mat4x4F = Mat<float, 4, 4>;   ///< Mat 4x4 float
using Mat4x4D = Mat<double, 4, 4>;  ///< Mat 4x4 double
using Mat5x5I = Mat<int32_t, 5, 5>; ///< Mat 5x5 int32_t
using Mat5x5F = Mat<float, 5, 5>;   ///< Mat 5x5 float
using Mat5x5D = Mat<double, 5, 5>;  ///< Mat 5x5 double