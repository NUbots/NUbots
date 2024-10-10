/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MESSAGE_CONVERSION_MATRIX_TYPES_HPP
#define MESSAGE_CONVERSION_MATRIX_TYPES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename Scalar, int Dim, int Mode>
inline bool operator==(const Eigen::Transform<Scalar, Dim, Mode>& lhs, const Eigen::Transform<Scalar, Dim, Mode>& rhs) {
    return lhs.matrix() == rhs.matrix();
}

namespace message::conversion::math {

    using iso3  = Eigen::Transform<double, 3, Eigen::Isometry>;
    using fiso3 = Eigen::Transform<float, 3, Eigen::Isometry>;
    using iso2  = Eigen::Transform<double, 2, Eigen::Isometry>;
    using fiso2 = Eigen::Transform<float, 2, Eigen::Isometry>;

    using quat  = Eigen::Quaternion<double>;
    using fquat = Eigen::Quaternion<float>;

    using vec    = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    using fvec   = Eigen::Matrix<float, Eigen::Dynamic, 1>;
    using ivec   = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using uvec   = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;
    using cvec   = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>;
    using vec2   = Eigen::Matrix<double, 2, 1>;
    using fvec2  = Eigen::Matrix<float, 2, 1>;
    using ivec2  = Eigen::Matrix<int, 2, 1>;
    using uvec2  = Eigen::Matrix<unsigned int, 2, 1>;
    using vec3   = Eigen::Matrix<double, 3, 1>;
    using fvec3  = Eigen::Matrix<float, 3, 1>;
    using ivec3  = Eigen::Matrix<int, 3, 1>;
    using uvec3  = Eigen::Matrix<unsigned int, 3, 1>;
    using vec4   = Eigen::Matrix<double, 4, 1>;
    using fvec4  = Eigen::Matrix<float, 4, 1>;
    using ivec4  = Eigen::Matrix<int, 4, 1>;
    using uvec4  = Eigen::Matrix<unsigned int, 4, 1>;
    using vec5   = Eigen::Matrix<double, 5, 1>;
    using fvec5  = Eigen::Matrix<float, 5, 1>;
    using ivec5  = Eigen::Matrix<int, 5, 1>;
    using uvec5  = Eigen::Matrix<unsigned int, 5, 1>;
    using vec6   = Eigen::Matrix<double, 6, 1>;
    using fvec6  = Eigen::Matrix<float, 6, 1>;
    using ivec6  = Eigen::Matrix<int, 6, 1>;
    using uvec6  = Eigen::Matrix<unsigned int, 6, 1>;
    using vec7   = Eigen::Matrix<double, 7, 1>;
    using fvec7  = Eigen::Matrix<float, 7, 1>;
    using ivec7  = Eigen::Matrix<int, 7, 1>;
    using uvec7  = Eigen::Matrix<unsigned int, 7, 1>;
    using vec8   = Eigen::Matrix<double, 8, 1>;
    using fvec8  = Eigen::Matrix<float, 8, 1>;
    using ivec8  = Eigen::Matrix<int, 8, 1>;
    using uvec8  = Eigen::Matrix<unsigned int, 8, 1>;
    using vec9   = Eigen::Matrix<double, 9, 1>;
    using fvec9  = Eigen::Matrix<float, 9, 1>;
    using ivec9  = Eigen::Matrix<int, 9, 1>;
    using uvec9  = Eigen::Matrix<unsigned int, 9, 1>;
    using vec10  = Eigen::Matrix<double, 10, 1>;
    using fvec10 = Eigen::Matrix<float, 10, 1>;
    using ivec10 = Eigen::Matrix<int, 10, 1>;
    using uvec10 = Eigen::Matrix<unsigned int, 10, 1>;
    using vec11  = Eigen::Matrix<double, 11, 1>;
    using fvec11 = Eigen::Matrix<float, 11, 1>;
    using ivec11 = Eigen::Matrix<int, 11, 1>;
    using uvec11 = Eigen::Matrix<unsigned int, 11, 1>;
    using vec12  = Eigen::Matrix<double, 12, 1>;
    using fvec12 = Eigen::Matrix<float, 12, 1>;
    using ivec12 = Eigen::Matrix<int, 12, 1>;
    using uvec12 = Eigen::Matrix<unsigned int, 12, 1>;
    using vec13  = Eigen::Matrix<double, 13, 1>;
    using fvec13 = Eigen::Matrix<float, 13, 1>;
    using ivec13 = Eigen::Matrix<int, 13, 1>;
    using uvec13 = Eigen::Matrix<unsigned int, 13, 1>;
    using vec14  = Eigen::Matrix<double, 14, 1>;
    using fvec14 = Eigen::Matrix<float, 14, 1>;
    using ivec14 = Eigen::Matrix<int, 14, 1>;
    using uvec14 = Eigen::Matrix<unsigned int, 14, 1>;
    using vec15  = Eigen::Matrix<double, 15, 1>;
    using fvec15 = Eigen::Matrix<float, 15, 1>;
    using ivec15 = Eigen::Matrix<int, 15, 1>;
    using uvec15 = Eigen::Matrix<unsigned int, 15, 1>;
    using vec16  = Eigen::Matrix<double, 16, 1>;
    using fvec16 = Eigen::Matrix<float, 16, 1>;
    using ivec16 = Eigen::Matrix<int, 16, 1>;
    using uvec16 = Eigen::Matrix<unsigned int, 16, 1>;

    using mat    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
    using fmat   = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
    using imat   = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
    using umat   = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;
    using cmat   = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;
    using mat2   = Eigen::Matrix<double, 2, 2>;
    using fmat2  = Eigen::Matrix<float, 2, 2>;
    using imat2  = Eigen::Matrix<int, 2, 2>;
    using umat2  = Eigen::Matrix<unsigned int, 2, 2>;
    using mat3   = Eigen::Matrix<double, 3, 3>;
    using fmat3  = Eigen::Matrix<float, 3, 3>;
    using imat3  = Eigen::Matrix<int, 3, 3>;
    using umat3  = Eigen::Matrix<unsigned int, 3, 3>;
    using mat4   = Eigen::Matrix<double, 4, 4>;
    using fmat4  = Eigen::Matrix<float, 4, 4>;
    using imat4  = Eigen::Matrix<int, 4, 4>;
    using umat4  = Eigen::Matrix<unsigned int, 4, 4>;
    using mat5   = Eigen::Matrix<double, 5, 5>;
    using fmat5  = Eigen::Matrix<float, 5, 5>;
    using imat5  = Eigen::Matrix<int, 5, 5>;
    using umat5  = Eigen::Matrix<unsigned int, 5, 5>;
    using mat6   = Eigen::Matrix<double, 6, 6>;
    using fmat6  = Eigen::Matrix<float, 6, 6>;
    using imat6  = Eigen::Matrix<int, 6, 6>;
    using umat6  = Eigen::Matrix<unsigned int, 6, 6>;
    using mat7   = Eigen::Matrix<double, 7, 7>;
    using fmat7  = Eigen::Matrix<float, 7, 7>;
    using imat7  = Eigen::Matrix<int, 7, 7>;
    using umat7  = Eigen::Matrix<unsigned int, 7, 7>;
    using mat8   = Eigen::Matrix<double, 8, 8>;
    using fmat8  = Eigen::Matrix<float, 8, 8>;
    using imat8  = Eigen::Matrix<int, 8, 8>;
    using umat8  = Eigen::Matrix<unsigned int, 8, 8>;
    using mat9   = Eigen::Matrix<double, 9, 9>;
    using fmat9  = Eigen::Matrix<float, 9, 9>;
    using imat9  = Eigen::Matrix<int, 9, 9>;
    using umat9  = Eigen::Matrix<unsigned int, 9, 9>;
    using mat10  = Eigen::Matrix<double, 10, 10>;
    using fmat10 = Eigen::Matrix<float, 10, 10>;
    using imat10 = Eigen::Matrix<int, 10, 10>;
    using umat10 = Eigen::Matrix<unsigned int, 10, 10>;
    using mat11  = Eigen::Matrix<double, 11, 11>;
    using fmat11 = Eigen::Matrix<float, 11, 11>;
    using imat11 = Eigen::Matrix<int, 11, 11>;
    using umat11 = Eigen::Matrix<unsigned int, 11, 11>;
    using mat12  = Eigen::Matrix<double, 12, 12>;
    using fmat12 = Eigen::Matrix<float, 12, 12>;
    using imat12 = Eigen::Matrix<int, 12, 12>;
    using umat12 = Eigen::Matrix<unsigned int, 12, 12>;
    using mat13  = Eigen::Matrix<double, 13, 13>;
    using fmat13 = Eigen::Matrix<float, 13, 13>;
    using imat13 = Eigen::Matrix<int, 13, 13>;
    using umat13 = Eigen::Matrix<unsigned int, 13, 13>;
    using mat14  = Eigen::Matrix<double, 14, 14>;
    using fmat14 = Eigen::Matrix<float, 14, 14>;
    using imat14 = Eigen::Matrix<int, 14, 14>;
    using umat14 = Eigen::Matrix<unsigned int, 14, 14>;
    using mat15  = Eigen::Matrix<double, 15, 15>;
    using fmat15 = Eigen::Matrix<float, 15, 15>;
    using imat15 = Eigen::Matrix<int, 15, 15>;
    using umat15 = Eigen::Matrix<unsigned int, 15, 15>;
    using mat16  = Eigen::Matrix<double, 16, 16>;
    using fmat16 = Eigen::Matrix<float, 16, 16>;
    using imat16 = Eigen::Matrix<int, 16, 16>;
    using umat16 = Eigen::Matrix<unsigned int, 16, 16>;

}  // namespace message::conversion::math

#endif  // MESSAGE_CONVERSION_MATRIX_TYPES_HPP
