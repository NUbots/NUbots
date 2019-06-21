#ifndef MESSAGE_CONVERSION_MATRIX_TYPES_H
#define MESSAGE_CONVERSION_MATRIX_TYPES_H

#include <eigen3/Eigen/Core>

namespace message {
namespace conversion {
    namespace math {

        using vec    = Eigen::Matrix<double, Eigen::Dynamic, 1>;
        using fvec   = Eigen::Matrix<float, Eigen::Dynamic, 1>;
        using ivec   = Eigen::Matrix<int, Eigen::Dynamic, 1>;
        using uvec   = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;
        using cvec   = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>;
        using vec2   = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
        using fvec2  = Eigen::Matrix<float, 2, 1, Eigen::DontAlign>;
        using ivec2  = Eigen::Matrix<int, 2, 1, Eigen::DontAlign>;
        using uvec2  = Eigen::Matrix<unsigned int, 2, 1, Eigen::DontAlign>;
        using vec3   = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
        using fvec3  = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
        using ivec3  = Eigen::Matrix<int, 3, 1, Eigen::DontAlign>;
        using uvec3  = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>;
        using vec4   = Eigen::Matrix<double, 4, 1, Eigen::DontAlign>;
        using fvec4  = Eigen::Matrix<float, 4, 1, Eigen::DontAlign>;
        using ivec4  = Eigen::Matrix<int, 4, 1, Eigen::DontAlign>;
        using uvec4  = Eigen::Matrix<unsigned int, 4, 1, Eigen::DontAlign>;
        using vec5   = Eigen::Matrix<double, 5, 1, Eigen::DontAlign>;
        using fvec5  = Eigen::Matrix<float, 5, 1, Eigen::DontAlign>;
        using ivec5  = Eigen::Matrix<int, 5, 1, Eigen::DontAlign>;
        using uvec5  = Eigen::Matrix<unsigned int, 5, 1, Eigen::DontAlign>;
        using vec6   = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
        using fvec6  = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;
        using ivec6  = Eigen::Matrix<int, 6, 1, Eigen::DontAlign>;
        using uvec6  = Eigen::Matrix<unsigned int, 6, 1, Eigen::DontAlign>;
        using vec7   = Eigen::Matrix<double, 7, 1, Eigen::DontAlign>;
        using fvec7  = Eigen::Matrix<float, 7, 1, Eigen::DontAlign>;
        using ivec7  = Eigen::Matrix<int, 7, 1, Eigen::DontAlign>;
        using uvec7  = Eigen::Matrix<unsigned int, 7, 1, Eigen::DontAlign>;
        using vec8   = Eigen::Matrix<double, 8, 1, Eigen::DontAlign>;
        using fvec8  = Eigen::Matrix<float, 8, 1, Eigen::DontAlign>;
        using ivec8  = Eigen::Matrix<int, 8, 1, Eigen::DontAlign>;
        using uvec8  = Eigen::Matrix<unsigned int, 8, 1, Eigen::DontAlign>;
        using vec9   = Eigen::Matrix<double, 9, 1, Eigen::DontAlign>;
        using fvec9  = Eigen::Matrix<float, 9, 1, Eigen::DontAlign>;
        using ivec9  = Eigen::Matrix<int, 9, 1, Eigen::DontAlign>;
        using uvec9  = Eigen::Matrix<unsigned int, 9, 1, Eigen::DontAlign>;
        using vec10  = Eigen::Matrix<double, 10, 1, Eigen::DontAlign>;
        using fvec10 = Eigen::Matrix<float, 10, 1, Eigen::DontAlign>;
        using ivec10 = Eigen::Matrix<int, 10, 1, Eigen::DontAlign>;
        using uvec10 = Eigen::Matrix<unsigned int, 10, 1, Eigen::DontAlign>;
        using vec11  = Eigen::Matrix<double, 11, 1, Eigen::DontAlign>;
        using fvec11 = Eigen::Matrix<float, 11, 1, Eigen::DontAlign>;
        using ivec11 = Eigen::Matrix<int, 11, 1, Eigen::DontAlign>;
        using uvec11 = Eigen::Matrix<unsigned int, 11, 1, Eigen::DontAlign>;
        using vec12  = Eigen::Matrix<double, 12, 1, Eigen::DontAlign>;
        using fvec12 = Eigen::Matrix<float, 12, 1, Eigen::DontAlign>;
        using ivec12 = Eigen::Matrix<int, 12, 1, Eigen::DontAlign>;
        using uvec12 = Eigen::Matrix<unsigned int, 12, 1, Eigen::DontAlign>;
        using vec13  = Eigen::Matrix<double, 13, 1, Eigen::DontAlign>;
        using fvec13 = Eigen::Matrix<float, 13, 1, Eigen::DontAlign>;
        using ivec13 = Eigen::Matrix<int, 13, 1, Eigen::DontAlign>;
        using uvec13 = Eigen::Matrix<unsigned int, 13, 1, Eigen::DontAlign>;
        using vec14  = Eigen::Matrix<double, 14, 1, Eigen::DontAlign>;
        using fvec14 = Eigen::Matrix<float, 14, 1, Eigen::DontAlign>;
        using ivec14 = Eigen::Matrix<int, 14, 1, Eigen::DontAlign>;
        using uvec14 = Eigen::Matrix<unsigned int, 14, 1, Eigen::DontAlign>;
        using vec15  = Eigen::Matrix<double, 15, 1, Eigen::DontAlign>;
        using fvec15 = Eigen::Matrix<float, 15, 1, Eigen::DontAlign>;
        using ivec15 = Eigen::Matrix<int, 15, 1, Eigen::DontAlign>;
        using uvec15 = Eigen::Matrix<unsigned int, 15, 1, Eigen::DontAlign>;
        using vec16  = Eigen::Matrix<double, 16, 1, Eigen::DontAlign>;
        using fvec16 = Eigen::Matrix<float, 16, 1, Eigen::DontAlign>;
        using ivec16 = Eigen::Matrix<int, 16, 1, Eigen::DontAlign>;
        using uvec16 = Eigen::Matrix<unsigned int, 16, 1, Eigen::DontAlign>;

        using mat    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
        using fmat   = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
        using imat   = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
        using umat   = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;
        using cmat   = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using mat2   = Eigen::Matrix<double, 2, 2, Eigen::DontAlign>;
        using fmat2  = Eigen::Matrix<float, 2, 2, Eigen::DontAlign>;
        using imat2  = Eigen::Matrix<int, 2, 2, Eigen::DontAlign>;
        using umat2  = Eigen::Matrix<unsigned int, 2, 2, Eigen::DontAlign>;
        using mat3   = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
        using fmat3  = Eigen::Matrix<float, 3, 3, Eigen::DontAlign>;
        using imat3  = Eigen::Matrix<int, 3, 3, Eigen::DontAlign>;
        using umat3  = Eigen::Matrix<unsigned int, 3, 3, Eigen::DontAlign>;
        using mat4   = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;
        using fmat4  = Eigen::Matrix<float, 4, 4, Eigen::DontAlign>;
        using imat4  = Eigen::Matrix<int, 4, 4, Eigen::DontAlign>;
        using umat4  = Eigen::Matrix<unsigned int, 4, 4, Eigen::DontAlign>;
        using mat5   = Eigen::Matrix<double, 5, 5, Eigen::DontAlign>;
        using fmat5  = Eigen::Matrix<float, 5, 5, Eigen::DontAlign>;
        using imat5  = Eigen::Matrix<int, 5, 5, Eigen::DontAlign>;
        using umat5  = Eigen::Matrix<unsigned int, 5, 5, Eigen::DontAlign>;
        using mat6   = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>;
        using fmat6  = Eigen::Matrix<float, 6, 6, Eigen::DontAlign>;
        using imat6  = Eigen::Matrix<int, 6, 6, Eigen::DontAlign>;
        using umat6  = Eigen::Matrix<unsigned int, 6, 6, Eigen::DontAlign>;
        using mat7   = Eigen::Matrix<double, 7, 7, Eigen::DontAlign>;
        using fmat7  = Eigen::Matrix<float, 7, 7, Eigen::DontAlign>;
        using imat7  = Eigen::Matrix<int, 7, 7, Eigen::DontAlign>;
        using umat7  = Eigen::Matrix<unsigned int, 7, 7, Eigen::DontAlign>;
        using mat8   = Eigen::Matrix<double, 8, 8, Eigen::DontAlign>;
        using fmat8  = Eigen::Matrix<float, 8, 8, Eigen::DontAlign>;
        using imat8  = Eigen::Matrix<int, 8, 8, Eigen::DontAlign>;
        using umat8  = Eigen::Matrix<unsigned int, 8, 8, Eigen::DontAlign>;
        using mat9   = Eigen::Matrix<double, 9, 9, Eigen::DontAlign>;
        using fmat9  = Eigen::Matrix<float, 9, 9, Eigen::DontAlign>;
        using imat9  = Eigen::Matrix<int, 9, 9, Eigen::DontAlign>;
        using umat9  = Eigen::Matrix<unsigned int, 9, 9, Eigen::DontAlign>;
        using mat10  = Eigen::Matrix<double, 10, 10, Eigen::DontAlign>;
        using fmat10 = Eigen::Matrix<float, 10, 10, Eigen::DontAlign>;
        using imat10 = Eigen::Matrix<int, 10, 10, Eigen::DontAlign>;
        using umat10 = Eigen::Matrix<unsigned int, 10, 10, Eigen::DontAlign>;
        using mat11  = Eigen::Matrix<double, 11, 11, Eigen::DontAlign>;
        using fmat11 = Eigen::Matrix<float, 11, 11, Eigen::DontAlign>;
        using imat11 = Eigen::Matrix<int, 11, 11, Eigen::DontAlign>;
        using umat11 = Eigen::Matrix<unsigned int, 11, 11, Eigen::DontAlign>;
        using mat12  = Eigen::Matrix<double, 12, 12, Eigen::DontAlign>;
        using fmat12 = Eigen::Matrix<float, 12, 12, Eigen::DontAlign>;
        using imat12 = Eigen::Matrix<int, 12, 12, Eigen::DontAlign>;
        using umat12 = Eigen::Matrix<unsigned int, 12, 12, Eigen::DontAlign>;
        using mat13  = Eigen::Matrix<double, 13, 13, Eigen::DontAlign>;
        using fmat13 = Eigen::Matrix<float, 13, 13, Eigen::DontAlign>;
        using imat13 = Eigen::Matrix<int, 13, 13, Eigen::DontAlign>;
        using umat13 = Eigen::Matrix<unsigned int, 13, 13, Eigen::DontAlign>;
        using mat14  = Eigen::Matrix<double, 14, 14, Eigen::DontAlign>;
        using fmat14 = Eigen::Matrix<float, 14, 14, Eigen::DontAlign>;
        using imat14 = Eigen::Matrix<int, 14, 14, Eigen::DontAlign>;
        using umat14 = Eigen::Matrix<unsigned int, 14, 14, Eigen::DontAlign>;
        using mat15  = Eigen::Matrix<double, 15, 15, Eigen::DontAlign>;
        using fmat15 = Eigen::Matrix<float, 15, 15, Eigen::DontAlign>;
        using imat15 = Eigen::Matrix<int, 15, 15, Eigen::DontAlign>;
        using umat15 = Eigen::Matrix<unsigned int, 15, 15, Eigen::DontAlign>;
        using mat16  = Eigen::Matrix<double, 16, 16, Eigen::DontAlign>;
        using fmat16 = Eigen::Matrix<float, 16, 16, Eigen::DontAlign>;
        using imat16 = Eigen::Matrix<int, 16, 16, Eigen::DontAlign>;
        using umat16 = Eigen::Matrix<unsigned int, 16, 16, Eigen::DontAlign>;

    }  // namespace math
}  // namespace conversion
}  // namespace message

#endif  // MESSAGE_CONVERSION_MATRIX_TYPES_H
