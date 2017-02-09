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

            using mat    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
            using fmat   = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
            using imat   = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
            using umat   = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;
            using cmat   = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
            using mat22  = Eigen::Matrix<double, 2, 2, Eigen::DontAlign>;
            using fmat22 = Eigen::Matrix<float, 2, 2, Eigen::DontAlign>;
            using imat22 = Eigen::Matrix<int, 2, 2, Eigen::DontAlign>;
            using umat22 = Eigen::Matrix<unsigned int, 2, 2, Eigen::DontAlign>;
            using mat33  = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
            using fmat33 = Eigen::Matrix<float, 3, 3, Eigen::DontAlign>;
            using imat33 = Eigen::Matrix<int, 3, 3, Eigen::DontAlign>;
            using umat33 = Eigen::Matrix<unsigned int, 3, 3, Eigen::DontAlign>;
            using mat44  = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;
            using fmat44 = Eigen::Matrix<float, 4, 4, Eigen::DontAlign>;
            using imat44 = Eigen::Matrix<int, 4, 4, Eigen::DontAlign>;
            using umat44 = Eigen::Matrix<unsigned int, 4, 4, Eigen::DontAlign>;
        }
    }
}

#endif // MESSAGE_CONVERSION_MATRIX_TYPES_H
