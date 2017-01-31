#ifndef MESSAGE_CONVERSION_MATRIX_TYPES_H
#define MESSAGE_CONVERSION_MATRIX_TYPES_H

#include <eigen3/Eigen/Core>

namespace message {
    namespace conversion {
        namespace math {

            using vec    = Eigen::VectorXd;
            using fvec   = Eigen::VectorXf;
            using ivec   = Eigen::VectorXi;
            using uvec   = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;
            using cvec   = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>;
            using vec2   = Eigen::Vector2d;
            using fvec2  = Eigen::Vector2f;
            using ivec2  = Eigen::Vector2i;
            using uvec2  = Eigen::Matrix<unsigned int, 2, 1>;
            using vec3   = Eigen::Vector3d;
            using fvec3  = Eigen::Vector3f;
            using ivec3  = Eigen::Vector3i;
            using uvec3  = Eigen::Matrix<unsigned int, 3, 1>;
            using vec4   = Eigen::Vector4d;
            using fvec4  = Eigen::Vector4f;
            using ivec4  = Eigen::Vector4i;
            using uvec4  = Eigen::Matrix<unsigned int, 4, 1>;

            using mat    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
            using fmat   = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
            using imat   = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
            using umat   = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;
            using cmat   = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;
            using mat22  = Eigen::Matrix<double, 2, 2>;
            using fmat22 = Eigen::Matrix<float, 2, 2>;
            using imat22 = Eigen::Matrix<int, 2, 2>;
            using umat22 = Eigen::Matrix<unsigned int, 2, 2>;
            using mat33  = Eigen::Matrix<double, 3, 3>;
            using fmat33 = Eigen::Matrix<float, 3, 3>;
            using imat33 = Eigen::Matrix<int, 3, 3>;
            using umat33 = Eigen::Matrix<unsigned int, 3, 3>;
            using mat44  = Eigen::Matrix<double, 4, 4>;
            using fmat44 = Eigen::Matrix<float, 4, 4>;
            using imat44 = Eigen::Matrix<int, 4, 4>;
            using umat44 = Eigen::Matrix<unsigned int, 4, 4>;

            using Transform2D  = Eigen::Transform<double, 3, Eigen::Affine>;
            using Transform2Df = Eigen::Transform<float, 3, Eigen::Affine>;
            using Rotation2D   = Eigen::Rotation2D<double>;
            using Rotation2Df  = Eigen::Rotation2D<float>;

            using Transform3D  = Eigen::Transform<double, 3, Eigen::Affine>;
            using Transform3Df = Eigen::Transform<float, 3, Eigen::Affine>;
            using Rotation3D   = Eigen::Rotation2D<double>;
            using Rotation3Df  = Eigen::Rotation2D<float>;

            using Quaternion   = Eigen::Quaternion<double>;
            using Quaternionf  = Eigen::Quaternion<float>;

        }
    }
}

#endif // MESSAGE_CONVERSION_MATRIX_TYPES_H
