/*
* MIT License
*
* Copyright (c) 2025 NUbots
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

#ifndef UTILITY_SLAM_CAMERA_POSE_HPP
#define UTILITY_SLAM_CAMERA_POSE_HPP

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace utility::slam::camera {

    /**
     * @brief Helper class for working with elements of \f$\mathsf{SE}(3)\f$
     *
     * Represents a pose as a transformation matrix \f$\mathbf{T} \in \mathsf{SE}(3)\f$:
     * @f[
     * \mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{r} \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix} \in
     * \mathsf{SE}(3)
     * @f]
     * where \f$\mathbf{R} \in \mathsf{SO}(3)\f$ and \f$\mathbf{r} \in \mathbb{R}^3\f$
     *
     * @tparam Scalar The scalar type (default: double)
     */
    template <typename Scalar = double>
    struct Pose {
        using Matrix3 = Eigen::Matrix3<Scalar>;
        using Vector3 = Eigen::Vector3<Scalar>;

        Matrix3 rotationMatrix;      ///< \f$\mathbf{R} \in \mathsf{SO}(3)\f$
        Vector3 translationVector;   ///< \f$\mathbf{r} \in \mathbb{R}^3\f$

        /**
         * @brief Default constructor (\f$\mathbf{R} = \mathbf{I}\f$, \f$\mathbf{r} = \mathbf{0}\f$)
         */
        Pose() : rotationMatrix(Matrix3::Identity()), translationVector(Vector3::Zero()) {}

        /**
         * @brief Constructor from Eigen rotation matrix and translation vector
         * @param R Rotation matrix
         * @param t Translation vector
         */
        Pose(const Matrix3& R, const Vector3& t) : rotationMatrix(R), translationVector(t) {}

        /**
         * @brief Constructor from OpenCV rotation matrix and translation vector
         * @param R OpenCV rotation matrix
         * @param t OpenCV translation vector
         */
        Pose(const cv::Matx33d& R, const cv::Vec3d& t) {
            cv::cv2eigen(R, rotationMatrix);
            cv::cv2eigen(t, translationVector);
        }

        /**
         * @brief Constructor from OpenCV rotation vector and translation vector
         * @param rvec OpenCV rotation vector (exponential coordinates)
         * @param tvec OpenCV translation vector
         */
        Pose(const cv::Mat& rvec, const cv::Mat& tvec) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::cv2eigen(R, rotationMatrix);
            cv::cv2eigen(tvec, translationVector);
        }

        /**
         * @brief Copy constructor with type conversion
         *
         * This constructor allows creating a Pose object from another Pose object
         * with a different scalar type. It performs a type conversion using Eigen's
         * cast() method.
         *
         * @tparam OtherScalar The scalar type of the input Pose
         * @param T The input Pose object to copy and convert
         */
        template <typename OtherScalar>
        Pose(const Pose<OtherScalar>& T)
            : rotationMatrix(T.rotationMatrix.template cast<Scalar>()),
            translationVector(T.translationVector.template cast<Scalar>()) {}

        /**
         * @brief Group operation of \f$\mathsf{SE}(3)\f$
         *
         * Computes the composition of two poses: \f$\mathbf{T}^a_c = \mathbf{T}^a_b \mathbf{T}^b_c\f$
         * @f[
         * \begin{bmatrix} \mathbf{R}^a_c & \mathbf{r}_{C/A}^a \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b & \mathbf{r}_{B/A}^a \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}
         * \begin{bmatrix} \mathbf{R}^b_c & \mathbf{r}_{C/B}^b \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b \mathbf{R}^b_c & \mathbf{R}^a_b \mathbf{r}_{C/B}^b + \mathbf{r}_{B/A}^a \\
         * \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}
         * @f]
         *
         * @param other The other pose to compose with
         * @return The resulting composed pose
         */
        Pose operator*(const Pose& other) const {
            Pose result;
            result.rotationMatrix     = rotationMatrix * other.rotationMatrix;
            result.translationVector  = rotationMatrix * other.translationVector + translationVector;
            return result;
        }

        /**
         * @brief Action of \f$\mathsf{SE}(3)\f$ on \f$\mathbb{P}^3\f$
         *
         * Transforms a point from one coordinate frame to another.
         * Point alias: \f$\mathbf{p}_{P/A}^a = \mathbf{T}^a_b \mathbf{p}_{P/B}^b\f$
         * @f[
         * \begin{bmatrix} \mathbf{r}_{P/A}^a \\ 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b & \mathbf{r}_{B/A}^a \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}
         * \begin{bmatrix} \mathbf{r}_{P/B}^b \\ 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b \mathbf{r}_{P/B}^b + \mathbf{r}_{B/A}^a \\ 1 \end{bmatrix}
         * @f]
         *
         * Point alibi: \f$\mathbf{p}_{B/P}^a = \mathbf{T}^a_b \mathbf{p}_{A/P}^b\f$
         * @f[
         * \begin{bmatrix} \mathbf{r}_{B/P}^a \\ 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b & \mathbf{r}_{B/A}^a \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}
         * \begin{bmatrix} \mathbf{r}_{A/P}^b \\ 1 \end{bmatrix} =
         * \begin{bmatrix} \mathbf{R}^a_b \mathbf{r}_{A/P}^b + \mathbf{r}_{B/A}^a \\ 1 \end{bmatrix}
         * @f]
         *
         * @param r The point to transform
         * @return The transformed point
         * @see Pose::operator*(const cv::Vec3d &) const
         */
        Vector3 operator*(const Vector3& r) const {
            return rotationMatrix * r + translationVector;
        }

        /**
         * @brief Action of \f$\mathsf{SE}(3)\f$ on \f$\mathbb{P}^3\f$ (OpenCV version)
         * @param r The point to transform (OpenCV vector)
         * @return The transformed point (OpenCV vector)
         * @see Pose::operator*(const Vector3 &) const
         */
        cv::Vec3d operator*(const cv::Vec3d& r) const {
            Vector3 result = rotationMatrix * Eigen::Map<const Vector3>(r.val) + translationVector;
            return cv::Vec3d(result[0], result[1], result[2]);
        }

        /**
         * @brief Inverse element in \f$\mathsf{SE}(3)\f$
         *
         * Computes the inverse of the pose: \f$(\mathbf{T}^a_b)^{-1} = \mathbf{T}^b_a\f$
         * @f[
         * \begin{bmatrix} \mathbf{R}^a_b & \mathbf{r}_{B/A}^a \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}^{-1} =
         * \begin{bmatrix} \mathbf{R}^b_a & \mathbf{r}_{A/B}^b \\ \mathbf{0}^\mathsf{T} & 1 \end{bmatrix} =
         * \begin{bmatrix} (\mathbf{R}^a_b)^\mathsf{T} & -(\mathbf{R}^a_b)^\mathsf{T} \mathbf{r}_{B/A}^a \\
         * \mathbf{0}^\mathsf{T} & 1 \end{bmatrix}
         * @f]
         *
         * @return The inverse pose
         */
        Pose inverse() const {
            Pose result;
            result.rotationMatrix    = rotationMatrix.transpose();
            result.translationVector = -result.rotationMatrix * translationVector;
            return result;
        }

        /**
         * @brief Conversion to OpenCV rotation matrix
         * @return The rotation matrix in OpenCV format
         * @see Pose::translationVectorCV
         */
        cv::Matx33d rotationMatrixCV() const {
            cv::Matx33d R;
            cv::eigen2cv(rotationMatrix, R);
            return R;
        }

        /**
         * @brief Conversion to OpenCV translation vector
         * @return The translation vector in OpenCV format
         * @see Pose::rotationMatrixCV
         */
        cv::Vec3d translationVectorCV() const {
            cv::Vec3d t;
            cv::eigen2cv(translationVector, t);
            return t;
        }
    };

}  // namespace utility::slam::camera

#endif  // UTILITY_SLAM_CAMERA_POSE_HPP
