/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_ANGLE_HPP
#define UTILITY_MATH_ANGLE_HPP

#include <Eigen/Core>
#include <cmath>

/**
 * @author Trent Houliston
 */
namespace utility::math::angle {

    /**
     * Takes an angle in radians and normalizes it to be between -pi and pi
     *
     * @param value the angle in radians
     *
     * @return the angle between -pi and pi
     */
    template <typename T>
    inline T normalizeAngle(const T value) {

        T angle = std::fmod(value, static_cast<T>(2 * M_PI));

        if (angle <= -M_PI) {
            angle += 2 * M_PI;
        }

        if (angle > M_PI) {
            angle -= 2 * M_PI;
        }

        return angle;
    }

    template <typename T>
    inline T acos_clamped(const T& a) {
        return std::acos(std::max(std::min(a, T(1.0)), T(-1.0)));
    }

    template <typename T>
    inline T asin_clamped(const T& a) {
        return std::asin(std::max(std::min(a, T(1.0)), T(-1.0)));
    }

    /**
     *  Finds the angle between two vectors. This method is numerically stable when the vectors are close to parallel.
     *
     * @details Based on the implementation found here https://www.plunk.org/~hatch/rightway.html
     *
     * @param u A unit vector
     * @param v A unit vector
     *
     * @tparam Derived Type derived from `Eigen::MatrixBase` (i.e. any dense matrix type)
     * @tparam Scalar  Type of each element stored in the dense matrix (must be a floating point type)
     *
     * @return The acute angle in range [0,pi] between two vectors.
     */
    template <typename Derived, typename Scalar = typename Derived::Scalar>
    typename std::enable_if_t<std::is_floating_point_v<Scalar>, Scalar> angle_between(
        const Eigen::MatrixBase<Derived>& u,
        const Eigen::MatrixBase<Derived>& v) {
        return u.dot(v) < Scalar(0) ? std::numbers::pi_v<Scalar> - Scalar(2) * std::asin((-v - u).norm() * Scalar(0.5))
                                    : Scalar(2) * std::asin((v - u).norm() * Scalar(0.5));
    }

    /**
     * Calculates the difference between two angles between -pi and pi
     * Method:
     * http://math.stackexchange.com/questions/1158223/solve-for-x-where-a-sin-x-b-cos-x-c-where-a-b-and-c-are-kno
     * @param
     */
    inline double difference(const double a, const double b) {

        return M_PI - std::fabs(std::fmod(std::fabs(a - b), (2 * M_PI)) - M_PI);
    }

    /**
     * Calculates the signed angle with the smallest absolute value such
     * that: normalizeAngle(b + signedDifference(a, b)) == normalizeAngle(a).
     * Method: http://stackoverflow.com/a/7869457
     */
    inline double signedDifference(const double a, const double b) {

        auto x = a - b;

        auto m = x - std::floor(x / (2 * M_PI)) * 2 * M_PI;

        auto d = std::fmod(m + M_PI, (2 * M_PI)) - M_PI;

        return d;
    }

    /**
     * Compute the oriented distance between the two given angle
     * in the range -PI/2:PI/2 radian from angleSrc to angleDst
     * (Better than doing angleDst-angleSrc)
     */
    template <typename Scalar>
    inline double angleDistance(const Scalar& angleSrc, const Scalar& angleDst) {
        const Scalar source = normalizeAngle(angleSrc);
        const Scalar dest   = normalizeAngle(angleDst);

        const Scalar max = source > dest ? source : dest;
        const Scalar min = source > dest ? dest : source;

        const Scalar dist1 = max - min;
        const Scalar dist2 = (2.0 * M_PI) - max + min;
        // Whichever distance is smaller is used, and we make sure that the rotation is in the correct direction
        // by negating the distance if it's clockwise
        return dist1 < dist2 ? (source > dest ? -dist1 : dist1) : (source > dest ? dist2 : -dist2);
    }

    inline double vectorToBearing(const Eigen::Vector2d& dirVec) {
        return std::atan2(dirVec.y(), dirVec.x());
    }

    /*! @brief Solves for x in $a \sin(x) + b \cos(x) = c ; x \in [0,\pi]$
     */
    inline float solveLinearTrigEquation(float a, float b, float c) {
        float norm = std::sqrt(a * a + b * b);
        if (norm == 0) {
            throw std::domain_error(
                "utility::math::angle::solveLinearTrigEquation - std::sqrt(a*a+b*b) == 0 => Any value for x is "
                "a "
                "solution");
        }

        // Normalise equation
        float a_ = a / norm;
        float b_ = b / norm;
        float c_ = c / norm;

        if (std::fabs(c_) > 1) {
            throw std::domain_error("utility::math::angle::solveLinearTrigEquation - no solution |c_|>1");
        }

        // Find alpha such that $\sin(\alpha) = a\_$ and $\cos(\alpha) = b\_$, which is possible because $a\_^2
        // + b\_^2 = 1$
        float alpha = std::atan2(a_, b_);

        // Hence the equation becomes $\cos(\alpha)\cos(x)+\sin(\alpha)\sin(x) = cos(x-\alpha) = c\_$
        return alpha + acos_clamped(c_);
    }
}  // namespace utility::math::angle
#endif
