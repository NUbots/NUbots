#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <cmath>

namespace filter::utilities {

    template <typename Scalar>
    constexpr inline Scalar deg_to_rad(const Scalar& angle) {
        return (angle * M_PI) / 180.0;
    }

    template <typename Scalar>
    constexpr inline Scalar asin_deg(const Scalar& angle) {
        return std::asin(deg_to_rad(angle));
    }

    template <typename Scalar>
    constexpr inline Scalar acos_deg(const Scalar& angle) {
        return std::acos(deg_to_rad(angle));
    }

    template <typename Scalar>
    constexpr inline Scalar atan_deg(const Scalar& angle) {
        return std::atan(deg_to_rad(angle));
    }

    template <typename Scalar>
    constexpr inline Scalar atan2_deg(const Scalar& x, const Scalar& y) {
        return std::atan2(deg_to_rad(x), deg_to_rad(y));
    }

}  // namespace filter::utilities

#endif  // UTILITIES_HPP