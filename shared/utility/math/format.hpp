#ifndef SHARED_UTILITY_MATH_FORMAT_HPP
#define SHARED_UTILITY_MATH_FORMAT_HPP
#include <Eigen/Core>
#include <fmt/ostream.h>

namespace fmt {

    template <typename T, int X, int Y>
    struct formatter<Eigen::Matrix<T, X, Y>> : ostream_formatter {};

    template <typename T, int X, int Y>
    struct formatter<Eigen::Transpose<Eigen::Matrix<T, X, Y>>> : ostream_formatter {};

    template <typename T, int X, int Y>
    struct formatter<Eigen::Transpose<const Eigen::Matrix<T, X, Y>>> : ostream_formatter {};

}  // namespace fmt

#endif  // SHARED_UTILITY_MATH_FORMAT_HPP
