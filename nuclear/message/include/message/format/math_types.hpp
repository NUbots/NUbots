#ifndef MESSAGE_FORMAT_MATRIX_TYPES_HPP
#define MESSAGE_FORMAT_MATRIX_TYPES_HPP
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

#endif  // MESSAGE_FORMAT_MATRIX_TYPES_HPP
