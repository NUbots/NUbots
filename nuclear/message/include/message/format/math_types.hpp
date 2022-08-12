#ifndef MESSAGE_FORMAT_MATRIX_TYPES_HPP
#define MESSAGE_FORMAT_MATRIX_TYPES_HPP
#include <Eigen/Core>
#include <fmt/ostream.h>

namespace fmt {

    template <> struct formatter<Eigen::Matrix<float, 3, 1>> : ostream_formatter {};
    template <> struct formatter<Eigen::Transpose<Eigen::Matrix<float, 3, 1>>> : ostream_formatter {};

} // namespace fmt

#endif // MESSAGE_FORMAT_MATRIX_TYPES_HPP
