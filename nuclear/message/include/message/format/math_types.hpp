#ifndef MESSAGE_FORMAT_MATRIX_TYPES_HPP
#define MESSAGE_FORMAT_MATRIX_TYPES_HPP
#include <Eigen/Core>
#include <fmt/ostream.h>

namespace message::format::math {

    template <> struct fmt::formatter<Eigen::Matrix<float, 3, 1>> : ostream_formatter {};

}  // namespace message::conversion::math


#endif // MESSAGE_FORMAT_MATRIX_TYPES_HPP
