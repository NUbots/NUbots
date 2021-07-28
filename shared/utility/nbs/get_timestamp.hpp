#ifndef UTILITY_NBS_GET_TIMESTAMP_HPP
#define UTILITY_NBS_GET_TIMESTAMP_HPP

#include <nuclear>
#include <type_traits>

#include "utility/type_traits/has_timestamp.hpp"

namespace utility::nbs {

    /// @brief Returns the timestamp field of data or, if timestamp does not exist, it returns original, both converted
    /// to uint64_t
    template <typename T>
    std::enable_if_t<!utility::type_traits::has_timestamp<T>::value, uint64_t> get_timestamp(
        const NUClear::clock::time_point& original,
        const T& /*data*/) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(original.time_since_epoch()).count();
    }

    template <typename T>
    std::enable_if_t<utility::type_traits::has_timestamp<T>::value, uint64_t> get_timestamp(
        const NUClear::clock::time_point& /*original*/,
        const T& data) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(data.timestamp.time_since_epoch()).count();
    }

}  // namespace utility::nbs

#endif  // UTILITY_NBS_GET_TIMESTAMP_HPP
