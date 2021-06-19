#ifndef UTILITY_NBS_GET_ID_HPP
#define UTILITY_NBS_GET_ID_HPP

#include <type_traits>

#include "utility/type_traits/has_id.hpp"

namespace utility::nbs {

    /// @brief Returns the subtype field of data or, if subtype does not exist, 0
    template <typename T>
    std::enable_if_t<!utility::type_traits::has_id<T>::value, uint32_t> get_subtype(const T& /*data*/) {
        return 0;
    }

    template <typename T>
    std::enable_if_t<utility::type_traits::has_id<T>::value, uint32_t> get_subtype(const T& data) {
        return data.id;
    }

}  // namespace utility::nbs

#endif  // UTILITY_NBS_GET_ID_HPP
