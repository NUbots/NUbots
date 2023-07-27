#ifndef UTILITY_TYPE_TRAITS_REFLECTION_EXCEPTIONS_HPP
#define UTILITY_TYPE_TRAITS_REFLECTION_EXCEPTIONS_HPP

#include <stdexcept>
#include <string>

namespace utility::reflection {
    struct unknown_message : std::runtime_error {
        unknown_message(const uint64_t& hash)
            : std::runtime_error("Unknown message type with hash " + std::to_string(hash)){};
    };
}  // namespace utility::reflection

#endif  // UTILITY_TYPE_TRAITS_REFLECTION_EXCEPTIONS_HPP
