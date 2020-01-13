#ifndef UTILITY_SUPPORT_MATH_STRING_H
#define UTILITY_SUPPORT_MATH_STRING_H

#include <string>

namespace utility {
namespace support {

    double parse_to_double(const std::string& str);

    /**
     * @brief Take a math expression as a string and convert it to a Scalar.
     *
     * @param str the string that represents the mathematical expression
     *
     * @return the double that this expression resolves to
     */
    template <typename Scalar>
    Scalar parse_math_string(const std::string& str) {
        return Scalar(parse_to_double(str));
    }

}  // namespace support
}  // namespace utility

#endif  // UTILITY_SUPPORT_MATH_STRING_H
