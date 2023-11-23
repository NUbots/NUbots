/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILITY_MATH_COMPARISON_HPP
#define UTILITY_MATH_COMPARISON_HPP

#include <limits>
#include <type_traits>

namespace utility::math {

    /**
     * Compare two floating-point numbers for 'almost' equality.
     *
     * Source: http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
     */
    template <class T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp) {
        // the machine epsilon has to be scaled to the magnitude of the values used
        // and multiplied by the desired precision in ULPs (units in the last place)
        return std::abs(x - y) < std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
               // unless the result is subnormal
               || std::abs(x - y) < std::numeric_limits<T>::min();
    }

    /**
     * signum function.
     * Returns either -1, 0, or 1 based on the sign of the number.
     * If -Wtype-limits is triggered, then need to specialise for unsigned types.
     * http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
     */
    template <typename T>
    inline constexpr typename std::enable_if<std::is_arithmetic<T>::value, T>::type sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * @brief Round up to nearest multiple of a number
     * @details Return the integer that is the next closest multiple of the given value.
     * http://stackoverflow.com/a/3407254/4795763
     *
     * @param numToRound The number to round up.
     * @param multiple The number that numToRound should be a multiple of.
     *
     * @return ret, such that ret % multiple == 0 and such that if ret2 and ret3 are also multiples of multiple
     *  then ret2 < numToRound <= ret < ret3 (that is, there is no other multiple that is closer to the starting
     * point).
     */
    template <typename T>
    inline constexpr typename std::enable_if<std::is_integral<T>::value, T>::type roundUp(T numToRound, T multiple) {
        if (multiple == 0) {
            return numToRound;
        }

        int remainder = abs(numToRound) % multiple;

        if (remainder == 0) {
            return numToRound;
        }

        if (numToRound < 0) {
            return -(abs(numToRound) - remainder);
        }

        return numToRound + multiple - remainder;
    }

    // http://stackoverflow.com/a/3409892/4795763
    template <typename T>
    inline constexpr typename std::enable_if<std::is_floating_point<T>::value, T>::type roundUp(T number, T fixedBase) {
        if ((fixedBase != 0.0) && (number != 0.0)) {
            T sign = sgn(number);
            number *= sign;
            number /= fixedBase;

            int fixedPoint = static_cast<int>(std::ceil(number));
            number         = fixedPoint * fixedBase;
            number *= sign;
        }

        return (number);
    }

    template <typename T>
    inline constexpr typename std::enable_if<std::is_arithmetic<T>::value, T>::type clamp(T min, T val, T max) {
        return (std::min(std::max(val, min), max));
    }
}  // namespace utility::math

#endif  // UTILITY_MATH_COMPARISON_HPP
