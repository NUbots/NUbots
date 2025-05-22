/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef UTILITY_SUPPORT_SI_UNIT_HPP
#define UTILITY_SUPPORT_SI_UNIT_HPP

#include <string>
#include <utility>

namespace utility::support {

    /**
     * @brief Converts the provided value into an SI representation, returning the SI prefix and a value reduced to the
     * SI unit where it is >=1 <=1000
     *
     * @tparam T the type of the value to convert, must be numeric
     * @param v  the value to convert into an SI prefixed representation
     *
     * @return a pair containing the reduced value, and the SI prefix for the unit
     */
    template <typename T>
    std::pair<double, std::string> inline si_unit(const T& v) {
        auto d = static_cast<double>(v);

        if (d == 0) {
            return std::make_pair(d, "");
        }

        // Work out our SI index between -8 and 8
        const auto si_index = std::max(-8L, std::min(8L, int64_t(std::log10(std::abs(d)) / 3)));

        switch (si_index) {
            case 8: return std::make_pair(d * 1e-24, "Y");  // yotta
            case 7: return std::make_pair(d * 1e-21, "Z");  // zetta
            case 6: return std::make_pair(d * 1e-18, "E");  // exa
            case 5: return std::make_pair(d * 1e-15, "P");  // peta
            case 4: return std::make_pair(d * 1e-12, "T");  // tera
            case 3: return std::make_pair(d * 1e-9, "G");   // giga
            case 2: return std::make_pair(d * 1e-6, "M");   // mega
            case 1: return std::make_pair(d * 1e-3, "k");   // kilo
            case 0: return std::make_pair(d, "");           //
            case -1: return std::make_pair(d * 1e3, "m");   // milli
            case -2: return std::make_pair(d * 1e6, "μ");   // micro
            case -3: return std::make_pair(d * 1e9, "n");   // nano
            case -4: return std::make_pair(d * 1e12, "p");  // pico
            case -5: return std::make_pair(d * 1e15, "f");  // femto
            case -6: return std::make_pair(d * 1e18, "a");  // atto
            case -7: return std::make_pair(d * 1e21, "z");  // zepto
            case -8: return std::make_pair(d * 1e24, "y");  // yocto
            default: return std::make_pair(d, "");
        }
    }

    /**
     * @brief Converts the provided value into a simplified time representation, using seconds as the base.
     *
     * @tparam T the type of the value to convert, must be numeric
     * @param v  the value to convert into an simplified time representation
     *
     * @return a pair containing the reduced value, and the corresponding unit
     */
    template <typename T>
    std::pair<double, std::string> inline si_time(const T& v) {
        auto d = static_cast<double>(v);

        int seconds = 1;
        int minutes = 60 * seconds;
        int hours   = 60 * minutes;
        int days    = 24 * hours;
        int weeks   = 7 * days;
        int months  = 4 * weeks;
        int years   = 12 * months;

        if (d < minutes) {
            auto [value, prefix] = si_unit(d);
            return std::make_pair(value, prefix + "s");
        }

        if (d < hours) {
            return std::make_pair(d / minutes, "min");
        }

        if (d < days) {
            return std::make_pair(d / hours, "hr");
        }

        if (d < weeks) {
            return std::make_pair(d / days, "d");
        }

        if (d < months) {
            return std::make_pair(d / weeks, "wk");
        }

        if (d < years) {
            return std::make_pair(d / months, "mon");
        }

        return std::make_pair(d / years, "y");
    }
}  // namespace utility::support

#endif  // UTILITY_SUPPORT_SI_UNIT_HPP
