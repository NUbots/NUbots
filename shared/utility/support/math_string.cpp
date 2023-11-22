/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef UTILITY_SUPPORT_MATH_STRING_HPP
#define UTILITY_SUPPORT_MATH_STRING_HPP

#include <fmt/format.h>
#include <limits>
#include <stdexcept>

#include "exprtk.hpp"

namespace utility::support {

    double parse_to_double(const std::string& str) {

        // Add constants to the symbol table
        exprtk::symbol_table<double> table;
        table.add_constants();
        table.add_constant("auto", std::numeric_limits<double>::infinity());

        // File size constants
        table.add_constant("KiB", 1024);        // 2^10
        table.add_constant("MiB", 1048576);     // 2^20
        table.add_constant("GiB", 1073741824);  // 2^30

        // Add table to expression
        exprtk::expression<double> expression;
        expression.register_symbol_table(table);

        // Add expression to parser and parse
        exprtk::parser<double> parser;
        if (!parser.compile(str, expression)) {
            throw std::invalid_argument(fmt::format("ExprTk failed to parse expression '{}'", str));
        }

        return expression.value();
    }

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_MATH_STRING_HPP
