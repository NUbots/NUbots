/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2017 NUbots <nubots@nubots.net>
 */

#include "math_string.h"

#include <limits>
#include "exprtk.hpp"

namespace utility {
namespace support {

    /**
     * @brief Take a math expression as a string and convert it to a double.
     *
     * @param str the string that represents the mathematical expression
     *
     * @return the double that this expression resolves to
     */
    double parse_math_string(const std::string& str) {

        // Add constants to the symbol table
        exprtk::symbol_table<double> table;
        table.add_constants();
        table.add_constant("auto", std::numeric_limits<double>::infinity());

        // Add table to expression
        exprtk::expression<double> expression;
        expression.register_symbol_table(table);

        // Add expression to parser and parse
        exprtk::parser<double> parser;
        parser.compile(str, expression);

        return expression.value();
    }

}  // namespace support
}  // namespace utility
