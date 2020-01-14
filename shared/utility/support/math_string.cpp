#ifndef UTILITY_SUPPORT_MATH_STRING_H
#define UTILITY_SUPPORT_MATH_STRING_H

#include <fmt/format.h>

#include <limits>
#include <stdexcept>

#include "exprtk.hpp"

namespace utility {
namespace support {

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

}  // namespace support
}  // namespace utility

#endif  // UTILITY_SUPPORT_MATH_STRING_H
