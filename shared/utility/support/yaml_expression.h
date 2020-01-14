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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_yaml_expression_H
#define UTILITY_SUPPORT_yaml_expression_H

#include <fmt/format.h>

#include <Eigen/Core>
#include <limits>
#include <system_error>

#include "math_string.h"

namespace utility {
namespace support {
    /**
     * Represents a mathematical expression
     * This could either be represented as a double or as a vector/matrix.
     */
    struct Expression {

        Expression() {}
        Expression(const YAML::Node& node) : node(node) {}

        operator double() {
            double value;

            try {
                value = parse_math_string<double>(node.as<std::string>());
            }

            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return value;
        }

        // Handle fixed-sized matrices.
        template <typename T, std::enable_if_t<((T::RowsAtCompileTime > 1) && (T::ColsAtCompileTime > 1))>* = nullptr>
        operator T() const {

            // value : [[a, b], [c, d]]
            const size_t rows = node.size();
            const size_t cols = node[0].size();

            // Count the columns on every row.
            for (const auto& row : node) {
                if (row.size() != cols) {
                    throw std::out_of_range(
                        fmt::format("Inconsistent number of cols in matrix (cols: {} vs {}).", row.size(), cols));
                }
            }

            // Validate row and column sizes.
            if ((rows != T::RowsAtCompileTime) || (cols != T::ColsAtCompileTime)) {
                throw std::out_of_range(
                    fmt::format("Rows and columns in YAML matrix do not align with output matrix. "
                                "(rows: {} vs {}, cols: {} vs {}).",
                                rows,
                                T::RowsAtCompileTime,
                                cols,
                                T::ColsAtCompileTime));
            }

            T matrix;

            try {
                for (size_t row = 0; row < rows; row++) {
                    for (size_t col = 0; col < cols; col++) {
                        matrix(row, col) = parse_math_string<typename T::Scalar>(node[row][col].as<std::string>());
                    }
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

        // Handle fixed-sized column vectors.
        template <
            typename T,
            std::enable_if_t<((T::RowsAtCompileTime != Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
        operator T() const {

            // Validate row size.
            if (node.size() != T::RowsAtCompileTime) {
                throw std::out_of_range(
                    fmt::format("Rows in YAML column vector do not align with output vector. "
                                "(rows: {} vs {}).",
                                node.size(),
                                T::RowsAtCompileTime));
            }

            T matrix;

            try {
                for (size_t i = 0; i < node.size(); i++) {
                    matrix(i) = parse_math_string<typename T::Scalar>(node[i].as<std::string>());
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

        // Handle fixed-sized row vectors.
        template <
            typename T,
            std::enable_if_t<((T::RowsAtCompileTime == 1) && (T::ColsAtCompileTime != Eigen::Dynamic))>* = nullptr>
        operator T() const {

            // value : [a, b, c, d]
            const size_t rows = node.size();
            const size_t cols = node[0].size();

            // Validate row size.
            if (rows != T::ColsAtCompileTime) {
                throw std::out_of_range(
                    fmt::format("Columns in YAML row vector do not align with output vector. "
                                "(cols: {} vs {}).",
                                node.size(),
                                T::ColsAtCompileTime));
            }

            // Count the columns on every row.
            for (const auto& col : node) {
                if (col.size() != cols) {
                    throw std::out_of_range(
                        fmt::format("Inconsistent number of cols in matrix (cols: {} vs {}).", col.size(), cols));
                }
            }


            T matrix;

            try {
                for (size_t row = 0; row < rows; row++) {
                    for (size_t col = 0; col < cols; col++) {
                        matrix(col, row) = parse_math_string<typename T::Scalar>(node[row][col].as<std::string>());
                    }
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

        // Handle dynamic-sized matrices.
        template <typename T,
                  std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic)
                                    && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
        operator T() const {

            // value : [[a, b], [c, d]]
            const size_t rows = node.size();
            const size_t cols = node[0].size();

            // Check to see if the input is formatted as a matrix.
            // Count the columns on every row.
            for (const auto& row : node) {
                if (row.size() != cols) {
                    throw std::out_of_range(
                        fmt::format("Inconsistent number of cols in matrix (cols: {} vs {}).", row.size(), cols));
                }
            }

            T matrix(rows, cols);

            try {
                for (size_t row = 0; row < rows; row++) {
                    for (size_t col = 0; col < cols; col++) {
                        matrix(row, col) = parse_math_string<typename T::Scalar>(node[row][col].as<std::string>());
                    }
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

        // Handle dynamic-sized column vectors.
        template <
            typename T,
            std::enable_if_t<((T::RowsAtCompileTime == Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
        operator T() const {

            // value : [[a, b], [c, d]]
            const size_t rows = node.size();
            const size_t cols = node[0].size();

            // Check to see if the input is formatted as a matrix.
            // Count the columns on every row.
            for (const auto& row : node) {
                if (row.size() != cols) {
                    throw std::out_of_range(
                        fmt::format("Inconsistent number of cols in matrix (cols: {} vs {}).", row.size(), cols));
                }
            }

            T matrix(rows, std::max(cols, size_t(1)));

            try {
                for (size_t i = 0; i < rows; i++) {
                    matrix(i) = parse_math_string<typename T::Scalar>(node[i].as<std::string>());
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

        // Handle dynamic-sized row vectors.
        template <
            typename T,
            std::enable_if_t<((T::RowsAtCompileTime == 1) && (T::ColsAtCompileTime == Eigen::Dynamic))>* = nullptr>
        operator T() const {

            // value : [[a, b], [c, d]]
            const size_t rows = node.size();
            const size_t cols = node[0].size();

            // Check to see if the input is formatted as a matrix.
            // Count the columns on every row.
            for (const auto& row : node) {
                if (row.size() != cols) {
                    throw std::out_of_range(
                        fmt::format("Inconsistent number of cols in matrix (cols: {} vs {}).", row.size(), cols));
                }
            }

            T matrix(cols, rows);

            try {
                for (size_t i = 0; i < rows; i++) {
                    matrix(i) = parse_math_string<typename T::Scalar>(node[i][0].as<std::string>());
                }
            }
            catch (const std::invalid_argument& ex) {
                throw std::invalid_argument(fmt::format("Unable to convert node to arithmetic type.\n{}", ex.what()));
            }

            return matrix;
        }

    private:
        YAML::Node node;
    };
}  // namespace support
}  // namespace utility

namespace YAML {

template <>
struct convert<utility::support::Expression> {
    static Node encode(const utility::support::Expression& rhs) {
        Node node;

        // Treat as a double
        node = rhs;

        return node;
    }

    static bool decode(const Node& node, utility::support::Expression& rhs) {
        rhs = utility::support::Expression(node);
        return true;
    }
};
}  // namespace YAML

#endif
