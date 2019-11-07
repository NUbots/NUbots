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

#ifndef UTILITY_SUPPORT_YAML_EXPRESSION_H
#define UTILITY_SUPPORT_YAML_EXPRESSION_H

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <cassert>
#include <iostream>
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

        template <typename T, std::enable_if_t<std::is_integral<T>::value>* = nullptr>
        operator T() {
            return parse<T>(node.as<std::string>());
        }

        template <typename T, std::enable_if_t<std::is_floating_point<T>::value>* = nullptr>
        operator T() {
            return parse<T>(node.as<std::string>());
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
                    throw std::system_error(-1,
                                            std::system_category(),
                                            "Inconsistent number of cols in matrix (cols: " + std::to_string(row.size())
                                                + " vs " + std::to_string(cols) + ").");
                }
            }

            // Validate row and column sizes.
            if ((rows != T::RowsAtCompileTime) || (cols != T::ColsAtCompileTime)) {
                throw std::system_error(
                    -1,
                    std::system_category(),
                    "Rows and columns in YAML matrix do not align with output matrix. (rows: " + std::to_string(rows)
                        + " vs " + std::to_string(T::RowsAtCompileTime) + " cols: " + std::to_string(cols) + " vs "
                        + std::to_string(T::ColsAtCompileTime) + ").");
            }

            T matrix;

            for (size_t row = 0; row < rows; row++) {
                for (size_t col = 0; col < cols; col++) {
                    matrix(row, col) = parse<typename T::Scalar>(node[row][col].as<std::string>());
                }
            }

            return matrix;
        }

        // Handle fixed-sized column vectors.
        template <
            typename T,
            std::enable_if_t<((T::RowsAtCompileTime != Eigen::Dynamic) && (T::ColsAtCompileTime == 1))>* = nullptr>
        operator T() const {

            // value : [a, b, c, d]

            // Validate row size.
            if (node.size() != T::RowsAtCompileTime) {
                throw std::system_error(
                    -1,
                    std::system_category(),
                    "Rows in YAML column vector do not align with output vector. (rows: " + std::to_string(node.size())
                        + " vs " + std::to_string(T::RowsAtCompileTime) + ").");
            }

            T matrix;

            for (size_t i = 0; i < node.size(); i++) {
                matrix(i) = parse<typename T::Scalar>(node[i].as<std::string>());
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
                throw std::system_error(-1,
                                        std::system_category(),
                                        "Columns in YAML column vector do not align with output vector. (rows: "
                                            + std::to_string(node.size()) + " vs "
                                            + std::to_string(T::RowsAtCompileTime) + ").");
            }

            // Count the columns on every row.
            for (const auto& col : node) {
                if (col.size() != cols) {
                    throw std::system_error(-1,
                                            std::system_category(),
                                            "Inconsistent number of cols in matrix (cols: " + std::to_string(col.size())
                                                + " vs " + std::to_string(cols) + ").");
                }
            }


            T matrix;

            for (size_t row = 0; row < rows; row++) {
                for (size_t col = 0; col < cols; col++) {
                    matrix(col, row) = parse<typename T::Scalar>(node[row][col].as<std::string>());
                }
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
                    throw std::system_error(-1,
                                            std::system_category(),
                                            "Inconsistent number of cols in matrix (cols: " + std::to_string(row.size())
                                                + " vs " + std::to_string(cols) + ").");
                }
            }

            T matrix(rows, cols);

            for (size_t row = 0; row < rows; row++) {
                for (size_t col = 0; col < cols; col++) {
                    matrix(row, col) = parse<typename T::Scalar>(node[row][col].as<std::string>());
                }
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
                    throw std::system_error(-1,
                                            std::system_category(),
                                            "Inconsistent number of cols in matrix (cols: " + std::to_string(row.size())
                                                + " vs " + std::to_string(cols) + ").");
                }
            }

            T matrix(rows, std::max(cols, size_t(1)));

            for (size_t i = 0; i < rows; i++) {
                matrix(i) = parse<typename T::Scalar>(node[i].as<std::string>());
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
                    throw std::system_error(-1,
                                            std::system_category(),
                                            "Inconsistent number of cols in matrix (cols: " + std::to_string(row.size())
                                                + " vs " + std::to_string(cols) + ").");
                }
            }

            T matrix(cols, rows);

            for (size_t i = 0; i < rows; i++) {
                matrix(i) = parse<typename T::Scalar>(node[i][0].as<std::string>());
            }

            return matrix;
        }

    private:
        YAML::Node node;

        template <typename Scalar>
        Scalar parse(const std::string& expression) const {
            // Parse the expression using the math string parser
            return static_cast<Scalar>(utility::support::parse_math_string(expression));
        }
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

#endif  // UTILITY_SUPPORT_YAML_EXPRESSION_H
