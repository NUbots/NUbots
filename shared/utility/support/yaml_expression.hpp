/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#ifndef UTILITY_SUPPORT_yaml_expression_HPP
#define UTILITY_SUPPORT_yaml_expression_HPP

#include <Eigen/Core>
#include <fmt/format.h>
#include <limits>
#include <system_error>
#include <yaml-cpp/yaml.h>

#include "math_string.hpp"

namespace utility::support {
    /**
     * Represents a mathematical expression
     * This could either be represented as a double or as a vector/matrix.
     */
    struct Expression {

        Expression() = default;
        Expression(const YAML::Node& node) : node(node) {}

        operator double() {
            double value = 0.0;

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

    template <typename T, std::size_t N, typename U = T>
    inline std::array<T, N> resolve_expression(const YAML::Node& config) {
        std::array<T, N> result;
        size_t i = 0;
        for (auto& data : config.as<std::vector<Expression>>()) {
            result[i++] = U(data);
        }
        assert((fmt::format("We expected {} elements in the YAML file but {} were found", N, i), i == N));
        return result;
    }

    template <typename T, typename U = T>
    inline std::vector<T> resolve_expression(const YAML::Node& config) {
        std::vector<T> result;
        for (auto& data : config.as<std::vector<Expression>>()) {
            result.emplace_back(U(data));
        }
        return result;
    }
}  // namespace utility::support

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
