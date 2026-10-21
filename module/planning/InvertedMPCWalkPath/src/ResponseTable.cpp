/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "ResponseTable.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace module::planning::inverted_walk_mpc {

    namespace {

        /// @brief A grid from its rows [a, b, outputs...], one per command: the commands along a and b, and the outputs
        /// as tables over them. Throws unless the rows are every (a, b) on the grid, once.
        void read_grid(const YAML::Node& node,
                       const std::size_t outputs,
                       const std::string& name,
                       std::vector<double>& a,
                       std::vector<double>& b,
                       std::vector<Eigen::MatrixXd>& values) {
            const auto rows = node.as<std::vector<std::vector<double>>>();
            for (const auto& row : rows) {
                if (row.size() != 2 + outputs) {
                    throw std::runtime_error("InvertedMPCWalkPath: a row of the response's " + name + " has "
                                             + std::to_string(row.size()) + " values, not "
                                             + std::to_string(2 + outputs));
                }
                a.push_back(row[0]);
                b.push_back(row[1]);
            }
            for (auto* axis : {&a, &b}) {
                std::ranges::sort(*axis);
                axis->erase(std::unique(axis->begin(), axis->end()), axis->end());
            }
            if (a.size() < 2 || b.size() < 2 || rows.size() != a.size() * b.size()) {
                throw std::runtime_error("InvertedMPCWalkPath: the response's " + name
                                         + " isn't every command on a grid of at least 2 × 2, once");
            }
            values.assign(outputs, Eigen::MatrixXd::Constant(a.size(), b.size(), std::nan("")));
            for (const auto& row : rows) {
                const auto i = std::distance(a.begin(), std::ranges::lower_bound(a, row[0]));
                const auto j = std::distance(b.begin(), std::ranges::lower_bound(b, row[1]));
                for (std::size_t k = 0; k < outputs; ++k) {
                    values[k](i, j) = row[2 + k];
                }
            }
            for (const auto& v : values) {
                if (!v.allFinite()) {
                    throw std::runtime_error("InvertedMPCWalkPath: the response's " + name
                                             + " has a command twice and another missing");
                }
            }
        }

        /// @brief The grid's commands, with SUBDIVISIONS steps in each grid step
        std::vector<double> subdivide(const std::vector<double>& axis) {
            std::vector<double> out{};
            for (std::size_t i = 0; i + 1 < axis.size(); ++i) {
                for (int k = 0; k < ResponseTable::SUBDIVISIONS; ++k) {
                    out.push_back(axis[i] + (axis[i + 1] - axis[i]) * k / ResponseTable::SUBDIVISIONS);
                }
            }
            out.push_back(axis.back());
            return out;
        }

        /// @brief The cell of the grid holding x, and how far across it x is (0 to 1), with x clamped to the grid
        std::pair<std::size_t, double> locate(const std::vector<double>& axis, const double x) {
            const double clamped = std::clamp(x, axis.front(), axis.back());
            const auto upper     = std::upper_bound(axis.begin(), axis.end() - 1, clamped);
            const std::size_t i  = std::max<std::ptrdiff_t>(0, std::distance(axis.begin(), upper) - 1);
            return {i, (clamped - axis[i]) / (axis[i + 1] - axis[i])};
        }

        /// @brief Of the candidates, the one closest to the target, or, within TIE_TOLERANCE of that, the one nearest
        /// zero
        template <typename Command, typename Delivered, typename Target>
        Command smallest_closest(const std::vector<Command>& commands,
                                 const std::vector<Delivered>& delivered,
                                 const Target& target) {
            double best = std::numeric_limits<double>::infinity();
            for (const auto& d : delivered) {
                best = std::min(best, (d - target).norm());
            }
            Command chosen    = commands.front();
            double chosen_sqr = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < commands.size(); ++i) {
                if ((delivered[i] - target).norm() <= best + ResponseTable::TIE_TOLERANCE
                    && commands[i].squaredNorm() < chosen_sqr) {
                    chosen     = commands[i];
                    chosen_sqr = commands[i].squaredNorm();
                }
            }
            return chosen;
        }

    }  // namespace

    Eigen::VectorXd ResponseTable::Grid::at(const double x, const double y) const {
        const auto [i, s] = locate(a, x);
        const auto [j, t] = locate(b, y);
        Eigen::VectorXd out(values.size());
        for (std::size_t k = 0; k < values.size(); ++k) {
            const auto& v = values[k];
            out[k]        = (1 - s) * (1 - t) * v(i, j) + s * (1 - t) * v(i + 1, j) + (1 - s) * t * v(i, j + 1)
                            + s * t * v(i + 1, j + 1);
        }
        return out;
    }

    ResponseTable ResponseTable::from_yaml(const YAML::Node& node) {
        ResponseTable table{};

        read_grid(node["translation"],
                  2,
                  "translation",
                  table.translation.a,
                  table.translation.b,
                  table.translation.values);
        read_grid(node["turn"], 1, "turn", table.turn.a, table.turn.b, table.turn.values);

        // The translation commands the inverse searches, and what they deliver
        for (const double x : subdivide(table.translation.a)) {
            for (const double y : subdivide(table.translation.b)) {
                table.translation_commands.emplace_back(x, y);
                table.translation_delivered.emplace_back(table.translation.at(x, y));
            }
        }
        return table;
    }

    Eigen::Vector3d ResponseTable::forward(const Eigen::Vector3d& command) const {
        const Eigen::VectorXd v = translation.at(command.x(), command.y());
        return {v[0], v[1], turn.at(command.x(), command.z())[0]};
    }

    Eigen::Vector3d ResponseTable::inverse(const Eigen::Vector3d& velocity) const {
        const Eigen::Vector2d u_xy =
            smallest_closest(translation_commands, translation_delivered, Eigen::Vector2d(velocity.head<2>()));

        // The turn rate the turn commands deliver at this forward command
        std::vector<Eigen::Matrix<double, 1, 1>> u_z{};
        std::vector<Eigen::Matrix<double, 1, 1>> delivered{};
        for (const double z : subdivide(turn.b)) {
            u_z.emplace_back(z);
            delivered.emplace_back(turn.at(u_xy.x(), z)[0]);
        }
        const double command_z = smallest_closest(u_z, delivered, Eigen::Matrix<double, 1, 1>(velocity.z()))[0];
        return {u_xy.x(), u_xy.y(), command_z};
    }

    Eigen::Vector3d ResponseTable::lower() const {
        return {translation.values[0].minCoeff(), translation.values[1].minCoeff(), turn.values[0].minCoeff()};
    }

    Eigen::Vector3d ResponseTable::upper() const {
        return {translation.values[0].maxCoeff(), translation.values[1].maxCoeff(), turn.values[0].maxCoeff()};
    }

    Eigen::Vector3d ResponseTable::command_lower() const {
        return {std::max(translation.a.front(), turn.a.front()), translation.b.front(), turn.b.front()};
    }

    Eigen::Vector3d ResponseTable::command_upper() const {
        return {std::min(translation.a.back(), turn.a.back()), translation.b.back(), turn.b.back()};
    }

}  // namespace module::planning::inverted_walk_mpc
