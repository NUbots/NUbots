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
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_RESPONSETABLE_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_RESPONSETABLE_HPP

#include <Eigen/Core>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace module::planning::inverted_walk_mpc {

    /// @brief The walk policy's measured steady-state response: the velocity each command delivered, on two grids of
    /// commands (InvertedMPCWalkPathResponse.yaml, written by codegen/import_response.py). Translation is the vx × vy
    /// grid with no turn, and turning the vx × ω grid with no sideways command; between grid commands the response is
    /// interpolated bilinearly.
    class ResponseTable {
    public:
        /// @brief Command steps per grid step the inverse searches: 10 on the evaluation's 0.1 grid is a 0.01 step
        static constexpr int SUBDIVISIONS = 10;
        /// @brief How much worse than the best match (m/s or rad/s) a command can be and still be chosen for being
        /// smaller, as the response is flat where the policy stands still
        static constexpr double TIE_TOLERANCE = 0.005;

        /// @brief Reads the tables, a row per command: [vx, vy, delivered vx, delivered vy] for translation and
        /// [vx, ω, delivered ω] for turning. Throws unless each is every command on its grid, once.
        static ResponseTable from_yaml(const YAML::Node& node);

        /// @brief The velocity [vx, vy, ω] the command delivers, with the command clamped to the tables' range
        [[nodiscard]] Eigen::Vector3d forward(const Eigen::Vector3d& command) const;

        /// @brief The smallest command that delivers the velocity [vx, vy, ω]: translation first, from the vx × vy
        /// grid, then the turn given that forward command, from the vx × ω grid. A velocity no command delivers gets
        /// the command that comes closest. Where several commands deliver it (where the policy stands still, or its
        /// turn on the spot), the one nearest zero.
        [[nodiscard]] Eigen::Vector3d inverse(const Eigen::Vector3d& velocity) const;

        /// @brief The smallest and largest velocity the tables deliver on each axis [vx, vy, ω]
        [[nodiscard]] Eigen::Vector3d lower() const;
        [[nodiscard]] Eigen::Vector3d upper() const;

        /// @brief The smallest and largest command in the tables on each axis [vx, vy, ω]
        [[nodiscard]] Eigen::Vector3d command_lower() const;
        [[nodiscard]] Eigen::Vector3d command_upper() const;

    private:
        /// @brief Values over a grid of commands (a, b), a row per a and a column per b
        struct Grid {
            std::vector<double> a{};
            std::vector<double> b{};
            /// @brief One table per output, rows over a
            std::vector<Eigen::MatrixXd> values{};

            /// @brief The outputs at (a, b), interpolated bilinearly, with (a, b) clamped to the grid
            [[nodiscard]] Eigen::VectorXd at(double x, double y) const;
        };

        Grid translation{};
        Grid turn{};
        /// @brief The translation commands the inverse searches, and what each delivers [vx, vy]
        std::vector<Eigen::Vector2d> translation_commands{};
        std::vector<Eigen::Vector2d> translation_delivered{};
    };

}  // namespace module::planning::inverted_walk_mpc

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_RESPONSETABLE_HPP
