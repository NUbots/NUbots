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
#ifndef MODULE_PLANNING_MODELLEDMPCWALKPATH_HAMMERSTEINMODEL_HPP
#define MODULE_PLANNING_MODELLEDMPCWALKPATH_HAMMERSTEINMODEL_HPP

#include <Eigen/Core>
#include <array>
#include <yaml-cpp/yaml.h>

namespace module::planning::modelled_walk_mpc {

    /// @brief One path of the Hammerstein model: a command axis through a piecewise-linear map, then a first-order
    /// lag with a one-step delay, z⁺ = pole·z + gain·f(u)
    struct HammersteinPath {
        /// @brief Breakpoints of the piecewise-linear map
        static constexpr int UNITS = 6;

        /// @brief The linear block's numerator b
        double gain = 0.0;
        /// @brief The linear block's pole p
        double pole = 0.0;
        /// @brief f(u) = linear·u + offset + Σₙ coef[n]·|u + translation[n]|
        double linear = 0.0;
        double offset = 0.0;
        std::array<double, UNITS> coef{};
        /// @brief Minus the breakpoints
        std::array<double, UNITS> translation{};

        /// @brief The piecewise-linear map f(u)
        [[nodiscard]] double input(double u) const;
        /// @brief The map with its kinks smoothed, |x| ≈ √(x² + ε²), as the MPC's solver sees it
        [[nodiscard]] double input(double u, double eps) const;
    };

    /// @brief The walk policy's identified response: from the velocity command to the gait-averaged delivered
    /// velocity. One path per (output axis, command axis), and each output is the sum of its three paths' lag states.
    /// This is MATLAB's idnlhw with idPiecewiseLinear input maps, nb = nf = nk = 1 and no output map.
    struct HammersteinModel {
        /// @brief One lag state per path, output-major: vx←(vx, vy, ω), vy←(vx, vy, ω), ω←(vx, vy, ω)
        using Lags = Eigen::Matrix<double, 9, 1>;

        /// @brief Sample time (s)
        double sample_time = 0.02;
        /// @brief paths[3·output + command]
        std::array<HammersteinPath, 9> paths{};

        /// @brief Reads the model from its configuration (written by codegen/import_model.py)
        static HammersteinModel from_yaml(const YAML::Node& node);

        /// @brief The delivered velocity [vx, vy, ω] the lag states give
        [[nodiscard]] static Eigen::Vector3d delivered(const Lags& z);

        /// @brief One sample with the command u held
        [[nodiscard]] Lags step(const Lags& z, const Eigen::Vector3d& u) const;
        /// @brief n samples with the command u held, in closed form
        [[nodiscard]] Lags step(const Lags& z, const Eigen::Vector3d& u, int n) const;
        /// @brief One sample with the maps' kinks smoothed by eps per command axis, as the MPC's solver sees it
        [[nodiscard]] Lags step(const Lags& z, const Eigen::Vector3d& u, const Eigen::Vector3d& eps) const;

        /// @brief The delivered velocity the robot settles to with the command u held
        [[nodiscard]] Eigen::Vector3d steady_state(const Eigen::Vector3d& u) const;
    };

}  // namespace module::planning::modelled_walk_mpc

#endif  // MODULE_PLANNING_MODELLEDMPCWALKPATH_HAMMERSTEINMODEL_HPP
