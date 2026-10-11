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
#include "HammersteinModel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace module::planning::modelled_walk_mpc {

    namespace {
        const std::array<std::string, 3> AXES = {"vx", "vy", "wz"};

        /// @brief The command into path n: its command axis, or ω·|vx| for the product path
        double path_command(const int n, const Eigen::Vector3d& u) {
            return n == HammersteinModel::PRODUCT ? u.z() * std::abs(u.x()) : u[n % 3];
        }

        /// @brief As the solver sees it: |vx| smoothed over vx's kink smoothing
        double path_command(const int n, const Eigen::Vector3d& u, const Eigen::Vector3d& eps) {
            return n == HammersteinModel::PRODUCT ? u.z() * std::sqrt(u.x() * u.x() + eps.x() * eps.x()) : u[n % 3];
        }

        /// @brief The command axis whose kink smoothing path n's map takes: ω's for the product path
        int kink_axis(const int n) {
            return n == HammersteinModel::PRODUCT ? 2 : n % 3;
        }

        HammersteinPath read_path(const YAML::Node& p, const std::string& name) {
            HammersteinPath path{};
            path.gain              = p["gain"].as<double>();
            path.pole              = p["pole"].as<double>();
            path.linear            = p["linear"].as<double>();
            path.offset            = p["offset"].as<double>();
            const auto coef        = p["coef"].as<std::vector<double>>();
            const auto translation = p["translation"].as<std::vector<double>>();
            if (int(coef.size()) != HammersteinPath::UNITS || int(translation.size()) != HammersteinPath::UNITS) {
                throw std::runtime_error("HammersteinModel: path " + name + " needs "
                                         + std::to_string(HammersteinPath::UNITS) + " breakpoints");
            }
            std::copy(coef.begin(), coef.end(), path.coef.begin());
            std::copy(translation.begin(), translation.end(), path.translation.begin());
            return path;
        }
    }  // namespace

    double HammersteinPath::input(const double u) const {
        double f = linear * u + offset;
        for (int n = 0; n < UNITS; ++n) {
            f += coef[n] * std::abs(u + translation[n]);
        }
        return f;
    }

    double HammersteinPath::input(const double u, const double eps) const {
        double f = linear * u + offset;
        for (int n = 0; n < UNITS; ++n) {
            const double x = u + translation[n];
            f += coef[n] * std::sqrt(x * x + eps * eps);
        }
        return f;
    }

    HammersteinModel HammersteinModel::from_yaml(const YAML::Node& node) {
        HammersteinModel model{};
        model.sample_time = node["sample_time"].as<double>();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                model.paths[3 * i + j] = read_path(node["paths"][AXES[i]][AXES[j]], AXES[i] + "←" + AXES[j]);
            }
        }
        if (const YAML::Node product = node["paths"]["wz"]["wz_abs_vx"]) {
            model.paths[PRODUCT] = read_path(product, "wz←wz·|vx|");
        }
        return model;
    }

    Eigen::Vector3d HammersteinModel::delivered(const Lags& z) {
        return {z.segment<3>(0).sum(), z.segment<3>(3).sum(), z.segment<3>(6).sum() + z[PRODUCT]};
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z, const Eigen::Vector3d& u) const {
        Lags next;
        for (int n = 0; n < PATHS; ++n) {
            next[n] = paths[n].pole * z[n] + paths[n].gain * paths[n].input(path_command(n, u));
        }
        return next;
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z, const Eigen::Vector3d& u, const int n) const {
        // z after n samples of z⁺ = p·z + w is pⁿ·z + (1 + p + ... + pⁿ⁻¹)·w
        Lags next;
        for (int i = 0; i < PATHS; ++i) {
            const double p      = paths[i].pole;
            const double pn     = std::pow(p, n);
            const double series = std::abs(1.0 - p) > 1e-12 ? (1.0 - pn) / (1.0 - p) : double(n);
            next[i]             = pn * z[i] + series * paths[i].gain * paths[i].input(path_command(i, u));
        }
        return next;
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z,
                                                  const Eigen::Vector3d& u,
                                                  const Eigen::Vector3d& eps) const {
        Lags next;
        for (int n = 0; n < PATHS; ++n) {
            next[n] = paths[n].pole * z[n] + paths[n].gain * paths[n].input(path_command(n, u, eps), eps[kink_axis(n)]);
        }
        return next;
    }

    Eigen::Vector3d HammersteinModel::steady_state(const Eigen::Vector3d& u) const {
        Lags z;
        for (int n = 0; n < PATHS; ++n) {
            z[n] = paths[n].gain * paths[n].input(path_command(n, u)) / (1.0 - paths[n].pole);
        }
        return delivered(z);
    }

}  // namespace module::planning::modelled_walk_mpc
