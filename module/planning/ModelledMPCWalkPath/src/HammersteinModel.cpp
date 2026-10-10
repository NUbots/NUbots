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
                const YAML::Node p     = node["paths"][AXES[i]][AXES[j]];
                HammersteinPath& path  = model.paths[3 * i + j];
                path.gain              = p["gain"].as<double>();
                path.pole              = p["pole"].as<double>();
                path.linear            = p["linear"].as<double>();
                path.offset            = p["offset"].as<double>();
                const auto coef        = p["coef"].as<std::vector<double>>();
                const auto translation = p["translation"].as<std::vector<double>>();
                if (int(coef.size()) != HammersteinPath::UNITS || int(translation.size()) != HammersteinPath::UNITS) {
                    throw std::runtime_error("HammersteinModel: path " + AXES[i] + "←" + AXES[j] + " needs "
                                             + std::to_string(HammersteinPath::UNITS) + " breakpoints");
                }
                std::copy(coef.begin(), coef.end(), path.coef.begin());
                std::copy(translation.begin(), translation.end(), path.translation.begin());
            }
        }
        return model;
    }

    Eigen::Vector3d HammersteinModel::delivered(const Lags& z) {
        return {z.segment<3>(0).sum(), z.segment<3>(3).sum(), z.segment<3>(6).sum()};
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z, const Eigen::Vector3d& u) const {
        Lags next;
        for (int n = 0; n < 9; ++n) {
            next[n] = paths[n].pole * z[n] + paths[n].gain * paths[n].input(u[n % 3]);
        }
        return next;
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z, const Eigen::Vector3d& u, const int n) const {
        // z after n samples of z⁺ = p·z + w is pⁿ·z + (1 + p + ... + pⁿ⁻¹)·w
        Lags next;
        for (int i = 0; i < 9; ++i) {
            const double p      = paths[i].pole;
            const double pn     = std::pow(p, n);
            const double series = std::abs(1.0 - p) > 1e-12 ? (1.0 - pn) / (1.0 - p) : double(n);
            next[i]             = pn * z[i] + series * paths[i].gain * paths[i].input(u[i % 3]);
        }
        return next;
    }

    HammersteinModel::Lags HammersteinModel::step(const Lags& z,
                                                  const Eigen::Vector3d& u,
                                                  const Eigen::Vector3d& eps) const {
        Lags next;
        for (int n = 0; n < 9; ++n) {
            next[n] = paths[n].pole * z[n] + paths[n].gain * paths[n].input(u[n % 3], eps[n % 3]);
        }
        return next;
    }

    Eigen::Vector3d HammersteinModel::steady_state(const Eigen::Vector3d& u) const {
        Lags z;
        for (int n = 0; n < 9; ++n) {
            z[n] = paths[n].gain * paths[n].input(u[n % 3]) / (1.0 - paths[n].pole);
        }
        return delivered(z);
    }

}  // namespace module::planning::modelled_walk_mpc
