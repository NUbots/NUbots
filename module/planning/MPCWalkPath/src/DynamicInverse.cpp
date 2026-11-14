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
#include "DynamicInverse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace module::planning::walk_mpc {

    namespace {

        /// @brief Commands from -range to range, step apart
        std::vector<double> symmetric_axis(const double range, const double step) {
            const int count = int(std::lround(range / step));
            std::vector<double> out{};
            for (int i = -count; i <= count; ++i) {
                out.push_back(i * step);
            }
            return out;
        }

        /// @brief The index of the zero command on a symmetric axis
        std::size_t zero_index(const std::vector<double>& axis) {
            return (axis.size() - 1) / 2;
        }

        /// @brief The command nearest zero among candidates within threshold of the target, chosen so far
        template <typename Command>
        struct Smallest {
            Command chosen    = Command::Zero();
            double chosen_sqr = std::numeric_limits<double>::infinity();

            void offer(const Command& command, const double error, const double threshold) {
                if (error <= threshold && command.squaredNorm() < chosen_sqr) {
                    chosen     = command;
                    chosen_sqr = command.squaredNorm();
                }
            }
        };

    }  // namespace

    DynamicInverse::DynamicInverse(const HammersteinModel& model,
                                   const Eigen::Vector3d& range,
                                   const double step,
                                   const double horizon)
        : model(model)
        , range(range)
        , step(step)
        , n(std::max(1, int(std::lround(horizon / model.sample_time)))) {
        if (!(step > 0.0) || !(range.minCoeff() >= step)) {
            throw std::runtime_error("MPCWalkPath: the command grid needs a positive step within its "
                                     "range");
        }
        for (int i = 0; i < 3; ++i) {
            axes[i] = symmetric_axis(range[i], step);
        }

        // Each translation path's rise over the horizon from standing, for each command on its axis
        const HammersteinModel::Lags zero = HammersteinModel::Lags::Zero();
        for (const double u : axes[0]) {
            const HammersteinModel::Lags z = model.step(zero, {u, 0.0, 0.0}, n);
            rise[0].push_back(z[0]);
            rise[2].push_back(z[3]);
        }
        for (const double u : axes[1]) {
            const HammersteinModel::Lags z = model.step(zero, {0.0, u, 0.0}, n);
            rise[1].push_back(z[1]);
            rise[3].push_back(z[4]);
        }

        // What the planner can plan: the velocities the model settles to, translation with no turn and turning with no
        // sideways command, as the search finds the command
        lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        hi = -lo;
        for (const double x : axes[0]) {
            for (const double y : axes[1]) {
                const Eigen::Vector3d v = model.steady_state({x, y, 0.0});
                lo.head<2>()            = lo.head<2>().cwiseMin(v.head<2>());
                hi.head<2>()            = hi.head<2>().cwiseMax(v.head<2>());
            }
            for (const double w : axes[2]) {
                const double v = model.steady_state({x, 0.0, w}).z();
                lo.z()         = std::min(lo.z(), v);
                hi.z()         = std::max(hi.z(), v);
            }
        }
    }

    void DynamicInverse::reset() {
        lags.setZero();
        leftover = 0.0;
        u_prev.setZero();
    }

    void DynamicInverse::advance(const double elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0) {
            return;
        }
        // Whole samples, carrying the remainder to the next advance
        const double t       = elapsed + leftover;
        const double samples = std::floor(t / model.sample_time + 1e-6);
        leftover             = std::max(0.0, t - samples * model.sample_time);
        lags                 = model.step(lags, u_prev, int(std::min(samples, 1e6)));
    }

    void DynamicInverse::record(const Eigen::Vector3d& command) {
        u_prev = command.allFinite() ? command : Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d DynamicInverse::delivered() const {
        return HammersteinModel::delivered(lags);
    }

    Eigen::Vector3d DynamicInverse::after_step(const Eigen::Vector3d& command) const {
        return HammersteinModel::delivered(model.step(lags, command, n));
    }

    Eigen::Vector3d DynamicInverse::inverse(const Eigen::Vector3d& velocity) const {
        const auto clamp_to_range = [&](const int axis, const double u) {
            return std::clamp(u, -range[axis], range[axis]);
        };

        // Translation over the grid, from the lag states over the horizon with no command, and each path's rise for its
        // command in place of its rise for none. The paths from the turn command are taken at no turn.
        const HammersteinModel::Lags free = model.step(lags, Eigen::Vector3d::Zero(), n);
        const std::size_t zx              = zero_index(axes[0]);
        const std::size_t zy              = zero_index(axes[1]);
        const double cx                   = free[0] + free[1] + free[2] - rise[0][zx] - rise[1][zy];
        const double cy                   = free[3] + free[4] + free[5] - rise[2][zx] - rise[3][zy];
        const Eigen::Vector2d target      = velocity.head<2>();
        // Returns a vector, not an Eigen expression, which would refer to the temporary
        const auto grid_error = [&](const std::size_t a, const std::size_t b) -> Eigen::Vector2d {
            return Eigen::Vector2d(cx + rise[0][a] + rise[1][b], cy + rise[2][a] + rise[3][b]) - target;
        };

        double best    = std::numeric_limits<double>::infinity();
        std::size_t ba = zx;
        std::size_t bb = zy;
        for (std::size_t a = 0; a < axes[0].size(); ++a) {
            for (std::size_t b = 0; b < axes[1].size(); ++b) {
                const double error = grid_error(a, b).norm();
                if (error < best) {
                    best = error;
                    ba   = a;
                    bb   = b;
                }
            }
        }

        // Around the closest, more finely, with the model itself
        std::vector<Eigen::Vector2d> fine{};
        std::vector<double> fine_error{};
        for (int i = -REFINEMENT; i <= REFINEMENT; ++i) {
            for (int j = -REFINEMENT; j <= REFINEMENT; ++j) {
                const Eigen::Vector2d u(clamp_to_range(0, axes[0][ba] + step * i / REFINEMENT),
                                        clamp_to_range(1, axes[1][bb] + step * j / REFINEMENT));
                fine.push_back(u);
                fine_error.push_back((after_step({u.x(), u.y(), 0.0}).head<2>() - target).norm());
                best = std::min(best, fine_error.back());
            }
        }

        Smallest<Eigen::Vector2d> translation{};
        for (std::size_t a = 0; a < axes[0].size(); ++a) {
            for (std::size_t b = 0; b < axes[1].size(); ++b) {
                translation.offer({axes[0][a], axes[1][b]}, grid_error(a, b).norm(), best + TIE_TOLERANCE);
            }
        }
        for (std::size_t k = 0; k < fine.size(); ++k) {
            translation.offer(fine[k], fine_error[k], best + TIE_TOLERANCE);
        }
        const Eigen::Vector2d u_xy = translation.chosen;

        // The turn given that translation command, with the model itself
        using Scalar          = Eigen::Matrix<double, 1, 1>;
        const auto turn_error = [&](const double w) {
            return std::abs(after_step({u_xy.x(), u_xy.y(), w}).z() - velocity.z());
        };
        std::vector<double> turns  = axes[2];
        std::vector<double> errors{};
        double best_turn           = std::numeric_limits<double>::infinity();
        double closest_turn        = 0.0;
        for (const double w : axes[2]) {
            errors.push_back(turn_error(w));
            if (errors.back() < best_turn) {
                best_turn    = errors.back();
                closest_turn = w;
            }
        }
        for (int k = -REFINEMENT; k <= REFINEMENT; ++k) {
            turns.push_back(clamp_to_range(2, closest_turn + step * k / REFINEMENT));
            errors.push_back(turn_error(turns.back()));
            best_turn = std::min(best_turn, errors.back());
        }
        Smallest<Scalar> turn{};
        for (std::size_t k = 0; k < turns.size(); ++k) {
            turn.offer(Scalar(turns[k]), errors[k], best_turn + TIE_TOLERANCE);
        }

        return {u_xy.x(), u_xy.y(), turn.chosen[0]};
    }

}  // namespace module::planning::walk_mpc
