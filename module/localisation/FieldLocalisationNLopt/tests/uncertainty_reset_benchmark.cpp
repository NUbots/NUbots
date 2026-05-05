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

// Reproduces the cost-driven grid search performed by FieldLocalisationNLopt::uncertainty_reset
// using the same OccupancyMap utilities and NLopt configuration. The aim is twofold:
//   (1) verify that, given clean synthetic field-line observations, the local grid search
//       recovers a pose close to ground truth, and
//   (2) report wall-clock runtime so changes to the reset's inner loop, NLopt parameters,
//       or grid resolution can be regression-tracked under `./b test`.
//
// The reactor's `uncertainty_reset` member function is not invoked directly because doing so
// requires a NUClear PowerPlant. Instead, the geometric and optimisation kernels that dominate
// runtime are reconstructed verbatim here.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <nlopt.hpp>
#include <vector>

#include "message/support/FieldDescription.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"
#include "utility/localisation/OccupancyMap.hpp"

namespace {

    using message::support::FieldDescription;
    using utility::localisation::OccupancyMap;
    using utility::localisation::setup_fieldline_distance_map;

    /// Mirrors `module::localisation::FieldLocalisationNLopt::compute_Hfw`.
    Eigen::Isometry3d compute_Hfw(const Eigen::Vector3d& state) {
        return Eigen::Translation<double, 3>(state.x(), state.y(), 0)
               * Eigen::AngleAxis<double>(state.z(), Eigen::Vector3d::UnitZ());
    }

    /// Mirrors `module::localisation::FieldLocalisationNLopt::position_in_map`.
    Eigen::Vector2i position_in_map(OccupancyMap<double>& map,
                                    double grid_size,
                                    const Eigen::Vector3d& particle,
                                    const Eigen::Vector3d& rPWw) {
        Eigen::Vector3d rPFf = compute_Hfw(particle) * rPWw;
        return Eigen::Vector2i(map.get_length() / 2 - static_cast<int>(std::round(rPFf.y() / grid_size)),
                               map.get_width() / 2 + static_cast<int>(std::round(rPFf.x() / grid_size)));
    }

    /// A canonical RoboCup-ish field description, mirroring the SoccerConfig defaults.
    FieldDescription make_field_description() {
        FieldDescription fd{};
        auto& d                   = fd.dimensions;
        d.line_width              = 0.05;
        d.field_length            = 6.8;
        d.field_width             = 5.0;
        d.goal_depth              = 0.4;
        d.goal_width              = 1.95;
        d.goal_area_length        = 1.05;
        d.goal_area_width         = 2.62;
        d.goal_crossbar_height    = 0.55;
        d.goalpost_width          = 0.10;
        d.goalpost_depth          = 0.10;
        d.goal_crossbar_width     = 0.10;
        d.goal_crossbar_depth     = 0.10;
        d.goal_net_height         = 1.0;
        d.penalty_mark_distance   = 1.27;
        d.center_circle_diameter  = 1.5;
        d.border_strip_min_width  = 0.38;
        d.penalty_area_length     = 1.55;
        d.penalty_area_width      = 4.05;
        return fd;
    }

    /// Sample points along the perimeter and centre line of the field. These are returned in
    /// FIELD frame so a known ground-truth `state` can transform them into a synthetic world-frame
    /// `rPWw` set, exactly as the camera would have produced.
    std::vector<Eigen::Vector3d> sample_field_line_points_in_field_frame(const FieldDescription& fd, double spacing) {
        const double L = fd.dimensions.field_length / 2.0;
        const double W = fd.dimensions.field_width / 2.0;
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(static_cast<size_t>((4 * L + 4 * W + 2 * W) / spacing) + 16);

        // Touchlines (long sides)
        for (double x = -L; x <= L; x += spacing) {
            pts.emplace_back(x, W, 0.0);
            pts.emplace_back(x, -W, 0.0);
        }
        // Goal lines (short sides)
        for (double y = -W; y <= W; y += spacing) {
            pts.emplace_back(L, y, 0.0);
            pts.emplace_back(-L, y, 0.0);
        }
        // Centre line
        for (double y = -W; y <= W; y += spacing) {
            pts.emplace_back(0.0, y, 0.0);
        }
        return pts;
    }

    struct CostContext {
        OccupancyMap<double>* map;
        double grid_size;
        const std::vector<Eigen::Vector3d>* observations_world;
        Eigen::Vector3d initial_guess;
        double field_line_distance_weight;
        double state_change_weight;
        double out_of_field_cost;
    };

    /// Mirrors the field-line-only branch of `run_field_line_optimisation`'s objective.
    double objective(unsigned /*n*/, const double* x, double* /*grad*/, void* data) {
        auto* ctx = static_cast<CostContext*>(data);
        Eigen::Vector3d particle(x[0], x[1], x[2]);

        double cost = 0.0;
        for (const auto& rPWw : *ctx->observations_world) {
            const Eigen::Vector2i mp = position_in_map(*ctx->map, ctx->grid_size, particle, rPWw);
            double v                 = ctx->map->get_occupancy_value(mp.x(), mp.y());
            v                        = (v == -1) ? ctx->out_of_field_cost : v;
            cost += ctx->field_line_distance_weight * v * v;
        }
        cost /= !ctx->observations_world->empty() ? static_cast<double>(ctx->observations_world->size()) : 1.0;
        cost += ctx->state_change_weight * (particle - ctx->initial_guess).squaredNorm();
        return cost;
    }

    struct OptParams {
        double xtol_rel = 1e-6;
        double ftol_rel = 1e-6;
        unsigned maxeval = 1000;
        Eigen::Vector3d change_limit = Eigen::Vector3d(0.5, 0.5, 0.5);
    };

    std::pair<Eigen::Vector3d, double> optimise(CostContext& ctx, const OptParams& params) {
        nlopt::opt opt(nlopt::LN_COBYLA, 3);
        opt.set_xtol_rel(params.xtol_rel);
        opt.set_ftol_rel(params.ftol_rel);
        opt.set_maxeval(params.maxeval);
        opt.set_min_objective(&objective, &ctx);

        Eigen::Vector3d lb = ctx.initial_guess - params.change_limit;
        Eigen::Vector3d ub = ctx.initial_guess + params.change_limit;
        opt.set_lower_bounds(std::vector<double>{lb.x(), lb.y(), lb.z()});
        opt.set_upper_bounds(std::vector<double>{ub.x(), ub.y(), ub.z()});

        std::vector<double> x{ctx.initial_guess.x(), ctx.initial_guess.y(), ctx.initial_guess.z()};
        double final_cost = 0.0;
        opt.optimize(x, final_cost);
        return {Eigen::Vector3d(x[0], x[1], x[2]), final_cost};
    }

}  // namespace

TEST_CASE("UncertaintyReset local grid search recovers a synthetic ground-truth pose",
          "[localisation][uncertainty_reset]") {

    // Defaults match `data/config/FieldLocalisationNLopt.yaml` so the test reflects production behaviour.
    constexpr double grid_size                = 1e-2;
    constexpr double field_line_distance_w    = 1.0;
    constexpr double state_change_weight      = 2.0;
    constexpr double out_of_field_cost        = 3.0;

    // Reset-time grid params. Production uses window_size=2.0/step_size=0.25/num_angles=8/maxeval=1000.
    // The test uses a smaller window and a tighter maxeval so it completes quickly under CI; the
    // shape of the work is the same.
    constexpr double window_size    = 1.5;
    constexpr double step_size      = 0.25;
    constexpr int num_angles        = 8;
    constexpr double cost_threshold = 2.0;

    OptParams opt_params;
    opt_params.maxeval = 100;

    const FieldDescription fd = make_field_description();
    OccupancyMap<double> map  = setup_fieldline_distance_map(fd, grid_size);

    // Ground truth: 2 m along x, 1 m along y, ~17 deg yaw on the right-hand half of the field.
    const Eigen::Vector3d truth(2.0, 1.0, 0.3);

    // Synthesise observations: sample ~field-line points in field frame, then push them through
    // Hwf = compute_Hfw(truth)^-1 to get rPWw, the format the reactor consumes.
    const auto field_pts        = sample_field_line_points_in_field_frame(fd, 0.40);
    const Eigen::Isometry3d Hwf = compute_Hfw(truth).inverse();
    std::vector<Eigen::Vector3d> rPWw;
    rPWw.reserve(field_pts.size());
    for (const auto& p : field_pts) {
        rPWw.push_back(Hwf * p);
    }
    REQUIRE(rPWw.size() > 50);

    // Last "certain" state — corrupted to mimic a robot whose pose has drifted.
    const Eigen::Vector3d last_certain_state = truth + Eigen::Vector3d(0.8, -0.6, 0.4);

    // Mirror reset.cpp's local search: dx, dy in [-window_size, +window_size] step step_size, all yaws.
    std::vector<double> angles;
    angles.reserve(num_angles);
    for (int i = 0; i < num_angles; ++i) {
        angles.push_back(i * (2.0 * M_PI / static_cast<double>(num_angles)));
    }

    const auto t0 = std::chrono::steady_clock::now();

    std::pair<Eigen::Vector3d, double> best{Eigen::Vector3d::Zero(), std::numeric_limits<double>::infinity()};
    long long evaluated = 0;
    for (double dx = -window_size; dx <= window_size + 1e-9; dx += step_size) {
        for (double dy = -window_size; dy <= window_size + 1e-9; dy += step_size) {
            for (double angle : angles) {
                Eigen::Vector3d guess(last_certain_state.x() + dx, last_certain_state.y() + dy, angle);
                CostContext ctx{&map,
                                grid_size,
                                &rPWw,
                                guess,
                                field_line_distance_w,
                                state_change_weight,
                                out_of_field_cost};
                auto [opt_state, opt_cost] = optimise(ctx, opt_params);
                if (opt_cost < best.second) {
                    best = {opt_state, opt_cost};
                }
                ++evaluated;
            }
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double duration_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

    INFO("Local grid search evaluated " << evaluated << " hypotheses in " << duration_s << " s");
    INFO("Best cost = " << best.second);
    INFO("Best state = " << best.first.transpose());
    INFO("Truth      = " << truth.transpose());
    std::cout << "[uncertainty_reset_benchmark] hypotheses=" << evaluated
              << " duration_s=" << duration_s
              << " best_cost=" << best.second
              << " maxeval_per_call=" << opt_params.maxeval << std::endl;

    // Correctness: the search should land near the truth.
    const double xy_err  = (best.first.head<2>() - truth.head<2>()).norm();
    const double yaw_err = std::abs(std::remainder(best.first.z() - truth.z(), 2.0 * M_PI));
    REQUIRE(best.second < cost_threshold);
    REQUIRE(xy_err < 0.30);
    REQUIRE(yaw_err < 0.20);
}
