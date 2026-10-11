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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>

using Catch::Approx;
using module::planning::modelled_walk_mpc::HammersteinModel;

namespace {

    /// @brief The K1 model in the module's configuration (tests run in the build directory, where it is copied)
    HammersteinModel k1_model() {
        return HammersteinModel::from_yaml(YAML::LoadFile("config/ModelledMPCWalkPath.yaml")["model"]);
    }

    void check(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
        CHECK(actual.x() == Approx(expected.x()).margin(1e-9));
        CHECK(actual.y() == Approx(expected.y()).margin(1e-9));
        CHECK(actual.z() == Approx(expected.z()).margin(1e-9));
    }

}  // namespace

// Reference values from evaluating the imported parameters in Python (numpy), with idPiecewiseLinear's map
// f(u) = L·u + d + Σ c·|u + t| and the linear blocks b/(1 − p·q⁻¹)
TEST_CASE("The steady state matches the reference evaluation of the model", "[HammersteinModel]") {
    const HammersteinModel model = k1_model();
    check(model.steady_state({0.5, 0.0, 0.0}), {0.45564513076, 0.0044679114877, 0.0048116700078});
    check(model.steady_state({0.0, 0.4, 0.0}), {0.0075811649402, 0.22070424661, 0.0048116700078});
    check(model.steady_state({0.0, 0.0, 1.0}), {0.0075811649402, 0.0044679114877, 0.4797848082});
    check(model.steady_state({0.6, -0.3, 0.8}), {0.56933830264, -0.035084026427, 0.71306695625});
    // Backwards at PlanWalkPath's limit, which is inside the policy's backward dead zone
    check(model.steady_state({-0.15, 0.0, 0.0}), {-0.05153969807, 0.0044679114877, 0.0048116700078});
    // Turning while walking, through the product path
    check(model.steady_state({1.0, 0.0, 0.5}), {1.0112084806, 0.0044679114877, 0.61611755088});
    check(model.steady_state({0.0, 0.0, 0.5}), {0.0075811649402, 0.0044679114877, 0.17989727742});
}

TEST_CASE("The product path turns by the turn command scaled by the forward speed's size", "[HammersteinModel]") {
    const HammersteinModel model = k1_model();
    // Forwards or backwards turn the same, and differently from turning on the spot
    CHECK(model.steady_state({1.0, 0.0, 0.5}).z() == Approx(model.steady_state({-1.0, 0.0, 0.5}).z()).margin(1e-12));
    CHECK(std::abs(model.steady_state({1.0, 0.0, 0.5}).z() - model.steady_state({0.0, 0.0, 0.5}).z()) > 0.1);
    // Without a turn command the product path has only its offset, whatever the forward speed
    HammersteinModel no_product                      = model;
    no_product.paths[HammersteinModel::PRODUCT].gain = 0.0;
    const double offset                              = model.paths[HammersteinModel::PRODUCT].gain
                          * model.paths[HammersteinModel::PRODUCT].input(0.0)
                          / (1.0 - model.paths[HammersteinModel::PRODUCT].pole);
    for (const double vx : {0.0, 0.5, -1.0}) {
        CHECK(model.steady_state({vx, 0.0, 0.0}).z() - no_product.steady_state({vx, 0.0, 0.0}).z()
              == Approx(offset).margin(1e-12));
    }
}

TEST_CASE("The step response matches the reference evaluation of the model", "[HammersteinModel]") {
    const HammersteinModel model = k1_model();
    HammersteinModel::Lags z     = HammersteinModel::Lags::Zero();
    const Eigen::Vector3d u(0.6, 0.0, 0.0);
    for (int k = 1; k <= 25; ++k) {
        z = model.step(z, u);
        if (k == 1) {
            check(HammersteinModel::delivered(z), {0.029837879979, 0.00029973648391, 0.021029889699});
        }
        if (k == 10) {
            check(HammersteinModel::delivered(z), {0.23699703489, 0.0022368147532, 0.018563300313});
        }
        if (k == 25) {
            check(HammersteinModel::delivered(z), {0.42111889369, 0.0036806134661, 0.0094121206789});
        }
    }
}

TEST_CASE("n samples in closed form are n single samples", "[HammersteinModel]") {
    const HammersteinModel model = k1_model();
    HammersteinModel::Lags start;
    start << 0.1, -0.05, 0.02, 0.0, 0.3, -0.01, 0.01, 0.0, 0.4, 0.05;
    const Eigen::Vector3d u(0.7, -0.4, 0.9);
    HammersteinModel::Lags z = start;
    for (int k = 0; k < 37; ++k) {
        z = model.step(z, u);
    }
    CHECK((model.step(start, u, 37) - z).cwiseAbs().maxCoeff() < 1e-12);
    CHECK(model.step(start, u, 0) == start);
}

TEST_CASE("Smoothing the kinks changes the map only near the breakpoints", "[HammersteinModel]") {
    const HammersteinModel model = k1_model();
    for (const auto& path : model.paths) {
        for (double u = -1.5; u <= 1.5; u += 0.01) {
            // √(x² + ε²) − |x| ≤ ε, so the smoothed map is within ε·Σ|c| of the exact one
            double bound = 0.0;
            for (const double c : path.coef) {
                bound += std::abs(c) * 0.05;
            }
            CHECK(std::abs(path.input(u, 0.05) - path.input(u)) <= bound + 1e-12);
            CHECK(path.input(u, 1e-9) == Approx(path.input(u)).margin(1e-8));
        }
    }
}
