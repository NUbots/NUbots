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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

using Catch::Approx;
using module::planning::inverted_walk_mpc::ResponseTable;
using utility::planning::HammersteinModel;

namespace {

    /// @brief The K1's response in the module's configuration (tests run in the build directory, where it is copied)
    YAML::Node k1_yaml() {
        return YAML::LoadFile("config/InvertedMPCWalkPathResponse.yaml");
    }

    /// @brief How close the inverse's command delivers what was asked: the search's step and its tie tolerance
    constexpr double MATCH = 0.02;

    /// @brief The Hammerstein model in the module's configuration, and its table as the module builds it: out to the
    /// identified range, every 0.01
    HammersteinModel k1_model() {
        return HammersteinModel::from_yaml(YAML::LoadFile("config/InvertedMPCWalkPathModel.yaml"));
    }
    const Eigen::Vector3d MODEL_RANGE(1.75, 1.75, 1.5);
    ResponseTable k1_model_table() {
        return ResponseTable::from_model(k1_model(), MODEL_RANGE, 0.01);
    }

    /// @brief How close the model's inverse delivers what was asked, by the model: the tie tolerance, and the
    /// refinement's step on the steepest edge
    constexpr double MODEL_MATCH = 0.01;

}  // namespace

TEST_CASE("At the grid's commands, the response is the table's", "[ResponseTable]") {
    const YAML::Node node     = k1_yaml();
    const ResponseTable table = ResponseTable::from_yaml(node);
    const auto translation    = node["translation"].as<std::vector<std::vector<double>>>();
    const auto turn           = node["turn"].as<std::vector<std::vector<double>>>();
    for (const std::size_t n : {std::size_t(0), std::size_t(33), translation.size() / 2, translation.size() - 1}) {
        const auto& row         = translation[n];
        const Eigen::Vector3d v = table.forward({row[0], row[1], 0.0});
        CHECK(v.x() == Approx(row[2]).margin(1e-12));
        CHECK(v.y() == Approx(row[3]).margin(1e-12));
    }
    for (const std::size_t n : {std::size_t(1), std::size_t(40), turn.size() / 3, turn.size() - 2}) {
        const auto& row = turn[n];
        CHECK(table.forward({row[0], 0.0, row[1]}).z() == Approx(row[2]).margin(1e-12));
    }
}

TEST_CASE("While walking, the inverse's command delivers the velocity asked for", "[ResponseTable]") {
    const ResponseTable table = ResponseTable::from_yaml(k1_yaml());
    for (const double ux : {-1.2, -0.5, 0.35, 0.8, 1.5}) {
        for (const double uy : {-1.0, -0.4, 0.0, 0.25, 0.9}) {
            for (const double uz : {-1.2, 0.0, 0.6}) {
                const Eigen::Vector3d wanted    = table.forward({ux, uy, uz});
                const Eigen::Vector3d command   = table.inverse(wanted);
                const Eigen::Vector3d delivered = table.forward(command);
                INFO("command " << ux << ", " << uy << ", " << uz << " delivers " << wanted.transpose());
                CHECK((delivered.head<2>() - wanted.head<2>()).norm() <= MATCH);
                // The turn is found for the forward command the translation chose, which can differ a little
                CHECK(std::abs(delivered.z() - wanted.z()) <= 2 * MATCH);
            }
        }
    }
}

TEST_CASE("Standing still is the zero command", "[ResponseTable]") {
    const ResponseTable table = ResponseTable::from_yaml(k1_yaml());
    CHECK(table.inverse(Eigen::Vector3d::Zero()).isZero());
}

TEST_CASE("A small velocity from standing steps out of the policy's standing region", "[ResponseTable]") {
    // From standing, the policy stands still for forward commands below about 0.15 m/s, sideways commands below about
    // 0.35 m/s and turns below about 0.4 rad/s
    const ResponseTable table = ResponseTable::from_yaml(k1_yaml());

    const Eigen::Vector3d forward = table.inverse({0.05, 0.0, 0.0});
    CHECK(forward.x() > 0.1);
    CHECK(table.forward(forward).x() == Approx(0.05).margin(MATCH));

    const Eigen::Vector3d sideways = table.inverse({0.0, 0.1, 0.0});
    CHECK((table.forward(sideways).head<2>() - Eigen::Vector2d(0.0, 0.1)).norm() <= MATCH);
    CHECK(sideways.head<2>().norm() > 0.1);

    const Eigen::Vector3d turn = table.inverse({0.0, 0.0, 0.3});
    CHECK(turn.z() > 0.5);
    CHECK(table.forward(turn).z() == Approx(0.3).margin(MATCH));
}

TEST_CASE("A velocity at the velocity limits can take a command beyond them", "[ResponseTable]") {
    // The policy delivers less than it is commanded at speed: 1.7 m/s forward takes about 1.9 m/s, 1.3 m/s sideways
    // about 1.6 m/s, and 1.5 rad/s turning at 1 m/s about 1.7 rad/s. The limits (1.75, 1.4, 1.5) bound the plan only.
    const ResponseTable table = ResponseTable::from_yaml(k1_yaml());

    const Eigen::Vector3d forward = table.inverse({1.7, 0.0, 0.0});
    CHECK(forward.x() > 1.75);
    CHECK(table.forward(forward).x() == Approx(1.7).margin(MATCH));

    const Eigen::Vector3d sideways = table.inverse({0.0, 1.3, 0.0});
    CHECK(sideways.y() > 1.4);
    CHECK(table.forward(sideways).y() == Approx(1.3).margin(MATCH));

    const Eigen::Vector3d turning = table.inverse({1.0, 0.0, 1.5});
    CHECK(turning.z() > 1.5);
    CHECK(table.forward(turning).z() == Approx(1.5).margin(2 * MATCH));
}

TEST_CASE("A velocity no command delivers gets the command that comes closest", "[ResponseTable]") {
    const ResponseTable table     = ResponseTable::from_yaml(k1_yaml());
    const Eigen::Vector3d command = table.inverse({5.0, 0.0, 0.0});
    CHECK(command.x() == Approx(table.command_upper().x()));
    CHECK(table.forward(command).x() == Approx(table.upper().x()).margin(MATCH));
}

TEST_CASE("A table that isn't every command on a grid is refused", "[ResponseTable]") {
    YAML::Node extra = k1_yaml();
    extra["turn"].push_back(std::vector<double>{0.05, 0.0, 0.0});
    CHECK_THROWS_AS(ResponseTable::from_yaml(extra), std::runtime_error);

    YAML::Node twice = k1_yaml();
    twice["turn"][1] = twice["turn"][0];
    CHECK_THROWS_AS(ResponseTable::from_yaml(twice), std::runtime_error);

    YAML::Node short_row        = k1_yaml();
    short_row["translation"][0] = std::vector<double>{0.0, 0.0, 0.0};
    CHECK_THROWS_AS(ResponseTable::from_yaml(short_row), std::runtime_error);
}


TEST_CASE("The model's table is the model's steady state", "[ResponseTable][model]") {
    const HammersteinModel model = k1_model();
    const ResponseTable table    = k1_model_table();
    for (const Eigen::Vector2d u :
         {Eigen::Vector2d(0.5, 0.0), Eigen::Vector2d(0.6, -0.3), Eigen::Vector2d(-1.2, 0.43)}) {
        const Eigen::Vector3d expected = model.steady_state({u.x(), u.y(), 0.0});
        CHECK((table.forward({u.x(), u.y(), 0.0}).head<2>() - expected.head<2>()).norm() < 1e-12);
    }
    for (const Eigen::Vector2d u : {Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.6, 0.8), Eigen::Vector2d(1.0, -1.5)}) {
        const double expected = model.steady_state({u.x(), 0.0, u.y()}).z();
        CHECK(table.forward({u.x(), 0.0, u.y()}).z() == Approx(expected).margin(1e-12));
    }
    // The planner's limits are what the model delivers within the identified range
    CHECK(table.upper().x() == Approx(model.steady_state({1.75, 0.0, 0.0}).x()));
    CHECK(table.command_upper().isApprox(MODEL_RANGE));
    CHECK(table.command_lower().isApprox(-MODEL_RANGE));
}

TEST_CASE("The model's inverse delivers the velocity asked for, by the model", "[ResponseTable][model]") {
    const HammersteinModel model = k1_model();
    const ResponseTable table    = k1_model_table();
    // Including commands on the steep edges of the dead zones (vy 0.43 and -0.45 m/s, vx 0.15 m/s), where a grid step
    // alone can't land on the velocity
    for (const double ux : {-1.2, -0.5, 0.15, 0.8, 1.5}) {
        for (const double uy : {-1.0, -0.45, 0.0, 0.43, 0.9}) {
            for (const double uz : {-1.2, 0.0, 0.6}) {
                const Eigen::Vector3d wanted    = model.steady_state({ux, uy, uz});
                const Eigen::Vector3d command   = table.inverse(wanted);
                const Eigen::Vector3d delivered = model.steady_state(command);
                INFO("command " << ux << ", " << uy << ", " << uz << " delivers " << wanted.transpose() << "; inverse "
                                << command.transpose() << " delivers " << delivered.transpose());
                CHECK((delivered.head<2>() - wanted.head<2>()).norm() <= MODEL_MATCH);
                CHECK(std::abs(delivered.z() - wanted.z()) <= 2 * MODEL_MATCH);
            }
        }
    }
}

TEST_CASE("The model's inverse finds the command along the edge of a dead zone", "[ResponseTable][model]") {
    // The model's vy dead zone ends between 0.40 and 0.45 m/s of command, where vy jumps from about 0.09 to 0.48 m/s
    const HammersteinModel model = k1_model();
    const ResponseTable table    = k1_model_table();
    for (const double vy : {0.15, 0.2, 0.3, 0.4}) {
        const Eigen::Vector3d command = table.inverse({0.0, vy, 0.0});
        INFO("vy " << vy << " gets " << command.transpose());
        CHECK(command.y() > 0.4);
        CHECK(command.y() < 0.45);
        CHECK(model.steady_state(command).y() == Approx(vy).margin(MODEL_MATCH));
    }
}

TEST_CASE("At standing still the model holds the robot with commands inside its dead zones", "[ResponseTable][model]") {
    // The model drifts at the zero command (about -0.006 m/s, 0.008 m/s and -0.016 rad/s), which the inverse cancels
    const HammersteinModel model  = k1_model();
    const Eigen::Vector3d command = k1_model_table().inverse(Eigen::Vector3d::Zero());
    CHECK(command.cwiseAbs().maxCoeff() < 0.1);
    CHECK(model.steady_state(command).cwiseAbs().maxCoeff() < MODEL_MATCH);
}

TEST_CASE("The model's commands stay within its identified range", "[ResponseTable][model]") {
    // A velocity beyond what the model delivers gets the command that comes closest, or, as the response flattens
    // towards the range's edge, the smallest within the tie tolerance of it
    const ResponseTable table     = k1_model_table();
    const Eigen::Vector3d command = table.inverse({5.0, -5.0, 5.0});
    CHECK((command.cwiseAbs() - MODEL_RANGE).maxCoeff() <= 1e-12);
    CHECK(table.forward(command).x() == Approx(table.upper().x()).margin(MODEL_MATCH));
    CHECK(table.forward(command).y() == Approx(table.lower().y()).margin(MODEL_MATCH));
    CHECK_THROWS_AS(ResponseTable::from_model(k1_model(), MODEL_RANGE, 0.0), std::runtime_error);
}
