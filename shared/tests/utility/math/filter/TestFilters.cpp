/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <array>
#include <catch.hpp>
#include <utility>

#include "VanDerPolModel.hpp"

#include "utility/math/filter/KalmanFilter.hpp"
#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/support/yaml_expression.hpp"

using utility::support::Expression;
using utility::support::resolve_expression;


TEST_CASE("Test the UKF", "[utility][math][filter][UKF]") {

    const YAML::Node config             = YAML::LoadFile("tests/TestFilters.yaml");
    const Eigen::Vector2d process_noise = config["parameters"]["noise"]["process"].as<Expression>();
    const Eigen::Matrix<double, 1, 1> measurement_noise(
        double(config["parameters"]["noise"]["measurement"].as<Expression>()));
    const Eigen::Vector2d initial_state = config["parameters"]["initial"]["state"].as<Expression>();
    const Eigen::Matrix2d initial_covariance =
        Eigen::Vector2d(config["parameters"]["initial"]["covariance"].as<Expression>()).asDiagonal();
    const double deltaT = config["parameters"]["delta_t"].as<Expression>();

    // Resolve the Expression list types into actual types
    const std::vector<Eigen::Vector2d> true_state = resolve_expression<Eigen::Vector2d>(config["true_state"]);
    const std::vector<Eigen::Matrix<double, 1, 1>> measurements =
        resolve_expression<Eigen::Matrix<double, 1, 1>, double>(config["measurements"]);

    REQUIRE(true_state.size() == measurements.size());

    utility::math::filter::UKF<double, shared::tests::VanDerPolModel> model_filter;

    INFO("Configuring the UKF with")
    INFO("    Time step.........: " << deltaT);
    INFO("    Process Noise.....: " << process_noise.transpose());
    INFO("    Initial State.....: " << initial_state.transpose());
    INFO("    Initial Covariance: \n" << initial_covariance);
    model_filter.model.process_noise = process_noise;
    model_filter.set_state(initial_state, initial_covariance);

    INFO("Feeding noisy measurements into the filter");
    std::vector<double> innovations;
    std::vector<std::pair<utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateVec,
                          utility::math::filter::UKF<double, shared::tests::VanDerPolModel>::StateMat>>
        actual_state;
    innovations.reserve(true_state.size());
    actual_state.reserve(true_state.size());
    for (const auto& measurement : measurements) {
        model_filter.measure(measurement, measurement_noise);
        model_filter.time(deltaT);
        innovations.push_back(measurement.x() - model_filter.get_state().x());
        actual_state.push_back(std::make_pair(model_filter.get_state(), model_filter.get_covariance()));
    }

    INFO("Calculating statistics")

    double count_x1                  = 0.0;
    double count_x2                  = 0.0;
    double mean_innovations          = 0.0;
    double mean_x1_boundary          = 0.0;
    double mean_x2_boundary          = 0.0;
    Eigen::Vector2d mean_state_error = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < actual_state.size(); ++i) {
        Eigen::Vector2d state_error = Eigen::Vector2d(true_state[i]) - actual_state[i].first;
        double covariance_bounds_x1 = std::sqrt(actual_state[i].second(0, 0));
        double covariance_bounds_x2 = std::sqrt(actual_state[i].second(1, 1));
        mean_x1_boundary += covariance_bounds_x1;
        mean_x2_boundary += covariance_bounds_x2;

        count_x1 += (std::abs(state_error.x()) - covariance_bounds_x1) > 0.0 ? 1.0 : 0.0;
        count_x2 += (std::abs(state_error.y()) - covariance_bounds_x2) > 0.0 ? 1.0 : 0.0;

        mean_innovations += innovations[i];
        mean_state_error += state_error;
    }

    mean_innovations /= innovations.size();
    mean_state_error /= actual_state.size();
    mean_x1_boundary /= actual_state.size();
    mean_x2_boundary /= actual_state.size();

    double percentage_x1 = 100.0 * count_x1 / actual_state.size();
    double percentage_x2 = 100.0 * count_x2 / actual_state.size();

    INFO("The mean of the innovations is: " << mean_innovations << ". This should be small.");
    INFO("The mean of the state errors is: " << mean_state_error.transpose() << ". This should be small.");
    INFO(percentage_x1 << "% of state 1 estimates exceed the 1\u03C3 boundary");
    INFO(percentage_x2 << "% of state 2 estimates exceed the 1\u03C3 boundary");
    INFO("The mean 1\u03C3 boundary for state 1 is [" << -mean_x1_boundary << ", " << mean_x1_boundary << "]");
    INFO("The mean 1\u03C3 boundary for state 2 is [" << -mean_x2_boundary << ", " << mean_x2_boundary << "]");

    REQUIRE(percentage_x1 <= 30.0);
}


TEST_CASE("Test the ParticleFilter", "[utility][math][filter][ParticleFilter]") {

    const YAML::Node config             = YAML::LoadFile("tests/TestFilters.yaml");
    const Eigen::Vector2d process_noise = config["parameters"]["noise"]["process"].as<Expression>();
    const Eigen::Matrix<double, 1, 1> measurement_noise(
        double(config["parameters"]["noise"]["measurement"].as<Expression>()));
    const Eigen::Vector2d initial_state = config["parameters"]["initial"]["state"].as<Expression>();
    const Eigen::Matrix2d initial_covariance =
        Eigen::Vector2d(config["parameters"]["initial"]["covariance"].as<Expression>()).asDiagonal();
    const double deltaT           = config["parameters"]["delta_t"].as<Expression>();
    const int number_of_particles = config["parameters"]["num_particles"].as<Expression>();

    // Resolve the Expression list types into actual types
    const std::vector<Eigen::Vector2d> true_state = resolve_expression<Eigen::Vector2d>(config["true_state"]);
    const std::vector<Eigen::Matrix<double, 1, 1>> measurements =
        resolve_expression<Eigen::Matrix<double, 1, 1>, double>(config["measurements"]);

    REQUIRE(true_state.size() == measurements.size());

    utility::math::filter::ParticleFilter<double, shared::tests::VanDerPolModel> model_filter;

    INFO("Configuring the ParticleFilter with");
    INFO("    Time step..........: " << deltaT);
    INFO("    Number of Particles: " << number_of_particles);
    INFO("    Process Noise......: " << process_noise.transpose());
    INFO("    Initial State......: " << initial_state.transpose());
    INFO("    Initial Covariance.: \n" << initial_covariance);
    model_filter.model.n_particles   = number_of_particles;
    model_filter.model.process_noise = process_noise;
    model_filter.set_state(initial_state, initial_covariance);

    INFO("Feeding noisy measurements into the filter");
    std::vector<double> innovations;
    std::vector<std::pair<utility::math::filter::ParticleFilter<double, shared::tests::VanDerPolModel>::StateVec,
                          utility::math::filter::ParticleFilter<double, shared::tests::VanDerPolModel>::StateMat>>
        actual_state;
    innovations.reserve(true_state.size());
    actual_state.reserve(true_state.size());
    for (const auto& measurement : measurements) {
        model_filter.measure(measurement, measurement_noise);
        model_filter.time(deltaT);
        innovations.push_back(measurement.x() - model_filter.get_state().x());
        actual_state.push_back(std::make_pair(model_filter.get_state(), model_filter.get_covariance()));
    }

    INFO("Calculating statistics")

    double count_x1                  = 0.0;
    double count_x2                  = 0.0;
    double mean_innovations          = 0.0;
    double mean_x1_boundary          = 0.0;
    double mean_x2_boundary          = 0.0;
    Eigen::Vector2d mean_state_error = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < actual_state.size(); ++i) {
        Eigen::Vector2d state_error = Eigen::Vector2d(true_state[i]) - actual_state[i].first;
        double covariance_bounds_x1 = std::sqrt(actual_state[i].second(0, 0));
        double covariance_bounds_x2 = std::sqrt(actual_state[i].second(1, 1));
        mean_x1_boundary += covariance_bounds_x1;
        mean_x2_boundary += covariance_bounds_x2;

        count_x1 += (std::abs(state_error.x()) - covariance_bounds_x1) > 0.0 ? 1.0 : 0.0;
        count_x2 += (std::abs(state_error.y()) - covariance_bounds_x2) > 0.0 ? 1.0 : 0.0;

        mean_innovations += innovations[i];
        mean_state_error += state_error;
    }

    mean_innovations /= innovations.size();
    mean_state_error /= actual_state.size();
    mean_x1_boundary /= actual_state.size();
    mean_x2_boundary /= actual_state.size();

    double percentage_x1 = 100.0 * count_x1 / actual_state.size();
    double percentage_x2 = 100.0 * count_x2 / actual_state.size();

    INFO("The mean of the innovations is: " << mean_innovations << ". This should be small.");
    INFO("The mean of the state errors is: " << mean_state_error.transpose() << ". This should be small.");
    INFO(percentage_x1 << "% of state 1 estimates exceed the 1\u03C3 boundary");
    INFO(percentage_x2 << "% of state 2 estimates exceed the 1\u03C3 boundary");
    INFO("The mean 1\u03C3 boundary for state 1 is [" << -mean_x1_boundary << ", " << mean_x1_boundary << "]");
    INFO("The mean 1\u03C3 boundary for state 2 is [" << -mean_x2_boundary << ", " << mean_x2_boundary << "]");

    REQUIRE(percentage_x1 <= 30.0);
}

TEST_CASE("Test the KalmanFilter", "[utility][math][filter][KalmanFilter]") {

    // Load in the test data
    const YAML::Node config                         = YAML::LoadFile("tests/TestKalman.yaml");
    const std::vector<double> true_angle            = resolve_expression<double>(config["true_angle"]);
    const std::vector<Eigen::Vector2d> measurements = resolve_expression<Eigen::Vector2d>(config["measurements"]);

    // Define the model
    const size_t n_states       = 3;
    const size_t n_inputs       = 0;
    const size_t n_measurements = 2;

    // Define the process model
    Eigen::Matrix3d A = config["kalman"]["A"].as<Expression>();

    // Define the input model
    Eigen::MatrixXd B = Eigen::Matrix<double, n_states, n_inputs>::Zero();

    // Define the measurement model
    Eigen::MatrixXd C = config["kalman"]["C"].as<Expression>();

    // Define the process noise covariance
    Eigen::Matrix3d Q = config["kalman"]["Q"].as<Expression>();

    // Define the measurement noise covariance
    Eigen::Matrix2d R = config["kalman"]["R"].as<Expression>();

    // Define the initial state
    Eigen::Matrix<double, n_states, 1> x0 = Eigen::Matrix<double, n_states, 1>::Zero();

    // Define the initial covariance
    Eigen::Matrix<double, n_states, n_states> P0 = Eigen::Matrix<double, n_states, n_states>::Identity();

    // Create a Kalman filter
    utility::math::filter::KalmanFilter<double, n_states, n_inputs, n_measurements> kf(x0, P0, A, B, C, Q, R);

    // Define the time step
    const double dt = 0.005;
    int N           = 1000;

    // Run the filter
    double total_error = 0.0;
    for (int i = 0; i < N; ++i) {
        // No control input
        Eigen::Matrix<double, n_inputs, 1> u = Eigen::Matrix<double, n_inputs, 1>::Zero();

        // Get the measurement
        Eigen::Matrix<double, n_measurements, 1> y = measurements[i];

        // Time update
        kf.time(u, dt);

        // Measurement update
        kf.measure(y);

        // Get the state
        Eigen::Matrix<double, n_states, 1> x_i = kf.get_state();

        // Compute the error between ground truth and the filter
        double error = std::abs(true_angle[i] - x_i(1));
        total_error += error;
    }
    double average_error = total_error / N;

    INFO("The total error is: " << total_error);
    INFO("The average error is: " << average_error);
    INFO("The final state is: " << kf.get_state().transpose());

    REQUIRE(average_error < 1e-2);
}
