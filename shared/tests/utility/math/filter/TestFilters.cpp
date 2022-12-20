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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
// #include <boost/algorithm/string.hpp>
#include <catch.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "VanDerPolModel.hpp"

#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/math/filter/inekf/InEKF.hpp"
#include "utility/strutil/strutil.hpp"
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
        innovations.push_back(measurement.x() - model_filter.get().x());
        actual_state.push_back(std::make_pair(model_filter.get(), model_filter.getCovariance()));
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
        innovations.push_back(measurement.x() - model_filter.getMean().x());
        actual_state.push_back(std::make_pair(model_filter.getMean(), model_filter.getCovariance()));
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

#define DT_MIN 1e-6
#define DT_MAX 1
using namespace std::chrono;

TEST_CASE("Test the InEKF", "[utility][math][filter][InEKF]") {
    // Get test data from a yaml file
    const YAML::Node config = YAML::LoadFile("tests/InEKFMeasurements.yaml");

    auto start = high_resolution_clock::now();

    //  ---- Initialize invariant extended Kalman filter ----- //
    utility::math::filter::inekf::RobotState initial_state;

    // Set initial state values
    initial_state.set_rotation(config["initial_state"]["orientation"].as<Expression>());
    initial_state.set_velocity(config["initial_state"]["velocity"].as<Expression>());
    initial_state.set_position(config["initial_state"]["position"].as<Expression>());
    initial_state.set_gyroscope_bias(config["initial_state"]["gyro_bias"].as<Expression>());
    initial_state.set_accelerometer_bias(config["initial_state"]["acc_bias"].as<Expression>());

    // Initialize state covariance
    utility::math::filter::inekf::NoiseParams noise_params;
    noise_params.set_gyroscope_noise(config["noise"]["gyro"].as<double>());
    noise_params.set_accelerometer_noise(config["noise"]["acc"].as<double>());
    noise_params.set_gyroscope_bias_noise(config["noise"]["gyro_bias"].as<double>());
    noise_params.set_accelerometer_bias_noise(config["noise"]["acc_bias"].as<double>());
    noise_params.set_contact_noise(config["noise"]["contact"].as<double>());

    // Initialize filter
    utility::math::filter::inekf::InEKF filter(initial_state, noise_params);
    INFO("Robot's state is initialized to:");
    INFO(filter.get_state());

    // Get data from yaml file
    const std::vector<double> times = resolve_expression<double>(config["measurements"]["time"]);
    const std::vector<Eigen::Vector3d> gyroscopes =
        resolve_expression<Eigen::Vector3d>(config["measurements"]["gyroscope"]);
    const std::vector<Eigen::Vector3d> accelerometers =
        resolve_expression<Eigen::Vector3d>(config["measurements"]["accelerometer"]);
    const std::vector<Eigen::Vector2i> force_sensors =
        resolve_expression<Eigen::Vector2i>(config["measurements"]["force_sensors"]);
    const std::vector<Eigen::Vector4d> quaternions =
        resolve_expression<Eigen::Vector4d>(config["measurements"]["kinematics_quaternion"]);
    const std::vector<Eigen::Vector3d> positions =
        resolve_expression<Eigen::Vector3d>(config["measurements"]["kinematics_position"]);
    const std::vector<Eigen::Matrix<double, 6, 6>> covariances =
        resolve_expression<Eigen::Matrix<double, 6, 6>>(config["measurements"]["covariance"]);

    // Should be able to remove these later
    Eigen::Matrix<double, 6, 1> imu_measurement      = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> imu_measurement_prev = Eigen::Matrix<double, 6, 1>::Zero();

    double t      = 0.0;
    double t_prev = 0.0;

    // Use each set of measurements in the filter
    for (long unsigned int i = 0; i < times.size(); i++) {
        // IMU DATA
        // Get the time difference between this run and the previous run
        double dt = i == 0 ? times[i] : times[i] - times[i - 1];
        // Check if delta time is within the bounds
        if (dt > DT_MIN && dt < DT_MAX) {
            if (i == 0) {
                filter.propagate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
            }
            else {
                filter.propagate(gyroscopes[i - 1], accelerometers[i - 1], dt);
            }
        }

        // CONTACT DATA
        Eigen::Vector2i force_sensor               = force_sensors[i];
        std::vector<std::pair<int, bool>> contacts = {{0, force_sensor[0]}, {1, force_sensor[1]}};
        filter.set_contacts(contacts);

        // KINEMATICS DATA
        // There are two sets, one for each leg
        utility::math::filter::inekf::kinematics measured_kinematics;

        Eigen::Isometry3d pose_right;
        pose_right.linear() = Eigen::Quaternion<double>(quaternions[i * 2][0],
                                                        quaternions[i * 2][1],
                                                        quaternions[i * 2][2],
                                                        quaternions[i * 2][3])
                                  .toRotationMatrix();
        pose_right.translation() = positions[i * 2];
        measured_kinematics.emplace_back(
            utility::math::filter::inekf::KinematicPose{0, pose_right.matrix(), covariances[i * 2]});

        Eigen::Isometry3d pose_left;
        pose_left.linear() = Eigen::Quaternion<double>(quaternions[(i * 2) + 1][0],
                                                       quaternions[(i * 2) + 1][1],
                                                       quaternions[(i * 2) + 1][2],
                                                       quaternions[(i * 2) + 1][3])
                                 .toRotationMatrix();
        pose_left.translation() = positions[(i * 2) + 1];
        measured_kinematics.emplace_back(
            utility::math::filter::inekf::KinematicPose{1, pose_left.matrix(), covariances[(i * 2) + 1]});

        // Correct state using kinematic measurements
        filter.correct_kinematics(measured_kinematics);
    }

    // Print final state
    INFO("Amount of sensor data points : " << times.size());
    INFO(filter.get_state());

    // Get time it took to compute filter with all the data
    auto stop     = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // Print the time and average time
    INFO("Total time [microseconds] : " << duration.count());
    INFO("Total time average [microseconds] : " << duration.count() / times.size());

    REQUIRE(10 == 0);
}
