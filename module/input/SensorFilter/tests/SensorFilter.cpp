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
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <yaml-cpp/yaml.h>

#include "MotionModel.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/math/quaternion.hpp"
#include "utility/support/yaml_expression.hpp"

using Catch::Matchers::WithinAbs;
using module::input::MotionModel;
using utility::math::filter::UKF;
using utility::support::Expression;

TEST_CASE("Test MotionModel Orientation", "[module][input][SensorFilter][MotionModel][orientation]") {
    // Create our motion model
    UKF<double, MotionModel> filter{};

    // Read in test data and ground truths
    std::vector<Eigen::Vector3d> gyro_readings{};
    std::vector<Eigen::Vector3d> acc_readings{};
    std::vector<Eigen::Quaterniond> quaternions{};

    char comma;
    std::ifstream ifs("tests/webots.csv");
    while (ifs.good()) {
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
        Eigen::Quaterniond quat;
        ifs >> gyro.x() >> comma >> gyro.y() >> comma >> gyro.z() >> comma >> acc.x() >> comma >> acc.y() >> comma
            >> acc.z();
        if (ifs.good()) {
            gyro_readings.emplace_back(gyro);
            acc_readings.emplace_back(acc);
            quaternions.emplace_back(Eigen::Quaterniond(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)));
        }
    }
    ifs.close();

    if (gyro_readings.size() == 0 || acc_readings.size() == 0 || quaternions.size() == 0) {
        FAIL("No data to test on");
    }

    // Configure the motion model
    YAML::Node config = YAML::LoadFile("config/SensorFilter.yaml");

    // Update our velocity timestep decay
    filter.model.timeUpdateVelocityDecay = config["ukf"]["update"]["velocity_decay"].as<Expression>();

    // Set our process noise in our filter
    MotionModel<double>::StateVec process_noise{};
    const auto& process        = config["ukf"]["noise"]["process"];
    process_noise.rTWw         = process["position"].as<Expression>();
    process_noise.vTw          = process["velocity"].as<Expression>();
    process_noise.Rwt          = Eigen::Vector4d(process["rotation"].as<Expression>());
    process_noise.omegaTTt     = process["rotational_velocity"].as<Expression>();
    filter.model.process_noise = process_noise;

    // Set our initial mean and covariance
    MotionModel<double>::StateVec mean{};
    MotionModel<double>::StateVec covariance{};
    const auto& initial = config["ukf"]["initial"];
    mean.rTWw           = initial["mean"]["position"].as<Expression>();
    mean.vTw            = initial["mean"]["velocity"].as<Expression>();
    mean.Rwt            = Eigen::Vector4d(initial["mean"]["rotation"].as<Expression>());
    mean.omegaTTt       = initial["mean"]["rotational_velocity"].as<Expression>();
    covariance.rTWw     = initial["covariance"]["position"].as<Expression>();
    covariance.vTw      = initial["covariance"]["velocity"].as<Expression>();
    covariance.Rwt      = Eigen::Vector4d(initial["covariance"]["rotation"].as<Expression>());
    covariance.omegaTTt = initial["covariance"]["rotational_velocity"].as<Expression>();

    bool failed = false;
    std::string error_msg;
    switch (filter.set_state(mean.getStateVec(), covariance.asDiagonal())) {
        case Eigen::Success: break;
        case Eigen::NumericalIssue:
            error_msg = "Cholesky decomposition failed. The provided data did not satisfy the prerequisites.";
            failed    = true;
            break;
        case Eigen::NoConvergence:
            error_msg = "Cholesky decomposition failed. Iterative procedure did not converge.";
            failed    = true;
            break;
        case Eigen::InvalidInput:
            error_msg =
                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been improperly called. "
                "When assertions are enabled, such errors trigger an assert.";
            failed = true;
            break;
        default:
            error_msg = "Cholesky decomposition failed. Some other reason.";
            failed    = true;
            break;
    }

    if (failed) {
        // Failed to initialise UKF
        const double covariance_sigma_weight = 0.1 * 0.1 * MotionModel<double>::size;
        const MotionModel<double>::StateMat state(
            covariance_sigma_weight * filter.get_covariance().unaryExpr([](const double& c) { return std::abs(c); }));
        INFO(state.diagonal());

        INFO(error_msg);
        FAIL("Failed to initialise UKF. Aborting");
    }

    // Noise to be applied to gyroscope measurements
    Eigen::Matrix3d gyroscope_noise =
        Eigen::Vector3d(config["ukf"]["noise"]["measurement"]["gyroscope"].as<Expression>()).asDiagonal();

    // Noise to be applied to accelerometer measurements
    Eigen::Matrix3d accelerometer_noise =
        Eigen::Vector3d(config["ukf"]["noise"]["measurement"]["accelerometer"].as<Expression>()).asDiagonal();
    Eigen::Matrix3d accelerometer_magnitude_noise =
        Eigen::Vector3d(config["ukf"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>()).asDiagonal();

    // Elapsed time between each sensor read
    static constexpr double deltaT = 1.0 / 90.0;

    // Set up for adding gaussian noise to the measurements
    // Gyroscope datasheet says the MEMS device has a 0.03 dps/sqrt(Hz) noise density with a bandwidth of 50Hz
    // Accelerometer datasheet says the MEMS device has a 220 micro-g/sqrt(Hz) noise density with a bandwidth 400Hz
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> gyro_sensor_noise{0.0, 0.03 * std::sqrt(50.0) * M_PI / 180.0};
    std::normal_distribution<> acc_sensor_noise{0.0, 22e-6 * std::sqrt(400) * module::input::G};

    // Vector of quaternion errors from each timestep
    std::vector<Eigen::Quaterniond> errors{};
    std::vector<double> angular_errors{};

    // Step through test data and get orientation predictions
    for (int i = 0; i < int(quaternions.size()); ++i) {
        // Perform the measurement update
        filter.measure(gyro_readings[i], gyroscope_noise, module::input::MeasurementType::GYROSCOPE());

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise = accelerometer_noise
                                    + ((acc_readings[i].norm() - std::abs(module::input::G))
                                       * (acc_readings[i].norm() - std::abs(module::input::G)))
                                          * accelerometer_magnitude_noise;

        // Accelerometer measurement update
        filter.measure(acc_readings[i], acc_noise, module::input::MeasurementType::ACCELEROMETER());

        // Time update
        INFO("Running time update for step " << i);
        INFO("Gyroscope Measurement....: " << gyro_readings[i].transpose());
        INFO("Accelerometer Measurement: " << acc_readings[i].transpose() << " (" << acc_readings[i].norm() << ")");
        INFO("Expected Orientation.....: " << quaternions[i].coeffs().transpose());
        bool failed = false;
        std::string error_msg;
        switch (filter.time(deltaT)) {
            case Eigen::Success: break;
            case Eigen::NumericalIssue:
                error_msg = "Cholesky decomposition failed. The provided data did not satisfy the prerequisites.";
                failed    = true;
                break;
            case Eigen::NoConvergence:
                error_msg = "Cholesky decomposition failed. Iterative procedure did not converge.";
                failed    = true;
                break;
            case Eigen::InvalidInput:
                error_msg =
                    "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                    "improperly called. When assertions are enabled, such errors trigger an assert.";
                failed = true;
                break;
            default:
                error_msg = "Cholesky decomposition failed. Some other reason.";
                failed    = true;
                break;
        }

        if (!failed) {
            // Calculate difference between expected and predicted orientations
            Eigen::Quaterniond Rwt = MotionModel<double>::StateVec(filter.get_state()).Rwt;
            INFO("Predicted Orientation....: " << Rwt.coeffs().transpose());

            angular_errors.emplace_back(Rwt.angularDistance(quaternions[i]));
            errors.emplace_back(utility::math::quaternion::difference(Rwt, quaternions[i]));
        }
        else {
            // UKF state unrecoverable. Print current average error and bail
            Eigen::Quaterniond Rwt             = MotionModel<double>::StateVec(filter.get_state()).Rwt;
            const double current_angular_error = Rwt.angularDistance(quaternions[i]);

            angular_errors.emplace_back(current_angular_error);
            errors.emplace_back(utility::math::quaternion::difference(Rwt, quaternions[i]));

            const Eigen::Quaterniond mean_error =
                utility::math::quaternion::meanRotation(errors.begin(), errors.end()).normalized();

            const double mean_angular_error =
                std::accumulate(angular_errors.begin(), angular_errors.end(), 0.0) / double(angular_errors.size());

            INFO("Predicted Orientation....: " << Rwt.coeffs().transpose());
            INFO("Current Angular Error....: " << current_angular_error);
            INFO("Mean Error........: " << mean_error.coeffs().transpose());
            INFO("Mean Angular Error: " << mean_angular_error);

            const double covariance_sigma_weight = 0.1 * 0.1 * MotionModel<double>::size;
            const MotionModel<double>::StateMat state(
                covariance_sigma_weight
                * filter.get_covariance().unaryExpr([](const double& c) { return std::abs(c); }));
            INFO(state.diagonal());

            INFO(error_msg);
            FAIL("UKF State unrecoverable. Aborting");
        }
    }

    const Eigen::Quaterniond mean_error = utility::math::quaternion::meanRotation(errors.begin(), errors.end());

    const double mean_angular_error =
        std::accumulate(angular_errors.begin(), angular_errors.end(), 0.0) / double(angular_errors.size());
    INFO("Mean Error........: " << mean_error.coeffs().transpose());
    INFO("Mean Angular Error: " << mean_angular_error);
    REQUIRE_THAT(mean_error.w(), WithinAbs(1.0, 0.01));
    REQUIRE_THAT(mean_error.x(), WithinAbs(0.0, 0.01));
    REQUIRE_THAT(mean_error.y(), WithinAbs(0.0, 0.01));
    REQUIRE_THAT(mean_error.z(), WithinAbs(0.0, 0.01));
    REQUIRE_THAT(mean_angular_error, WithinAbs(0.0, 0.02));
}
