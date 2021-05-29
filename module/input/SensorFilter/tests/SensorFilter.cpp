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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <yaml-cpp/yaml.h>

#include "MotionModel.hpp"

#include "utility/math/filter/UKF.hpp"
#include "utility/math/quaternion.hpp"
#include "utility/support/yaml_expression.hpp"

void print_error(const int& line, const std::string& msg, const Eigen::Matrix<double, 16, 16>& covariance) {
    const double covariance_sigma_weight = 0.1 * 0.1 * 16;
    const Eigen::Matrix<double, 16, 16> state(covariance_sigma_weight
                                              * covariance.unaryExpr([](const double& c) { return std::abs(c); }));
    INFO(state);
    FAIL("Line " << line << ": " << msg << "\n");
}

TEST_CASE("Test MotionModel Orientation", "[module][input][SensorFilter][MotionModel][orientation]") {
    using module::input::MotionModel;
    using utility::math::filter::UKF;
    using utility::support::Expression;

    // Create our motion model
    UKF<double, MotionModel> filter;

    // Configure the motion model
    YAML::Node config = YAML::LoadFile("config/SensorFilter.yaml");

    // Update our velocity timestep decay
    filter.model.timeUpdateVelocityDecay = config["motion_filter"]["update"]["velocity_decay"].as<Expression>();

    // Set our process noise in our filter
    MotionModel<double>::StateVec process_noise;
    const auto& process         = config["motion_filter"]["noise"]["process"];
    process_noise.rTWw          = process["position"].as<Expression>();
    process_noise.vTw           = process["velocity"].as<Expression>();
    process_noise.Rwt           = Eigen::Vector4d(process["rotation"].as<Expression>());
    process_noise.omegaTTt      = process["rotational_velocity"].as<Expression>();
    process_noise.omegaTTt_bias = process["gyroscope_bias"].as<Expression>();
    filter.model.process_noise  = process_noise;

    // Set our initial mean and covariance
    MotionModel<double>::StateVec mean;
    MotionModel<double>::StateVec covariance;
    const auto& initial      = config["motion_filter"]["initial"];
    mean.rTWw                = initial["mean"]["position"].as<Expression>();
    mean.vTw                 = initial["mean"]["velocity"].as<Expression>();
    mean.Rwt                 = Eigen::Vector4d(initial["mean"]["rotation"].as<Expression>());
    mean.omegaTTt            = initial["mean"]["rotational_velocity"].as<Expression>();
    mean.omegaTTt_bias       = initial["mean"]["gyroscope_bias"].as<Expression>();
    covariance.rTWw          = initial["covariance"]["position"].as<Expression>();
    covariance.vTw           = initial["covariance"]["velocity"].as<Expression>();
    covariance.Rwt           = Eigen::Vector4d(initial["covariance"]["rotation"].as<Expression>());
    covariance.omegaTTt      = initial["covariance"]["rotational_velocity"].as<Expression>();
    covariance.omegaTTt_bias = initial["covariance"]["gyroscope_bias"].as<Expression>();

    switch (filter.set_state(mean.getStateVec(), covariance.asDiagonal())) {
        case Eigen::Success: break;
        case Eigen::NumericalIssue:
            print_error(__LINE__,
                        "Cholesky decomposition failed. The provided data did not satisfy the "
                        "prerequisites.",
                        filter.getCovariance());
            break;
        case Eigen::NoConvergence:
            print_error(__LINE__,
                        "Cholesky decomposition failed. Iterative procedure did not converge.",
                        filter.getCovariance());
            break;
        case Eigen::InvalidInput:
            print_error(__LINE__,
                        "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                        "improperly called. When assertions are enabled, such errors trigger an assert.",
                        filter.getCovariance());
            break;
        default:
            print_error(__LINE__, "Cholesky decomposition failed. Some other reason.", filter.getCovariance());
            break;
    }

    // Noise to be applied to gyroscope measurements
    Eigen::Matrix3d gyroscope_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<Expression>()).asDiagonal();

    // Elapsed time between each sensor read
    constexpr double deltaT = 1.0 / 90.0;

    // Set up for adding gaussian noise to the measurements
    // Gyroscope datasheet says the MEMS device has a 0.03 dps/sqrt(Hz) noise density with a bandwidth of 50Hz
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0.0, 0.03 * std::sqrt(50.0) * M_PI / 180.0};

    // Vector of quaternion errors from each timestep
    std::vector<Eigen::Quaterniond> errors;

    // Oscillate rotation around the x-axis
    // Oscillation should go 0 -> PI/2 -> 0 -> -PI/2 -> 0
    for (int i = 0; i <= 360; ++i) {
        // Calculate our next rotation and set it in Htw
        const double current_angle = M_PI_2 * std::sin(M_PI * double(i) / 180.0);
        Eigen::Quaterniond Rwt(Eigen::AngleAxisd(current_angle, Eigen::Vector3d::UnitX()));

        // Calculate our previous angle so we can determine gyroscope velocity
        const double previous_angle = M_PI_2 * std::sin(M_PI * double(i - 1) / 180.0);

        // We are rotating around the x-axis, so the other 2 axes are zero
        // const Eigen::Vector3d gyroscope((d(gen) + (current_angle - previous_angle)) / deltaT, 0.0, 0.0);
        const Eigen::Vector3d gyroscope((current_angle - previous_angle) / deltaT, 0.0, 0.0);

        // Perform the measurement update
        filter.measure(gyroscope, gyroscope_noise, module::input::MeasurementType::GYROSCOPE());
        // filter.measure(Eigen::Vector3d(0.0, 0.0, 0.0), gyroscope_noise, module::input::MeasurementType::GYROSCOPE());

        // Time update
        INFO("Running time update for step " << i);
        switch (filter.time(deltaT)) {
            case Eigen::Success:
                // Calculate difference between expected and predicted orientations
                errors.emplace_back(
                    utility::math::quaternion::difference(Rwt, MotionModel<double>::StateVec(filter.get()).Rwt));
                break;
            case Eigen::NumericalIssue:
                print_error(__LINE__,
                            "Cholesky decomposition failed. The provided data did not satisfy the "
                            "prerequisites.",
                            filter.getCovariance());
                break;
            case Eigen::NoConvergence:
                print_error(__LINE__,
                            "Cholesky decomposition failed. Iterative procedure did not converge.",
                            filter.getCovariance());
                break;
            case Eigen::InvalidInput:
                print_error(__LINE__,
                            "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                            "improperly called. When assertions are enabled, such errors trigger an assert.",
                            filter.getCovariance());
                break;
            default:
                print_error(__LINE__, "Cholesky decomposition failed. Some other reason.", filter.getCovariance());
                break;
        }
    }

    const Eigen::Quaterniond mean_error = utility::math::quaternion::mean(errors.begin(), errors.end());
    INFO("mean error: " << mean_error.coeffs().transpose());
    REQUIRE(mean_error.w() == Approx(1.0));
    REQUIRE(mean_error.x() == Approx(0.0));
    REQUIRE(mean_error.y() == Approx(0.0));
    REQUIRE(mean_error.z() == Approx(0.0));
}
