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

#include "utility/math/filter/InEKF.hpp"
#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/filter/UKF.hpp"
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
    auto start  = high_resolution_clock::now();
    int n_count = 0;
    //  ---- Initialize invariant extended Kalman filter ----- //
    inekf::RobotState initial_state;

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0,  // initial orientation
        0, -1, 0,   // IMU frame is rotated 90deg about the x-axis
        0, 0, -1;
    v0 << 0, 0, 0;   // initial velocity
    p0 << 0, 0, 0;   // initial position
    bg0 << 0, 0, 0;  // initial gyroscope bias
    ba0 << 0, 0, 0;  // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.01);

    // Initialize filter
    inekf::InEKF filter(initial_state, noise_params);
    std::cout << "Noise parameters are initialized to: \n";
    std::cout << filter.getNoiseParams() << std::endl;
    std::cout << "Robot's state is initialized to: \n";
    std::cout << filter.getState() << std::endl;

    // Open data file
    std::ifstream infile("/home/nubots/NUbots/shared/tests/data/imu_kinematic_measurements.txt");
    std::string line;
    Eigen::Matrix<double, 6, 1> imu_measurement      = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> imu_measurement_prev = Eigen::Matrix<double, 6, 1>::Zero();
    double t                                         = 0;
    double t_prev                                    = 0;

    // ---- Loop through data file and read in measurements line by line ---- //
    while (getline(infile, line)) {
        std::vector<std::string> measurement = utility::strutil::split(line, ' ');
        // boost::split(measurement, line, boost::is_any_of(" "));
        // // Handle measurements
        if (measurement[0].compare("IMU") == 0) {
            // cout << "Received IMU Data, propagating state\n";
            assert((measurement.size() - 2) == 6);
            t = atof(measurement[1].c_str());
            // Read in IMU data
            imu_measurement << std::stod(measurement[2]), std::stod(measurement[3]), std::stod(measurement[4]),
                std::stod(measurement[5]), std::stod(measurement[6]), std::stod(measurement[7]);

            // Propagate using IMU data
            double dt = t - t_prev;
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(imu_measurement_prev, dt);
            }
        }
        else if (measurement[0].compare("CONTACT") == 0) {
            // cout << "Received CONTACT Data, setting filter's contact state\n";
            assert((measurement.size() - 2) % 2 == 0);
            std::vector<std::pair<int, bool>> contacts;
            int id;
            bool indicator;
            t = std::stod(measurement[1]);
            // Read in contact data
            for (int i = 2; i < measurement.size(); i += 2) {
                id        = std::stoi(measurement[i]);
                indicator = bool(std::stod(measurement[i + 1]));
                contacts.push_back(std::pair<int, bool>(id, indicator));
            }
            // Set filter's contact state
            filter.setContacts(contacts);
        }
        else if (measurement[0].compare("KINEMATIC") == 0) {
            // cout << "Received KINEMATIC observation, correcting state\n";
            assert((measurement.size() - 2) % 44 == 0);
            int id;
            Eigen::Quaternion<double> q;
            Eigen::Vector3d p;
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            Eigen::Matrix<double, 6, 6> covariance;
            inekf::vectorKinematics measured_kinematics;
            t = std::stod(measurement[1]);
            // Read in kinematic data
            for (int i = 2; i < measurement.size(); i += 44) {
                id = std::stoi(measurement[i]);
                q  = Eigen::Quaternion<double>(std::stod(measurement[i + 1]),
                                              std::stod(measurement[i + 2]),
                                              std::stod(measurement[i + 3]),
                                              std::stod(measurement[i + 4]));
                q.normalize();
                p << std::stod(measurement[i + 5]), std::stod(measurement[i + 6]), std::stod(measurement[i + 7]);
                pose.block<3, 3>(0, 0) = q.toRotationMatrix();
                pose.block<3, 1>(0, 3) = p;
                for (int j = 0; j < 6; ++j) {
                    for (int k = 0; k < 6; ++k) {
                        covariance(j, k) = std::stod(measurement[i + 8 + j * 6 + k]);
                    }
                }
                inekf::Kinematics frame(id, pose, covariance);
                measured_kinematics.push_back(frame);
            }
            // Correct state using kinematic measurements
            filter.CorrectKinematics(measured_kinematics);
        }

        // Store previous timestamp
        t_prev               = t;
        imu_measurement_prev = imu_measurement;
        n_count++;
    }

    // Print final state
    std::cout << "n count : " << n_count << std::endl;
    std::cout << filter.getState() << std::endl;
    // After function call
    auto stop = high_resolution_clock::now();
    // Subtract stop and start timepoints and
    // cast it to required unit. Predefined units
    // are nanoseconds, microseconds, milliseconds,
    // seconds, minutes, hours. Use std::chrono::duration_cast()
    // function.
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // To get the value of duration use the count()
    // member function on the duration object
    std::cout << "Total time [microseconds] : " << duration.count() << std::endl;
    std::cout << "Total time average [microseconds] : " << duration.count() / n_count << std::endl;

    REQUIRE(10 == 0);
}
