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


#include "module/platform/darwin/SensorFilter/src/SensorFilter.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <catch.hpp>
#include <nuclear>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "module/platform/darwin/SensorFilter/src/MotionModel.h"

#include "message/input/Sensors.h"
#include "message/motion/BodySide.h"

#include "utility/input/ServoID.h"
#include "utility/math/filter/eigen/UKF.h"
#include "utility/platform/darwin/SensorFilter.h"
#include "utility/support/yaml_expression.h"

TEST_CASE("Testing the motion model", "[hardware][sensor filter][motion model]") {
    using message::motion::BodySide;

    using module::platform::darwin::MotionModel;
    using module::platform::darwin::MeasurementType::ACCELEROMETER;
    using module::platform::darwin::MeasurementType::GYROSCOPE;

    using utility::input::ServoID;
    using utility::support::Expression;

    // Constants
    // TODO: These are taken from the SesorFilter config file, we should load the file to get them
    const double foot_down_certainty = 0.01;

    const Eigen::Vector3d mean_position(0.0, 0.0, 0.49);
    const Eigen::Vector3d mean_velocity(0.0, 0.0, 0.0);
    const Eigen::Vector4d mean_rotation(0.0, 0.0, 0.0, 1.0);
    const Eigen::Vector3d mean_rotational_velocity(0.0, 0.0, 0.0);
    const Eigen::Vector3d mean_gyroscope_bias(0.0, 0.0, 0.0);

    const Eigen::Vector3d covariance_position(1e-3, 1e-3, 1.0);
    const Eigen::Vector3d covariance_velocity(1e-3, 1e-3, 1e-3);
    const Eigen::Vector4d covariance_rotation(0.1, 0.1, 0.1, 0.1);
    const Eigen::Vector3d covariance_rotational_velocity(0.1, 0.1, 0.1);
    const Eigen::Vector3d covariance_gyroscope_bias(1.0, 1.0, 1.0);

    const Eigen::Matrix3d accelerometer_measurement_noise(Eigen::Vector3d(3e-4, 3e-4, 3e-4).asDiagonal());
    const Eigen::Matrix3d accelerometer_magnitude_measurement_noise(Eigen::Vector3d(1e-4, 1e-4, 1e-4).asDiagonal());
    const Eigen::Matrix3d gyroscope_measurement_noise(Eigen::Vector3d(1e-8, 1e-8, 1e-8).asDiagonal());
    const Eigen::Matrix3d flat_foot_odometry_measurement_noise(Eigen::Vector3d(5e-8, 5e-8, 5e-8).asDiagonal());
    const Eigen::Matrix4d flat_foot_orientation_measurement_noise(Eigen::Vector4d(5e-6, 5e-6, 5e-6, 5e-6).asDiagonal());

    const Eigen::Vector3d position_process_noise(1e-10, 1e-10, 1e-10);
    const Eigen::Vector3d velocity_process_noise(1e-6, 1e-6, 1e-6);
    const Eigen::Vector4d rotation_process_noise(1e-12, 1e-12, 1e-12, 1e-12);
    const Eigen::Vector3d rotational_velocity_process_noise(1e-8, 1e-8, 1e-8);
    const Eigen::Vector3d gyroscope_bias_process_noise(1e-12, 1e-12, 1e-12);

    MotionModel<double>::StateVec process_noise;
    process_noise.segment<3>(MotionModel<double>::PX) = position_process_noise;
    process_noise.segment<3>(MotionModel<double>::VX) = velocity_process_noise;
    process_noise.segment<4>(MotionModel<double>::QX) = rotation_process_noise;
    process_noise.segment<3>(MotionModel<double>::WX) = rotational_velocity_process_noise;
    process_noise.segment<3>(MotionModel<double>::BX) = gyroscope_bias_process_noise;

    MotionModel<double>::StateVec mean;
    mean.segment<3>(MotionModel<double>::PX) = mean_position;
    mean.segment<3>(MotionModel<double>::VX) = mean_velocity;
    mean.segment<4>(MotionModel<double>::QX) = mean_rotation;
    mean.segment<3>(MotionModel<double>::WX) = mean_rotational_velocity;
    mean.segment<3>(MotionModel<double>::BX) = mean_gyroscope_bias;

    MotionModel<double>::StateVec covariance;
    covariance.segment<3>(MotionModel<double>::PX) = covariance_position;
    covariance.segment<3>(MotionModel<double>::VX) = covariance_velocity;
    covariance.segment<4>(MotionModel<double>::QX) = covariance_rotation;
    covariance.segment<3>(MotionModel<double>::WX) = covariance_rotational_velocity;
    covariance.segment<3>(MotionModel<double>::BX) = covariance_gyroscope_bias;

    // Load test data
    YAML::Node data = YAML::LoadFile("tests/motion_filter.yaml");

    std::vector<Eigen::Vector3d> raw_gyroscope;
    std::vector<Eigen::Vector3d> filtered_accelerometer;
    std::vector<Eigen::Vector3d> raw_accelerometer;
    std::vector<Eigen::Vector3d> filtered_gyroscope;
    std::vector<Eigen::Vector3d> CoM;
    std::vector<Eigen::Affine3d> Htw;
    std::vector<Eigen::Affine3d> Htw_filtered;
    std::vector<Eigen::Affine3d> HtRSp;
    std::vector<Eigen::Affine3d> HtLSp;
    std::vector<Eigen::Affine3d> HtRSr;
    std::vector<Eigen::Affine3d> HtLSr;
    std::vector<Eigen::Affine3d> HtRE;
    std::vector<Eigen::Affine3d> HtLE;
    std::vector<Eigen::Affine3d> HtRHy;
    std::vector<Eigen::Affine3d> HtLHy;
    std::vector<Eigen::Affine3d> HtRHp;
    std::vector<Eigen::Affine3d> HtLHp;
    std::vector<Eigen::Affine3d> HtRHr;
    std::vector<Eigen::Affine3d> HtLHr;
    std::vector<Eigen::Affine3d> HtRK;
    std::vector<Eigen::Affine3d> HtLK;
    std::vector<Eigen::Affine3d> HtRAp;
    std::vector<Eigen::Affine3d> HtLAp;
    std::vector<Eigen::Affine3d> HtRAr;
    std::vector<Eigen::Affine3d> HtLAr;
    std::vector<Eigen::Affine3d> HtHy;
    std::vector<Eigen::Affine3d> HtHp;

    std::vector<double> time_step = data["time"].as<std::vector<double>>();

    for (int i = 0; i < int(time_step.size()); ++i) {
        // TODO: IMU only has 2 axes .... need to get better data
        raw_gyroscope.emplace_back(data["raw_gyroscope"]["x"][i].as<double>(),
                                   data["raw_gyroscope"]["y"][i].as<double>(),
                                   data["raw_gyroscope"]["z"][i].as<double>());
        raw_accelerometer.emplace_back(data["raw_accelerometer"]["x"][i].as<double>(),
                                       data["raw_accelerometer"]["y"][i].as<double>(),
                                       data["raw_accelerometer"]["z"][i].as<double>());
        filtered_gyroscope.emplace_back(data["filtered_gyroscope"]["x"][i].as<double>(),
                                        data["filtered_gyroscope"]["y"][i].as<double>(),
                                        data["filtered_gyroscope"]["z"][i].as<double>());
        filtered_accelerometer.emplace_back(data["filtered_accelerometer"]["x"][i].as<double>(),
                                            data["filtered_accelerometer"]["y"][i].as<double>(),
                                            data["filtered_accelerometer"]["z"][i].as<double>());
        CoM.emplace_back(data["CoM"]["x"][i].as<double>(),
                         data["CoM"]["y"][i].as<double>(),
                         data["CoM"]["z"][i].as<double>());
        Htw.emplace_back(Eigen::Matrix4d(data["Htw"].as<Expression>()));
        HtHp.emplace_back(Eigen::Matrix4d(data["HtHp"].as<Expression>()));
        HtLAr.emplace_back(Eigen::Matrix4d(data["HtLAr"].as<Expression>()));
        HtLSp.emplace_back(Eigen::Matrix4d(data["HtLSp"].as<Expression>()));
        HtLSr.emplace_back(Eigen::Matrix4d(data["HtLSr"].as<Expression>()));
        HtLK.emplace_back(Eigen::Matrix4d(data["HtLK"].as<Expression>()));
        HtLE.emplace_back(Eigen::Matrix4d(data["HtLE"].as<Expression>()));
        HtRHy.emplace_back(Eigen::Matrix4d(data["HtRHy"].as<Expression>()));
        HtRHr.emplace_back(Eigen::Matrix4d(data["HtRHr"].as<Expression>()));
        HtRHp.emplace_back(Eigen::Matrix4d(data["HtRHp"].as<Expression>()));
        HtRE.emplace_back(Eigen::Matrix4d(data["HtRE"].as<Expression>()));
        HtRSr.emplace_back(Eigen::Matrix4d(data["HtRSr"].as<Expression>()));
        HtLHp.emplace_back(Eigen::Matrix4d(data["HtLHp"].as<Expression>()));
        HtRSp.emplace_back(Eigen::Matrix4d(data["HtRSp"].as<Expression>()));
        HtLHr.emplace_back(Eigen::Matrix4d(data["HtLHr"].as<Expression>()));
        HtLHy.emplace_back(Eigen::Matrix4d(data["HtLHy"].as<Expression>()));
        HtRK.emplace_back(Eigen::Matrix4d(data["HtRK"].as<Expression>()));
        HtRAr.emplace_back(Eigen::Matrix4d(data["HtRAr"].as<Expression>()));
        HtHy.emplace_back(Eigen::Matrix4d(data["HtHy"].as<Expression>()));
        HtRAp.emplace_back(Eigen::Matrix4d(data["HtRAp"].as<Expression>()));
        HtLAp.emplace_back(Eigen::Matrix4d(data["HtLAp"].as<Expression>()));
    }

    std::array<bool, 2> previous_foot_down = {false, false};
    std::array<Eigen::Affine3d, 2> footlanding_Hwf;

    utility::math::filter::UKF<double, MotionModel> filter;
    filter.model.process_noise = process_noise;
    filter.set_state(mean, covariance.asDiagonal());

    // The IMU has a different sampling rate to the servos
    int imu_step = raw_gyroscope.size() / time_step.size();

    // Step through the data and feed into the motion model
    for (int i = 0, j = 0; i < int(time_step.size()); ++i, j += imu_step) {
        // Create sensors message
        message::input::Sensors sensors;

        j                     = std::min(j, int(raw_accelerometer.size()) - 1);
        sensors.accelerometer = raw_accelerometer[j];
        sensors.gyroscope     = raw_gyroscope[j];

        sensors.Htx[ServoID::R_SHOULDER_PITCH] = HtRSp[i].matrix();
        sensors.Htx[ServoID::L_SHOULDER_PITCH] = HtLSp[i].matrix();
        sensors.Htx[ServoID::R_SHOULDER_ROLL]  = HtRSr[i].matrix();
        sensors.Htx[ServoID::L_SHOULDER_ROLL]  = HtLSr[i].matrix();
        sensors.Htx[ServoID::R_ELBOW]          = HtRE[i].matrix();
        sensors.Htx[ServoID::L_ELBOW]          = HtLE[i].matrix();
        sensors.Htx[ServoID::R_HIP_YAW]        = HtRHy[i].matrix();
        sensors.Htx[ServoID::L_HIP_YAW]        = HtLHy[i].matrix();
        sensors.Htx[ServoID::R_HIP_ROLL]       = HtRHr[i].matrix();
        sensors.Htx[ServoID::L_HIP_ROLL]       = HtLHr[i].matrix();
        sensors.Htx[ServoID::R_HIP_PITCH]      = HtRHp[i].matrix();
        sensors.Htx[ServoID::L_HIP_PITCH]      = HtLHp[i].matrix();
        sensors.Htx[ServoID::R_KNEE]           = HtRK[i].matrix();
        sensors.Htx[ServoID::L_KNEE]           = HtLK[i].matrix();
        sensors.Htx[ServoID::R_ANKLE_PITCH]    = HtRAp[i].matrix();
        sensors.Htx[ServoID::L_ANKLE_PITCH]    = HtLAp[i].matrix();
        sensors.Htx[ServoID::R_ANKLE_ROLL]     = HtRAr[i].matrix();
        sensors.Htx[ServoID::L_ANKLE_ROLL]     = HtLAr[i].matrix();
        sensors.Htx[ServoID::HEAD_YAW]         = HtHy[i].matrix();
        sensors.Htx[ServoID::HEAD_PITCH]       = HtHp[i].matrix();

        sensors.feet.resize(2);
        sensors.feet[BodySide::RIGHT].down = true;
        sensors.feet[BodySide::LEFT].down  = true;

        std::array<bool, 2> feet_down = {true};
        feet_down                     = utility::platform::darwin::calculate_foot_down(sensors, foot_down_certainty);

        sensors.feet[BodySide::RIGHT].down = feet_down[BodySide::RIGHT];
        sensors.feet[BodySide::LEFT].down  = feet_down[BodySide::LEFT];

        // Gyroscope measurement update
        filter.measure(sensors.gyroscope, gyroscope_measurement_noise, GYROSCOPE());

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise = accelerometer_measurement_noise
                                    + ((sensors.accelerometer.norm() - std::abs(module::platform::darwin::G))
                                       * (sensors.accelerometer.norm() - std::abs(module::platform::darwin::G)))
                                          * accelerometer_magnitude_measurement_noise;

        // Accelerometer measurement update
        filter.measure(sensors.accelerometer, acc_noise, ACCELEROMETER());

        for (auto& side : {BodySide::LEFT, BodySide::RIGHT}) {
            Eigen::Affine3d Htf(sensors.Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);
            Eigen::Affine3d Hwf = footlanding_Hwf[side];
            utility::platform::darwin::update_flat_foot(filter,
                                                        sensors.feet[side].down,
                                                        previous_foot_down[side],
                                                        flat_foot_odometry_measurement_noise,
                                                        flat_foot_orientation_measurement_noise,
                                                        Htf,
                                                        Hwf);

            footlanding_Hwf[side]  = Hwf;
            sensors.feet[side].Hwf = Hwf.matrix();
        }

        // Calculate our time offset from the last read
        double deltaT = std::max(time_step[i] - (i > 0 ? time_step[i - 1] : time_step[i]), 0.0);

        // Time update
        filter.time(deltaT);

        // Gives us the quaternion representation
        const auto& o = filter.get();

        // Map from world to torso coordinates (Rtw)
        Eigen::Affine3d Hwt;
        Hwt.linear()      = Eigen::Quaterniond(o.segment<4>(MotionModel<double>::QX)).toRotationMatrix();
        Hwt.translation() = Eigen::Vector3d(o.segment<3>(MotionModel<double>::PX));
        Htw_filtered.emplace_back(Hwt.inverse());
    }

    // Calculate the error
    // https://math.stackexchange.com/questions/1411657/measure-change-similarity-between-two-affine-transformations
    double error = 0.0;
    for (int i = 0; i < int(time_step.size()); ++i) {
        // Calculate the relative transformation between the two transforms
        // If they are equal this will be an identity matrix
        Eigen::Affine3d Hww = Htw[i].inverse() * Htw_filtered[i];

        // https://en.wikipedia.org/wiki/Matrix_norm#Special_cases
        // ||A||_2 = sigma_max(A)
        // where sigma_max(A) is the largest singular value of A
        Eigen::JacobiSVD<Eigen::Matrix4d> svd = Hww.matrix().jacobiSvd();
        double norm                           = svd.singularValues().x();

        error += std::abs(norm - 1.0);
    }

    // Determine the mean error
    error /= time_step.size();

    INFO("The mean of the transform error is: " << error << ". This should be small.");
    REQUIRE(error <= 1.0);
}
