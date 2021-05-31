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
#include <string>
#include <yaml-cpp/yaml.h>

#include "MotionModel.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/BodySide.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/math/quaternion.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/support/yaml_expression.hpp"

using module::input::MotionModel;
using utility::math::filter::UKF;
using utility::support::Expression;

message::motion::KinematicsModel construct_kinematics_model() {
    YAML::Node config = YAML::LoadFile("config/KinematicsConfiguration.yaml");

    message::motion::KinematicsModel model;

    const auto& objLeg            = config["leg"];
    Eigen::Vector3f leg_hipOffset = objLeg["hip_offset"].as<Expression>();
    model.leg.HIP_OFFSET_X        = leg_hipOffset.x();
    model.leg.HIP_OFFSET_Y        = leg_hipOffset.y();
    model.leg.HIP_OFFSET_Z        = leg_hipOffset.z();

    model.leg.UPPER_LEG_LENGTH = objLeg["upper_leg_length"].as<float>();
    model.leg.LOWER_LEG_LENGTH = objLeg["lower_leg_length"].as<float>();

    model.leg.HEEL_LENGTH = objLeg["heel_length"].as<float>();

    model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = objLeg["foot_centre_to_ankle_centre"].as<float>();

    const auto& objFoot   = objLeg["foot"];
    model.leg.FOOT_WIDTH  = objFoot["width"].as<float>();
    model.leg.FOOT_HEIGHT = objFoot["height"].as<float>();
    model.leg.FOOT_LENGTH = objFoot["length"].as<float>();
    model.leg.TOE_LENGTH  = objFoot["toe_length"].as<float>();

    model.leg.LENGTH_BETWEEN_LEGS = 2.0 * model.leg.HIP_OFFSET_Y;

    const auto& objLeftRight            = objLeg["left_to_right"];
    model.leg.LEFT_TO_RIGHT_HIP_YAW     = objLeftRight["hip_yaw"].as<int>();
    model.leg.LEFT_TO_RIGHT_HIP_ROLL    = objLeftRight["hip_roll"].as<int>();
    model.leg.LEFT_TO_RIGHT_HIP_PITCH   = objLeftRight["hip_pitch"].as<int>();
    model.leg.LEFT_TO_RIGHT_KNEE        = objLeftRight["knee"].as<int>();
    model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = objLeftRight["ankle_pitch"].as<int>();
    model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = objLeftRight["ankle_roll"].as<int>();

    // const auto& objHead = config["head"];
    // model.head.CAMERA_DECLINATION_ANGLE_OFFSET = objHead["camera_declination_angle_offset"].as<Expression>();
    // Eigen::Vector3f head_neckToCamera  = objHead["neck_to_camera"].as<Expression>();
    // model.head.NECK_TO_CAMERA_X        = head_neckToCamera.x();
    // model.head.NECK_TO_CAMERA_Y        = head_neckToCamera.y();
    // model.head.NECK_TO_CAMERA_Z        = head_neckToCamera.z();
    // model.head.INTERPUPILLARY_DISTANCE = objHead["ipd"].as<float>();

    // const auto& objNeck = objHead["neck"];
    // model.head.NECK_LENGTH = objNeck["length"].as<float>();

    // Eigen::Vector3f neck_basePositionFromOrigin = objNeck["base_position_from_origin"].as<Expression>();
    // model.head.NECK_BASE_POS_FROM_ORIGIN_X      = neck_basePositionFromOrigin.x();
    // model.head.NECK_BASE_POS_FROM_ORIGIN_Y      = neck_basePositionFromOrigin.y();
    // model.head.NECK_BASE_POS_FROM_ORIGIN_Z      = neck_basePositionFromOrigin.z();

    // auto& objHeadMovementLimits = objHead["limits"];
    // Eigen::Vector2f headMovementLimits_yaw   = objHeadMovementLimits["yaw"].as<Expression>();
    // Eigen::Vector2f headMovementLimits_pitch = objHeadMovementLimits["pitch"].as<Expression>();
    // model.head.MIN_YAW                       = headMovementLimits_yaw.x();
    // model.head.MAX_YAW                       = headMovementLimits_yaw.y();
    // model.head.MIN_PITCH                     = headMovementLimits_pitch.x();
    // model.head.MAX_PITCH                     = headMovementLimits_pitch.y();

    // const auto& objArm = config["arm"];
    // const auto& objShoulder = objArm["shoulder"];
    // const auto& objUpperArm = objArm["upper_arm"];
    // const auto& objLowerArm = objArm["lower_arm"];

    // model.arm.DISTANCE_BETWEEN_SHOULDERS = objArm["distance_between_shoulders"].as<float>();
    // Eigen::Vector2f shoulderOffset       = objShoulder["offset"].as<Expression>();
    // model.arm.SHOULDER_X_OFFSET          = shoulderOffset.x();
    // model.arm.SHOULDER_Z_OFFSET          = shoulderOffset.y();
    // model.arm.SHOULDER_LENGTH            = objShoulder["length"].as<float>();
    // model.arm.SHOULDER_WIDTH             = objShoulder["width"].as<float>();
    // model.arm.SHOULDER_HEIGHT            = objShoulder["height"].as<float>();

    // model.arm.UPPER_ARM_LENGTH     = objUpperArm["length"].as<float>();
    // Eigen::Vector2f upperArmOffset = objUpperArm["offset"].as<Expression>();
    // model.arm.UPPER_ARM_Y_OFFSET   = upperArmOffset.x();
    // model.arm.UPPER_ARM_X_OFFSET   = upperArmOffset.y();

    // model.arm.LOWER_ARM_LENGTH     = objLowerArm["length"].as<float>();
    // Eigen::Vector2f lowerArmOffset = objLowerArm["offset"].as<Expression>();
    // model.arm.LOWER_ARM_Y_OFFSET   = lowerArmOffset.x();
    // model.arm.LOWER_ARM_Z_OFFSET   = lowerArmOffset.y();

    // const auto& objMassModel = config["mass_model"];
    // model.mass_model.head        = objMassModel["particles"]["head"].as<Expression>();
    // model.mass_model.arm_upper   = objMassModel["particles"]["arm_upper"].as<Expression>();
    // model.mass_model.arm_lower   = objMassModel["particles"]["arm_lower"].as<Expression>();
    // model.mass_model.torso       = objMassModel["particles"]["torso"].as<Expression>();
    // model.mass_model.hip_block   = objMassModel["particles"]["hip_block"].as<Expression>();
    // model.mass_model.leg_upper   = objMassModel["particles"]["leg_upper"].as<Expression>();
    // model.mass_model.leg_lower   = objMassModel["particles"]["leg_lower"].as<Expression>();
    // model.mass_model.ankle_block = objMassModel["particles"]["ankle_block"].as<Expression>();
    // model.mass_model.foot        = objMassModel["particles"]["foot"].as<Expression>();

    // const auto& objTensorModel = config["tensor_model"];
    // model.tensor_model.head        = objTensorModel["particles"]["head"].as<Expression>();
    // model.tensor_model.arm_upper   = objTensorModel["particles"]["arm_upper"].as<Expression>();
    // model.tensor_model.arm_lower   = objTensorModel["particles"]["arm_lower"].as<Expression>();
    // model.tensor_model.torso       = objTensorModel["particles"]["torso"].as<Expression>();
    // model.tensor_model.hip_block   = objTensorModel["particles"]["hip_block"].as<Expression>();
    // model.tensor_model.leg_upper   = objTensorModel["particles"]["leg_upper"].as<Expression>();
    // model.tensor_model.leg_lower   = objTensorModel["particles"]["leg_lower"].as<Expression>();
    // model.tensor_model.ankle_block = objTensorModel["particles"]["ankle_block"].as<Expression>();
    // model.tensor_model.foot        = objTensorModel["particles"]["foot"].as<Expression>();

    return model;
}

void print_error(const int& line, const std::string& msg, const Eigen::Matrix<double, 16, 16>& covariance) {
    const double covariance_sigma_weight = 0.1 * 0.1 * 16;
    const Eigen::Matrix<double, 16, 16> state(covariance_sigma_weight
                                              * covariance.unaryExpr([](const double& c) { return std::abs(c); }));
    INFO(state);
    FAIL("Line " << line << ": " << msg << "\n");
}

TEST_CASE("Test MotionModel Orientation", "[module][input][SensorFilter][MotionModel][orientation]") {
    // Get our kinematics model
    message::motion::KinematicsModel model = construct_kinematics_model();

    // Construct a default sensors message so we can calculate kinematics for footdown
    message::input::Sensors sensors;
    for (int id = 0; id < 20; ++id) {
        sensors.servo.emplace_back(0, id, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

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

    // Noise to be applied to accelerometer measurements
    Eigen::Matrix3d accelerometer_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<Expression>()).asDiagonal();
    Eigen::Matrix3d accelerometer_magnitude_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>())
            .asDiagonal();

    // Noise to be applied to flat foot measurements
    Eigen::Matrix3d flat_foot_odometry_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<Expression>())
            .asDiagonal();
    Eigen::Matrix4d flat_foot_orientation_noise =
        Eigen::Vector4d(config["motion_filter"]["noise"]["measurement"]["flat_foot_orientation"].as<Expression>())
            .asDiagonal();

    // Elapsed time between each sensor read
    constexpr double deltaT = 1.0 / 90.0;

    // Set up for adding gaussian noise to the measurements
    // Gyroscope datasheet says the MEMS device has a 0.03 dps/sqrt(Hz) noise density with a bandwidth of 50Hz
    // Accelerometer datasheet says the MEMS device has a 220 micro-g/sqrt(Hz) noise density with a bandwidth 400Hz
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> gyro_sensor_noise{0.0, 0.03 * std::sqrt(50.0) * M_PI / 180.0};
    std::normal_distribution<> acc_sensor_noise{0.0, 22e-6 * std::sqrt(400) * module::input::G};

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
        const Eigen::Vector3d gyroscope((gyro_sensor_noise(gen) + (current_angle - previous_angle)) / deltaT, 0.0, 0.0);

        // Perform the measurement update
        filter.measure(gyroscope, gyroscope_noise, module::input::MeasurementType::GYROSCOPE());


        // ************************************


        const Eigen::Vector3d accelerometer = Rwt.toRotationMatrix().transpose().rightCols<1>() * module::input::G;

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise = accelerometer_noise
                                    + ((accelerometer.norm() - std::abs(module::input::G))
                                       * (accelerometer.norm() - std::abs(module::input::G)))
                                          * accelerometer_magnitude_noise;

        // Accelerometer measurement update
        filter.measure(accelerometer, acc_noise, module::input::MeasurementType::ACCELEROMETER());


        // ************************************


        Eigen::Affine3d Hwt;
        Hwt.linear()      = Rwt.toRotationMatrix();
        Hwt.translation() = mean.rTWw;

        // Set hip roll angles according to our current angle
        if (current_angle > 5.0 * M_PI / 180.0) {
            // Left foot is off the ground
            sensors.servo[utility::input::ServoID::L_HIP_ROLL].present_position = 0.0;
            sensors.servo[utility::input::ServoID::R_HIP_ROLL].present_position = current_angle;

            auto Htx = utility::motion::kinematics::calculateAllPositions(model, sensors);
            for (const auto& entry : Htx) {
                sensors.Htx[entry.first] = entry.second.matrix();
            }

            Eigen::Affine3d Htf(sensors.Htx[utility::input::ServoID::R_ANKLE_ROLL]);
            Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

            Eigen::Affine3d Hwf   = Hwt * Htg;
            Hwf.translation().z() = 0.0;

            // Use stored Hwf and Htf to calculate Hwt
            Eigen::Affine3d Hwt = Hwf * Htf.inverse();

            // Do a foot based position update
            filter.measure(Eigen::Vector3d(Hwt.translation()),
                           flat_foot_odometry_noise,
                           module::input::MeasurementType::FLAT_FOOT_ODOMETRY());

            // Do a foot based orientation update
            Eigen::Quaterniond foot_Rwt(Hwt.linear());
            filter.measure(foot_Rwt.coeffs(),
                           flat_foot_orientation_noise,
                           module::input::MeasurementType::FLAT_FOOT_ORIENTATION());
        }
        else if (current_angle < -5.0 * M_PI / 180.0) {
            // Right foot is off the ground
            sensors.servo[utility::input::ServoID::L_HIP_ROLL].present_position = current_angle;
            sensors.servo[utility::input::ServoID::R_HIP_ROLL].present_position = 0.0;

            auto Htx = utility::motion::kinematics::calculateAllPositions(model, sensors);
            for (const auto& entry : Htx) {
                sensors.Htx[entry.first] = entry.second.matrix();
            }

            Eigen::Affine3d Htf(sensors.Htx[utility::input::ServoID::L_ANKLE_ROLL]);
            Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

            Eigen::Affine3d Hwf   = Hwt * Htg;
            Hwf.translation().z() = 0.0;

            // Use stored Hwf and Htf to calculate Hwt
            Eigen::Affine3d Hwt = Hwf * Htf.inverse();

            // Do a foot based position update
            filter.measure(Eigen::Vector3d(Hwt.translation()),
                           flat_foot_odometry_noise,
                           module::input::MeasurementType::FLAT_FOOT_ODOMETRY());

            // Do a foot based orientation update
            Eigen::Quaterniond foot_Rwt(Hwt.linear());
            filter.measure(foot_Rwt.coeffs(),
                           flat_foot_orientation_noise,
                           module::input::MeasurementType::FLAT_FOOT_ORIENTATION());
        }
        else {
            // Both feet are on the ground
            sensors.servo[utility::input::ServoID::L_HIP_ROLL].present_position = 0.0;
            sensors.servo[utility::input::ServoID::R_HIP_ROLL].present_position = 0.0;

            auto Htx = utility::motion::kinematics::calculateAllPositions(model, sensors);
            for (const auto& entry : Htx) {
                sensors.Htx[entry.first] = entry.second.matrix();
            }

            // Do the update for one foot, shouldnt matter which since we are in zombie
            Eigen::Affine3d Htf(sensors.Htx[utility::input::ServoID::L_ANKLE_ROLL]);
            Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

            Eigen::Affine3d Hwf   = Hwt * Htg;
            Hwf.translation().z() = 0.0;

            // Use stored Hwf and Htf to calculate Hwt
            Eigen::Affine3d Hwt = Hwf * Htf.inverse();

            // Do a foot based position update
            filter.measure(Eigen::Vector3d(Hwt.translation()),
                           flat_foot_odometry_noise,
                           module::input::MeasurementType::FLAT_FOOT_ODOMETRY());

            // Do a foot based orientation update
            Eigen::Quaterniond foot_Rwt(Hwt.linear());
            filter.measure(foot_Rwt.coeffs(),
                           flat_foot_orientation_noise,
                           module::input::MeasurementType::FLAT_FOOT_ORIENTATION());
        }

        // Time update
        INFO("Running time update for step " << i);
        INFO("Gyroscope Measurement....: " << gyroscope.transpose());
        INFO("Current Angle............: " << current_angle);
        INFO("Previous Angle...........: " << previous_angle);
        INFO("Rwt......................: " << Rwt.coeffs().transpose());
        INFO("Accelerometer Measurement: " << accelerometer.transpose() << " (" << accelerometer.norm() << ")");
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
