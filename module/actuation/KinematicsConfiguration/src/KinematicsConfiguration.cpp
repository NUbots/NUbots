/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
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

#include "KinematicsConfiguration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/ServoOffsets.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::KinematicsModel;
    using message::actuation::ServoOffsets;
    using utility::support::Expression;

    KinematicsConfiguration::KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("KinematicsConfiguration.yaml").then([this](const Configuration& config) {
            KinematicsModel model;
            configure(model, config);
            emit(std::make_unique<KinematicsModel>(model));
        });

        on<Configuration>("Offsets.yaml").then([this](const Configuration& config) {
            auto offsets = std::make_unique<ServoOffsets>();
            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                offsets->offsets.emplace_back(config["servos"][i]["offset"].as<Expression>(),
                                              config["servos"][i]["direction"].as<Expression>(),
                                              config["servos"][i]["simulated"].as<bool>());
            }
            emit(offsets);
        });
    }

    void KinematicsConfiguration::configure(KinematicsModel& model, const Configuration& nugus_model) {
        configure_leg(model, nugus_model["leg"]);
        configure_head(model, nugus_model["head"]);
        configure_arm(model, nugus_model["arm"]);

        configure_mass_model(model, nugus_model["mass_model"]);
        configure_tensor_model(model, nugus_model["tensor_model"]);
    }

    void KinematicsConfiguration::configure_leg(KinematicsModel& model, const YAML::Node& leg) {
        const Eigen::Vector3f leg_hip_offset = leg["hip_offset"].as<Expression>();
        model.leg.HIP_OFFSET_X               = leg_hip_offset.x();
        model.leg.HIP_OFFSET_Y               = leg_hip_offset.y();
        model.leg.HIP_OFFSET_Z               = leg_hip_offset.z();

        model.leg.UPPER_LEG_LENGTH = leg["upper_leg_length"].as<float>();
        model.leg.LOWER_LEG_LENGTH = leg["lower_leg_length"].as<float>();

        model.leg.HEEL_LENGTH = leg["heel_length"].as<float>();

        model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = leg["foot_centre_to_ankle_centre"].as<float>();

        const auto& foot      = leg["foot"];
        model.leg.FOOT_WIDTH  = foot["width"].as<float>();
        model.leg.FOOT_HEIGHT = foot["height"].as<float>();
        model.leg.FOOT_LENGTH = foot["length"].as<float>();
        model.leg.TOE_LENGTH  = foot["toe_length"].as<float>();

        model.leg.LENGTH_BETWEEN_LEGS = 2.0 * model.leg.HIP_OFFSET_Y;

        const auto& left_right              = leg["left_to_right"];
        model.leg.LEFT_TO_RIGHT_HIP_YAW     = left_right["hip_yaw"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_ROLL    = left_right["hip_roll"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_PITCH   = left_right["hip_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_KNEE        = left_right["knee"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = left_right["ankle_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = left_right["ankle_roll"].as<int>();
    }

    void KinematicsConfiguration::configure_head(KinematicsModel& model, const YAML::Node& head) {
        model.head.CAMERA_DECLINATION_ANGLE_OFFSET = head["camera_declination_angle_offset"].as<Expression>();

        const Eigen::Vector3f head_neck_to_camera = head["neck_to_camera"].as<Expression>();
        model.head.NECK_TO_CAMERA_X               = head_neck_to_camera.x();
        model.head.NECK_TO_CAMERA_Y               = head_neck_to_camera.y();
        model.head.NECK_TO_CAMERA_Z               = head_neck_to_camera.z();
        model.head.INTERPUPILLARY_DISTANCE        = head["ipd"].as<float>();

        const auto& neck       = head["neck"];
        model.head.NECK_LENGTH = neck["length"].as<float>();

        Eigen::Vector3f neck_base_position_from_origin = neck["base_position_from_origin"].as<Expression>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X         = neck_base_position_from_origin.x();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y         = neck_base_position_from_origin.y();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z         = neck_base_position_from_origin.z();

        const auto& head_limits = head["limits"];

        const Eigen::Vector2f head_limits_yaw   = head_limits["yaw"].as<Expression>();
        const Eigen::Vector2f head_limits_pitch = head_limits["pitch"].as<Expression>();
        model.head.MIN_YAW                      = head_limits_yaw.x();
        model.head.MAX_YAW                      = head_limits_yaw.y();
        model.head.MIN_PITCH                    = head_limits_pitch.x();
        model.head.MAX_PITCH                    = head_limits_pitch.y();
    }

    void KinematicsConfiguration::configure_arm(KinematicsModel& model, const YAML::Node& arm) {
        const auto& shoulder  = arm["shoulder"];
        const auto& upper_arm = arm["upper_arm"];
        const auto& lower_arm = arm["lower_arm"];

        model.arm.DISTANCE_BETWEEN_SHOULDERS  = arm["distance_between_shoulders"].as<float>();
        const Eigen::Vector2f shoulder_offset = shoulder["offset"].as<Expression>();
        model.arm.SHOULDER_X_OFFSET           = shoulder_offset.x();
        model.arm.SHOULDER_Z_OFFSET           = shoulder_offset.y();
        model.arm.SHOULDER_LENGTH             = shoulder["length"].as<float>();
        model.arm.SHOULDER_WIDTH              = shoulder["width"].as<float>();
        model.arm.SHOULDER_HEIGHT             = shoulder["height"].as<float>();

        model.arm.UPPER_ARM_LENGTH             = upper_arm["length"].as<float>();
        const Eigen::Vector2f upper_arm_offset = upper_arm["offset"].as<Expression>();
        model.arm.UPPER_ARM_Y_OFFSET           = upper_arm_offset.x();
        model.arm.UPPER_ARM_X_OFFSET           = upper_arm_offset.y();

        model.arm.LOWER_ARM_LENGTH             = lower_arm["length"].as<float>();
        const Eigen::Vector2f lower_arm_offset = lower_arm["offset"].as<Expression>();
        model.arm.LOWER_ARM_Y_OFFSET           = lower_arm_offset.x();
        model.arm.LOWER_ARM_Z_OFFSET           = lower_arm_offset.y();
    }

    void KinematicsConfiguration::configure_mass_model(KinematicsModel& model, const YAML::Node& mass_model) {
        model.mass_model.head        = mass_model["particles"]["head"].as<Expression>();
        model.mass_model.arm_upper   = mass_model["particles"]["arm_upper"].as<Expression>();
        model.mass_model.arm_lower   = mass_model["particles"]["arm_lower"].as<Expression>();
        model.mass_model.torso       = mass_model["particles"]["torso"].as<Expression>();
        model.mass_model.hip_block   = mass_model["particles"]["hip_block"].as<Expression>();
        model.mass_model.leg_upper   = mass_model["particles"]["leg_upper"].as<Expression>();
        model.mass_model.leg_lower   = mass_model["particles"]["leg_lower"].as<Expression>();
        model.mass_model.ankle_block = mass_model["particles"]["ankle_block"].as<Expression>();
        model.mass_model.foot        = mass_model["particles"]["foot"].as<Expression>();
    }

    void KinematicsConfiguration::configure_tensor_model(KinematicsModel& model, const YAML::Node& tensor_model) {
        model.tensor_model.head        = tensor_model["particles"]["head"].as<Expression>();
        model.tensor_model.arm_upper   = tensor_model["particles"]["arm_upper"].as<Expression>();
        model.tensor_model.arm_lower   = tensor_model["particles"]["arm_lower"].as<Expression>();
        model.tensor_model.torso       = tensor_model["particles"]["torso"].as<Expression>();
        model.tensor_model.hip_block   = tensor_model["particles"]["hip_block"].as<Expression>();
        model.tensor_model.leg_upper   = tensor_model["particles"]["leg_upper"].as<Expression>();
        model.tensor_model.leg_lower   = tensor_model["particles"]["leg_lower"].as<Expression>();
        model.tensor_model.ankle_block = tensor_model["particles"]["ankle_block"].as<Expression>();
        model.tensor_model.foot        = tensor_model["particles"]["foot"].as<Expression>();
    }
}  // namespace module::actuation
