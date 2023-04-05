/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "KinematicsConfiguration.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::actuation {

    using extension::Configuration;
    using message::actuation::KinematicsModel;
    using utility::support::Expression;

    KinematicsConfiguration::KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("KinematicsConfiguration.yaml").then([this](const Configuration& config) {
            // Create NUclear robot model message
            KinematicsModel model;

            // Create a tinyrobotics model of the NUgus URDF file description
            cfg.urdf_path = config["urdf_path"].as<std::string>();
            nugus_model   = tinyrobotics::import_urdf<float, n_joints>(cfg.urdf_path);
            nugus_model.show_details();

            // Configure NUclear robot model message with values from config file and tinyrobotics model
            configure(model, config);
            emit(std::make_unique<KinematicsModel>(model));
        });
    }

    void KinematicsConfiguration::configure(KinematicsModel& model, const Configuration& config) {
        configure_leg(model, config["leg"]);
        configure_head(model, config["head"]);
        configure_arm(model, config["arm"]);
        configure_mass_model(model, config["mass_model"]);
        configure_tensor_model(model, config["tensor_model"]);
    }

    void KinematicsConfiguration::configure_leg(KinematicsModel& model, const YAML::Node& obj_leg) {
        // Create a vector of configuration coordinates for home configuration (all zeros)
        auto q_home = nugus_model.home_configuration();

        // Calculate the forward kinematics from torso to hip
        Eigen::Vector3f leg_hip_offset =
            translation(nugus_model, q_home, std::string("torso"), std::string("left_upper_leg"));
        model.leg.HIP_OFFSET_X = leg_hip_offset.x();
        model.leg.HIP_OFFSET_Y = leg_hip_offset.y();
        model.leg.HIP_OFFSET_Z = leg_hip_offset.z();

        // Calculate the forward kinematics from upper leg pitch to knee
        model.leg.UPPER_LEG_LENGTH =
            translation(nugus_model, q_home, std::string("left_upper_leg"), std::string("left_lower_leg")).x();

        // Calculate the forward kinematics from knee to ankle pitch
        model.leg.LOWER_LEG_LENGTH =
            translation(nugus_model, q_home, std::string("left_lower_leg"), std::string("left_ankle")).x();
        std::cout << "lower leg length: " << model.leg.LOWER_LEG_LENGTH << std::endl;

        model.leg.HEEL_LENGTH = obj_leg["heel_length"].as<float>();

        model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = obj_leg["foot_centre_to_ankle_centre"].as<float>();

        const auto& obj_foot  = obj_leg["foot"];
        model.leg.FOOT_WIDTH  = obj_foot["width"].as<float>();
        model.leg.FOOT_HEIGHT = obj_foot["height"].as<float>();
        model.leg.FOOT_LENGTH = obj_foot["length"].as<float>();
        model.leg.TOE_LENGTH  = obj_foot["toe_length"].as<float>();

        model.leg.LENGTH_BETWEEN_LEGS = 2.0 * model.leg.HIP_OFFSET_Y;

        const auto& obj_left_right          = obj_leg["left_to_right"];
        model.leg.LEFT_TO_RIGHT_HIP_YAW     = obj_left_right["hip_yaw"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_ROLL    = obj_left_right["hip_roll"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_PITCH   = obj_left_right["hip_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_KNEE        = obj_left_right["knee"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = obj_left_right["ankle_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = obj_left_right["ankle_roll"].as<int>();
    }

    void KinematicsConfiguration::configure_head(KinematicsModel& model, const YAML::Node& obj_head) {
        model.head.CAMERA_DECLINATION_ANGLE_OFFSET = obj_head["camera_declination_angle_offset"].as<Expression>();

        Eigen::Vector3f head_neck_to_camera = obj_head["neck_to_camera"].as<Expression>();
        model.head.NECK_TO_CAMERA_X         = head_neck_to_camera.x();
        model.head.NECK_TO_CAMERA_Y         = head_neck_to_camera.y();
        model.head.NECK_TO_CAMERA_Z         = head_neck_to_camera.z();
        model.head.INTERPUPILLARY_DISTANCE  = obj_head["ipd"].as<float>();

        const auto& objNeck = obj_head["neck"];

        model.head.NECK_LENGTH = objNeck["length"].as<float>();

        Eigen::Vector3f neck_base_position_from_origin = objNeck["base_position_from_origin"].as<Expression>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X         = neck_base_position_from_origin.x();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y         = neck_base_position_from_origin.y();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z         = neck_base_position_from_origin.z();

        const auto& obj_headMovementLimits = obj_head["limits"];

        Eigen::Vector2f head_movement_limits_yaw  = obj_headMovementLimits["yaw"].as<Expression>();
        Eigen::Vector2f headMovement_limits_pitch = obj_headMovementLimits["pitch"].as<Expression>();
        model.head.MIN_YAW                        = head_movement_limits_yaw.x();
        model.head.MAX_YAW                        = head_movement_limits_yaw.y();
        model.head.MIN_PITCH                      = headMovement_limits_pitch.x();
        model.head.MAX_PITCH                      = headMovement_limits_pitch.y();
    }

    void KinematicsConfiguration::configure_arm(KinematicsModel& model, const YAML::Node& obj_arm) {
        const auto& obj_shoulder  = obj_arm["shoulder"];
        const auto& obj_upper_arm = obj_arm["upper_arm"];
        const auto& obj_lower_arm = obj_arm["lower_arm"];

        model.arm.DISTANCE_BETWEEN_SHOULDERS = obj_arm["distance_between_shoulders"].as<float>();
        Eigen::Vector2f shoulder_offset      = obj_shoulder["offset"].as<Expression>();
        model.arm.SHOULDER_X_OFFSET          = shoulder_offset.x();
        model.arm.SHOULDER_Z_OFFSET          = shoulder_offset.y();
        model.arm.SHOULDER_LENGTH            = obj_shoulder["length"].as<float>();
        model.arm.SHOULDER_WIDTH             = obj_shoulder["width"].as<float>();
        model.arm.SHOULDER_HEIGHT            = obj_shoulder["height"].as<float>();

        model.arm.UPPER_ARM_LENGTH       = obj_upper_arm["length"].as<float>();
        Eigen::Vector2f upper_arm_offset = obj_upper_arm["offset"].as<Expression>();
        model.arm.UPPER_ARM_Y_OFFSET     = upper_arm_offset.x();
        model.arm.UPPER_ARM_X_OFFSET     = upper_arm_offset.y();

        model.arm.LOWER_ARM_LENGTH     = obj_lower_arm["length"].as<float>();
        Eigen::Vector2f lowerArmOffset = obj_lower_arm["offset"].as<Expression>();
        model.arm.LOWER_ARM_Y_OFFSET   = lowerArmOffset.x();
        model.arm.LOWER_ARM_Z_OFFSET   = lowerArmOffset.y();
    }

    void KinematicsConfiguration::configure_mass_model(KinematicsModel& model, const YAML::Node& obj_mass_model) {
        model.mass_model.head        = obj_mass_model["particles"]["head"].as<Expression>();
        model.mass_model.arm_upper   = obj_mass_model["particles"]["arm_upper"].as<Expression>();
        model.mass_model.arm_lower   = obj_mass_model["particles"]["arm_lower"].as<Expression>();
        model.mass_model.torso       = obj_mass_model["particles"]["torso"].as<Expression>();
        model.mass_model.hip_block   = obj_mass_model["particles"]["hip_block"].as<Expression>();
        model.mass_model.leg_upper   = obj_mass_model["particles"]["leg_upper"].as<Expression>();
        model.mass_model.leg_lower   = obj_mass_model["particles"]["leg_lower"].as<Expression>();
        model.mass_model.ankle_block = obj_mass_model["particles"]["ankle_block"].as<Expression>();
        model.mass_model.foot        = obj_mass_model["particles"]["foot"].as<Expression>();
    }

    void KinematicsConfiguration::configure_tensor_model(KinematicsModel& model, const YAML::Node& obj_tensor_model) {
        model.tensor_model.head        = obj_tensor_model["particles"]["head"].as<Expression>();
        model.tensor_model.arm_upper   = obj_tensor_model["particles"]["arm_upper"].as<Expression>();
        model.tensor_model.arm_lower   = obj_tensor_model["particles"]["arm_lower"].as<Expression>();
        model.tensor_model.torso       = obj_tensor_model["particles"]["torso"].as<Expression>();
        model.tensor_model.hip_block   = obj_tensor_model["particles"]["hip_block"].as<Expression>();
        model.tensor_model.leg_upper   = obj_tensor_model["particles"]["leg_upper"].as<Expression>();
        model.tensor_model.leg_lower   = obj_tensor_model["particles"]["leg_lower"].as<Expression>();
        model.tensor_model.ankle_block = obj_tensor_model["particles"]["ankle_block"].as<Expression>();
        model.tensor_model.foot        = obj_tensor_model["particles"]["foot"].as<Expression>();
    }
}  // namespace module::actuation
