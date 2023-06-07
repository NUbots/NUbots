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
            KinematicsModel model;
            configure(model, config);
            emit(std::make_unique<KinematicsModel>(model));
        });
    }

    void KinematicsConfiguration::configure(KinematicsModel& model, const Configuration& objNugusModel) {
        configureLeg(model, objNugusModel["leg"]);
        configureHead(model, objNugusModel["head"]);
        configureArm(model, objNugusModel["arm"]);

        configureMassModel(model, objNugusModel["mass_model"]);
        configureTensorModel(model, objNugusModel["tensor_model"]);
    }

    void KinematicsConfiguration::configureLeg(KinematicsModel& model, const YAML::Node& objLeg) {
        Eigen::Vector3d leg_hipOffset = objLeg["hip_offset"].as<Expression>();
        model.leg.HIP_OFFSET_X        = leg_hipOffset.x();
        model.leg.HIP_OFFSET_Y        = leg_hipOffset.y();
        model.leg.HIP_OFFSET_Z        = leg_hipOffset.z();

        model.leg.UPPER_LEG_LENGTH = objLeg["upper_leg_length"].as<double>();
        model.leg.LOWER_LEG_LENGTH = objLeg["lower_leg_length"].as<double>();

        model.leg.HEEL_LENGTH = objLeg["heel_length"].as<double>();

        model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = objLeg["foot_centre_to_ankle_centre"].as<double>();

        const auto& objFoot   = objLeg["foot"];
        model.leg.FOOT_WIDTH  = objFoot["width"].as<double>();
        model.leg.FOOT_HEIGHT = objFoot["height"].as<double>();
        model.leg.FOOT_LENGTH = objFoot["length"].as<double>();
        model.leg.TOE_LENGTH  = objFoot["toe_length"].as<double>();

        model.leg.LENGTH_BETWEEN_LEGS = 2.0 * model.leg.HIP_OFFSET_Y;

        const auto& objLeftRight            = objLeg["left_to_right"];
        model.leg.LEFT_TO_RIGHT_HIP_YAW     = objLeftRight["hip_yaw"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_ROLL    = objLeftRight["hip_roll"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_PITCH   = objLeftRight["hip_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_KNEE        = objLeftRight["knee"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = objLeftRight["ankle_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = objLeftRight["ankle_roll"].as<int>();
    }

    void KinematicsConfiguration::configureHead(KinematicsModel& model, const YAML::Node& objHead) {
        model.head.CAMERA_DECLINATION_ANGLE_OFFSET = objHead["camera_declination_angle_offset"].as<Expression>();

        Eigen::Vector3d head_neckToCamera  = objHead["neck_to_camera"].as<Expression>();
        model.head.NECK_TO_CAMERA_X        = head_neckToCamera.x();
        model.head.NECK_TO_CAMERA_Y        = head_neckToCamera.y();
        model.head.NECK_TO_CAMERA_Z        = head_neckToCamera.z();
        model.head.INTERPUPILLARY_DISTANCE = objHead["ipd"].as<double>();

        const auto& objNeck = objHead["neck"];

        model.head.NECK_LENGTH = objNeck["length"].as<double>();

        Eigen::Vector3d neck_basePositionFromOrigin = objNeck["base_position_from_origin"].as<Expression>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X      = neck_basePositionFromOrigin.x();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y      = neck_basePositionFromOrigin.y();
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z      = neck_basePositionFromOrigin.z();

        const auto& objHeadMovementLimits = objHead["limits"];

        Eigen::Vector2d headMovementLimits_yaw   = objHeadMovementLimits["yaw"].as<Expression>();
        Eigen::Vector2d headMovementLimits_pitch = objHeadMovementLimits["pitch"].as<Expression>();
        model.head.MIN_YAW                       = headMovementLimits_yaw.x();
        model.head.MAX_YAW                       = headMovementLimits_yaw.y();
        model.head.MIN_PITCH                     = headMovementLimits_pitch.x();
        model.head.MAX_PITCH                     = headMovementLimits_pitch.y();
    }

    void KinematicsConfiguration::configureArm(KinematicsModel& model, const YAML::Node& objArm) {
        const auto& objShoulder = objArm["shoulder"];
        const auto& objUpperArm = objArm["upper_arm"];
        const auto& objLowerArm = objArm["lower_arm"];

        model.arm.DISTANCE_BETWEEN_SHOULDERS = objArm["distance_between_shoulders"].as<double>();
        Eigen::Vector2d shoulderOffset       = objShoulder["offset"].as<Expression>();
        model.arm.SHOULDER_X_OFFSET          = shoulderOffset.x();
        model.arm.SHOULDER_Z_OFFSET          = shoulderOffset.y();
        model.arm.SHOULDER_LENGTH            = objShoulder["length"].as<double>();
        model.arm.SHOULDER_WIDTH             = objShoulder["width"].as<double>();
        model.arm.SHOULDER_HEIGHT            = objShoulder["height"].as<double>();

        model.arm.UPPER_ARM_LENGTH     = objUpperArm["length"].as<double>();
        Eigen::Vector2d upperArmOffset = objUpperArm["offset"].as<Expression>();
        model.arm.UPPER_ARM_Y_OFFSET   = upperArmOffset.x();
        model.arm.UPPER_ARM_X_OFFSET   = upperArmOffset.y();

        model.arm.LOWER_ARM_LENGTH     = objLowerArm["length"].as<double>();
        Eigen::Vector2d lowerArmOffset = objLowerArm["offset"].as<Expression>();
        model.arm.LOWER_ARM_Y_OFFSET   = lowerArmOffset.x();
        model.arm.LOWER_ARM_Z_OFFSET   = lowerArmOffset.y();
    }

    void KinematicsConfiguration::configureMassModel(KinematicsModel& model, const YAML::Node& objMassModel) {
        model.mass_model.head        = objMassModel["particles"]["head"].as<Expression>();
        model.mass_model.arm_upper   = objMassModel["particles"]["arm_upper"].as<Expression>();
        model.mass_model.arm_lower   = objMassModel["particles"]["arm_lower"].as<Expression>();
        model.mass_model.torso       = objMassModel["particles"]["torso"].as<Expression>();
        model.mass_model.hip_block   = objMassModel["particles"]["hip_block"].as<Expression>();
        model.mass_model.leg_upper   = objMassModel["particles"]["leg_upper"].as<Expression>();
        model.mass_model.leg_lower   = objMassModel["particles"]["leg_lower"].as<Expression>();
        model.mass_model.ankle_block = objMassModel["particles"]["ankle_block"].as<Expression>();
        model.mass_model.foot        = objMassModel["particles"]["foot"].as<Expression>();
    }

    void KinematicsConfiguration::configureTensorModel(KinematicsModel& model, const YAML::Node& objTensorModel) {
        model.tensor_model.head        = objTensorModel["particles"]["head"].as<Expression>();
        model.tensor_model.arm_upper   = objTensorModel["particles"]["arm_upper"].as<Expression>();
        model.tensor_model.arm_lower   = objTensorModel["particles"]["arm_lower"].as<Expression>();
        model.tensor_model.torso       = objTensorModel["particles"]["torso"].as<Expression>();
        model.tensor_model.hip_block   = objTensorModel["particles"]["hip_block"].as<Expression>();
        model.tensor_model.leg_upper   = objTensorModel["particles"]["leg_upper"].as<Expression>();
        model.tensor_model.leg_lower   = objTensorModel["particles"]["leg_lower"].as<Expression>();
        model.tensor_model.ankle_block = objTensorModel["particles"]["ankle_block"].as<Expression>();
        model.tensor_model.foot        = objTensorModel["particles"]["foot"].as<Expression>();
    }
}  // namespace module::actuation
