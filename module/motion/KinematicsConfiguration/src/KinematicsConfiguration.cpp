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

#include "message/motion/KinematicsModel.hpp"

#include "utility/support/eigen_armadillo.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {
namespace motion {

    using extension::Configuration;
    using message::motion::KinematicsModel;
    using utility::support::Expression;


    KinematicsConfiguration::KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("KinematicsConfiguration.yaml").then([this](const Configuration& config) {
            KinematicsModel model;
            configure(model, config);
            emit(std::make_unique<KinematicsModel>(model));
        });
    }

    void KinematicsConfiguration::configure(KinematicsModel& model, const Configuration& objDarwinModel) {
        configureLeg(model, objDarwinModel["leg"]);
        configureHead(model, objDarwinModel["head"]);
        configureArm(model, objDarwinModel["arm"]);

        configureMassModel(model, objDarwinModel["mass_model"]);
        configureTensorModel(model, objDarwinModel["tensor_model"]);
    }

    void KinematicsConfiguration::configureLeg(KinematicsModel& model, const YAML::Node& objLeg) {
        arma::vec3 leg_hipOffset = objLeg["hip_offset"].as<Expression>();
        model.leg.HIP_OFFSET_X   = leg_hipOffset[0];
        model.leg.HIP_OFFSET_Y   = leg_hipOffset[1];
        model.leg.HIP_OFFSET_Z   = leg_hipOffset[2];

        model.leg.UPPER_LEG_LENGTH = objLeg["upper_leg_length"].as<float>();
        model.leg.LOWER_LEG_LENGTH = objLeg["lower_leg_length"].as<float>();

        model.leg.HEEL_LENGTH = objLeg["heel_length"].as<float>();

        model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = objLeg["foot_centre_to_ankle_centre"].as<float>();

        auto& objFoot         = objLeg["foot"];
        model.leg.FOOT_WIDTH  = objFoot["width"].as<float>();
        model.leg.FOOT_HEIGHT = objFoot["height"].as<float>();
        model.leg.FOOT_LENGTH = objFoot["length"].as<float>();
        model.leg.TOE_LENGTH  = objFoot["toe_length"].as<float>();

        model.leg.LENGTH_BETWEEN_LEGS = 2.0 * model.leg.HIP_OFFSET_Y;

        auto& objLeftRight                  = objLeg["left_to_right"];
        model.leg.LEFT_TO_RIGHT_HIP_YAW     = objLeftRight["hip_yaw"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_ROLL    = objLeftRight["hip_roll"].as<int>();
        model.leg.LEFT_TO_RIGHT_HIP_PITCH   = objLeftRight["hip_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_KNEE        = objLeftRight["knee"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_PITCH = objLeftRight["ankle_pitch"].as<int>();
        model.leg.LEFT_TO_RIGHT_ANKLE_ROLL  = objLeftRight["ankle_roll"].as<int>();
    }

    void KinematicsConfiguration::configureHead(KinematicsModel& model, const YAML::Node& objHead) {
        model.head.CAMERA_DECLINATION_ANGLE_OFFSET = objHead["camera_declination_angle_offset"].as<Expression>();

        arma::vec3 head_neckToCamera       = objHead["neck_to_camera"].as<Expression>();
        model.head.NECK_TO_CAMERA_X        = head_neckToCamera[0];
        model.head.NECK_TO_CAMERA_Y        = head_neckToCamera[1];
        model.head.NECK_TO_CAMERA_Z        = head_neckToCamera[2];
        model.head.INTERPUPILLARY_DISTANCE = objHead["ipd"].as<float>();

        auto& objNeck = objHead["neck"];

        model.head.NECK_LENGTH = objNeck["length"].as<float>();

        arma::vec3 neck_basePositionFromOrigin = objNeck["base_position_from_origin"].as<Expression>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X = neck_basePositionFromOrigin[0];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y = neck_basePositionFromOrigin[1];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z = neck_basePositionFromOrigin[2];

        auto& objHeadMovementLimits = objHead["limits"];

        arma::vec2 headMovementLimits_yaw   = objHeadMovementLimits["yaw"].as<Expression>();
        arma::vec2 headMovementLimits_pitch = objHeadMovementLimits["pitch"].as<Expression>();
        model.head.MIN_YAW                  = headMovementLimits_yaw[0];
        model.head.MAX_YAW                  = headMovementLimits_yaw[1];
        model.head.MIN_PITCH                = headMovementLimits_pitch[0];
        model.head.MAX_PITCH                = headMovementLimits_pitch[1];
    }

    void KinematicsConfiguration::configureArm(KinematicsModel& model, const YAML::Node& objArm) {
        auto& objShoulder = objArm["shoulder"];
        auto& objUpperArm = objArm["upper_arm"];
        auto& objLowerArm = objArm["lower_arm"];

        model.arm.DISTANCE_BETWEEN_SHOULDERS = objArm["distance_between_shoulders"].as<float>();
        arma::vec2 shoulder                  = objShoulder["offset"].as<Expression>();
        model.arm.SHOULDER_Z_OFFSET          = shoulder[1];
        model.arm.SHOULDER_X_OFFSET          = shoulder[0];
        model.arm.SHOULDER_LENGTH            = objShoulder["length"].as<float>();
        model.arm.SHOULDER_WIDTH             = objShoulder["width"].as<float>();
        model.arm.SHOULDER_HEIGHT            = objShoulder["height"].as<float>();
        arma::vec2 upper_arm                 = objUpperArm["offset"].as<Expression>();
        model.arm.UPPER_ARM_LENGTH           = objUpperArm["length"].as<float>();
        model.arm.UPPER_ARM_Y_OFFSET         = upper_arm[0];
        model.arm.UPPER_ARM_X_OFFSET         = upper_arm[1];
        arma::vec2 lower_arm                 = objLowerArm["offset"].as<Expression>();
        model.arm.LOWER_ARM_LENGTH           = objLowerArm["length"].as<float>();
        model.arm.LOWER_ARM_Y_OFFSET         = lower_arm[0];
        model.arm.LOWER_ARM_Z_OFFSET         = lower_arm[1];
    }

    void KinematicsConfiguration::configureMassModel(KinematicsModel& model, const YAML::Node& objMassModel) {
        model.massModel.head        = convert(arma::vec4(objMassModel["particles"]["head"].as<Expression>()));
        model.massModel.arm_upper   = convert(arma::vec4(objMassModel["particles"]["arm_upper"].as<Expression>()));
        model.massModel.arm_lower   = convert(arma::vec4(objMassModel["particles"]["arm_lower"].as<Expression>()));
        model.massModel.torso       = convert(arma::vec4(objMassModel["particles"]["torso"].as<Expression>()));
        model.massModel.hip_block   = convert(arma::vec4(objMassModel["particles"]["hip_block"].as<Expression>()));
        model.massModel.leg_upper   = convert(arma::vec4(objMassModel["particles"]["leg_upper"].as<Expression>()));
        model.massModel.leg_lower   = convert(arma::vec4(objMassModel["particles"]["leg_lower"].as<Expression>()));
        model.massModel.ankle_block = convert(arma::vec4(objMassModel["particles"]["ankle_block"].as<Expression>()));
        model.massModel.foot        = convert(arma::vec4(objMassModel["particles"]["foot"].as<Expression>()));
    }

    void KinematicsConfiguration::configureTensorModel(KinematicsModel& model, const YAML::Node& objTensorModel) {
        model.tensorModel.head      = convert(arma::mat33(objTensorModel["particles"]["head"].as<Expression>()));
        model.tensorModel.arm_upper = convert(arma::mat33(objTensorModel["particles"]["arm_upper"].as<Expression>()));
        model.tensorModel.arm_lower = convert(arma::mat33(objTensorModel["particles"]["arm_lower"].as<Expression>()));
        model.tensorModel.torso     = convert(arma::mat33(objTensorModel["particles"]["torso"].as<Expression>()));
        model.tensorModel.hip_block = convert(arma::mat33(objTensorModel["particles"]["hip_block"].as<Expression>()));
        model.tensorModel.leg_upper = convert(arma::mat33(objTensorModel["particles"]["leg_upper"].as<Expression>()));
        model.tensorModel.leg_lower = convert(arma::mat33(objTensorModel["particles"]["leg_lower"].as<Expression>()));
        model.tensorModel.ankle_block =
            convert(arma::mat33(objTensorModel["particles"]["ankle_block"].as<Expression>()));
        model.tensorModel.foot = convert(arma::mat33(objTensorModel["particles"]["foot"].as<Expression>()));
    }
}  // namespace motion
}  // namespace module
