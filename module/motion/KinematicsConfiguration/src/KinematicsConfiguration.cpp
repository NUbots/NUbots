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

#include "KinematicsConfiguration.h"

#include "extension/Configuration.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "message/motion/KinematicsModel.h"

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

        model.TEAMDARWINCHEST_TO_ORIGIN =
            objDarwinModel["team_darwin_chest_to_origin"].as<float>() - model.leg.HIP_OFFSET_Z;
    }

    void KinematicsConfiguration::configureLeg(KinematicsModel& model, const YAML::Node& objLeg) {
        arma::vec3 leg_hipOffset = objLeg["hip_offset"].as<arma::vec3>();
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

        arma::vec3 head_neckToCamera       = objHead["neck_to_camera"].as<arma::vec3>();
        model.head.NECK_TO_CAMERA_X        = head_neckToCamera[0];
        model.head.NECK_TO_CAMERA_Y        = head_neckToCamera[1];
        model.head.NECK_TO_CAMERA_Z        = head_neckToCamera[2];
        model.head.INTERPUPILLARY_DISTANCE = objHead["ipd"].as<float>();

        auto& objNeck = objHead["neck"];

        model.head.NECK_LENGTH = objNeck["length"].as<float>();

        arma::vec3 neck_basePositionFromOrigin = objNeck["base_position_from_origin"].as<arma::vec3>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X = neck_basePositionFromOrigin[0];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y = neck_basePositionFromOrigin[1];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z = neck_basePositionFromOrigin[2];

        auto& objHeadMovementLimits = objHead["limits"];

        arma::vec2 headMovementLimits_yaw   = objHeadMovementLimits["yaw"].as<arma::vec2>();
        arma::vec2 headMovementLimits_pitch = objHeadMovementLimits["pitch"].as<arma::vec2>();
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
        model.arm.SHOULDER_Z_OFFSET          = objShoulder["offset"].as<arma::vec2>()[1];
        model.arm.SHOULDER_X_OFFSET          = objShoulder["offset"].as<arma::vec2>()[0];
        model.arm.SHOULDER_LENGTH            = objShoulder["length"].as<float>();
        model.arm.SHOULDER_WIDTH             = objShoulder["width"].as<float>();
        model.arm.SHOULDER_HEIGHT            = objShoulder["height"].as<float>();
        model.arm.UPPER_ARM_LENGTH           = objUpperArm["length"].as<float>();
        model.arm.UPPER_ARM_Y_OFFSET         = objUpperArm["offset"].as<arma::vec2>()[0];
        model.arm.UPPER_ARM_X_OFFSET         = objUpperArm["offset"].as<arma::vec2>()[1];
        model.arm.LOWER_ARM_LENGTH           = objLowerArm["length"].as<float>();
        model.arm.LOWER_ARM_Y_OFFSET         = objLowerArm["offset"].as<arma::vec2>()[0];
        model.arm.LOWER_ARM_Z_OFFSET         = objLowerArm["offset"].as<arma::vec2>()[1];
    }

    void KinematicsConfiguration::configureMassModel(KinematicsModel& model, const YAML::Node& objMassModel) {
        model.massModel.head        = convert<double, 4>(objMassModel["particles"]["head"].as<arma::vec4>());
        model.massModel.arm_upper   = convert<double, 4>(objMassModel["particles"]["arm_upper"].as<arma::vec4>());
        model.massModel.arm_lower   = convert<double, 4>(objMassModel["particles"]["arm_lower"].as<arma::vec4>());
        model.massModel.torso       = convert<double, 4>(objMassModel["particles"]["torso"].as<arma::vec4>());
        model.massModel.hip_block   = convert<double, 4>(objMassModel["particles"]["hip_block"].as<arma::vec4>());
        model.massModel.leg_upper   = convert<double, 4>(objMassModel["particles"]["leg_upper"].as<arma::vec4>());
        model.massModel.leg_lower   = convert<double, 4>(objMassModel["particles"]["leg_lower"].as<arma::vec4>());
        model.massModel.ankle_block = convert<double, 4>(objMassModel["particles"]["ankle_block"].as<arma::vec4>());
        model.massModel.foot        = convert<double, 4>(objMassModel["particles"]["foot"].as<arma::vec4>());
    }

    void KinematicsConfiguration::configureTensorModel(KinematicsModel& model, const YAML::Node& objTensorModel) {
        model.tensorModel.head      = convert<double, 3, 3>(objTensorModel["particles"]["head"].as<arma::mat33>());
        model.tensorModel.arm_upper = convert<double, 3, 3>(objTensorModel["particles"]["arm_upper"].as<arma::mat33>());
        model.tensorModel.arm_lower = convert<double, 3, 3>(objTensorModel["particles"]["arm_lower"].as<arma::mat33>());
        model.tensorModel.torso     = convert<double, 3, 3>(objTensorModel["particles"]["torso"].as<arma::mat33>());
        model.tensorModel.hip_block = convert<double, 3, 3>(objTensorModel["particles"]["hip_block"].as<arma::mat33>());
        model.tensorModel.leg_upper = convert<double, 3, 3>(objTensorModel["particles"]["leg_upper"].as<arma::mat33>());
        model.tensorModel.leg_lower = convert<double, 3, 3>(objTensorModel["particles"]["leg_lower"].as<arma::mat33>());
        model.tensorModel.ankle_block =
            convert<double, 3, 3>(objTensorModel["particles"]["ankle_block"].as<arma::mat33>());
        model.tensorModel.foot = convert<double, 3, 3>(objTensorModel["particles"]["foot"].as<arma::mat33>());
    }
}  // namespace motion
}  // namespace module
