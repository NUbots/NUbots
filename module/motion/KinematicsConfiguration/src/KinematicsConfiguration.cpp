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

#include "message/motion/KinematicsModels.h"

namespace module {
namespace motion {

    using extension::Configuration;
    using message::motion::KinematicsModel;
    using utility::support::Expression;


    KinematicsConfiguration::KinematicsConfiguration(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("KinematicsConfiguration.yaml").then([this] (const Configuration& config) {
           
            KinematicsModel model;
            configure(model, config);
            emit(std::make_unique<KinematicsModel>(model));

        });
        
    }

    void KinematicsConfiguration::configure (KinematicsModel& model, const Configuration& objDarwinModel) {
        configureLeg(model, objDarwinModel["leg"]);
        configureHead(model, objDarwinModel["head"]);
        configureArm(model, objDarwinModel["arm"]);

        configureMassModel(model, objDarwinModel["mass_model"]);
        model.TEAMDARWINCHEST_TO_ORIGIN = objDarwinModel["team_darwin_chest_to_origin"].as<float>() - model.leg.HIP_OFFSET_Z;
    }

    void KinematicsConfiguration::configureLeg (KinematicsModel& model, const YAML::Node& objLeg) {
        arma::vec3 leg_hipOffset = objLeg["hip_offset"].as<arma::vec3>();
        model.leg.HIP_OFFSET_X = leg_hipOffset[0];
        model.leg.HIP_OFFSET_Y = leg_hipOffset[1];
        model.leg.HIP_OFFSET_Z = leg_hipOffset[2];

        model.leg.UPPER_LEG_LENGTH = objLeg["upper_leg_length"].as<float>();
        model.leg.LOWER_LEG_LENGTH = objLeg["lower_leg_length"].as<float>();

        model.leg.HEEL_LENGTH = objLeg["heel_length"].as<float>();

        model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = objLeg["foot_centre_to_ankle_centre"].as<float>();

        auto& objFoot = objLeg["foot"];
        model.leg.FOOT_WIDTH = objFoot["width"].as<float>();
        model.leg.FOOT_HEIGHT = objFoot["height"].as<float>();
        model.leg.FOOT_LENGTH = objFoot["length"].as<float>();
        model.leg.TOE_LENGTH = objFoot["toe_length"].as<float>();

        model.leg.LENGTH_BETWEEN_LEGS = objLeg["length_between_legs"].as<float>();
    }

    void KinematicsConfiguration::configureHead (KinematicsModel& model, const YAML::Node& objHead) {
        model.head.CAMERA_DECLINATION_ANGLE_OFFSET = objHead["camera_declination_angle_offset"].as<float>();

        arma::vec3 head_neckToCamera = objHead["neck_to_camera"].as<arma::vec3>();
        model.head.NECK_TO_CAMERA_X = head_neckToCamera[0];
        model.head.NECK_TO_CAMERA_Y = head_neckToCamera[1];
        model.head.NECK_TO_CAMERA_Z = head_neckToCamera[2];

        auto& objNeck = objHead["neck"];

        model.head.NECK_LENGTH = objNeck["length"].as<float>();

        arma::vec3 neck_basePositionFromOrigin = objNeck["base_position_from_origin"].as<arma::vec3>();
        model.head.NECK_BASE_POS_FROM_ORIGIN_X = neck_basePositionFromOrigin[0];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Y = neck_basePositionFromOrigin[1];
        model.head.NECK_BASE_POS_FROM_ORIGIN_Z = neck_basePositionFromOrigin[2];

        auto& objHeadMovementLimits = objHead["limits"];

        arma::vec2 headMovementLimits_yaw = objHeadMovementLimits["yaw"].as<arma::vec2>();
        arma::vec2 headMovementLimits_pitch = objHeadMovementLimits["pitch"].as<arma::vec2>();
        model.head.MIN_YAW = headMovementLimits_yaw[0];
        model.head.MAX_YAW = headMovementLimits_yaw[1];
        model.head.MIN_PITCH = headMovementLimits_pitch[0];
        model.head.MAX_PITCH = headMovementLimits_pitch[1];

    }

    void KinematicsConfiguration::configureArm (KinematicsModel& model, const YAML::Node& objArm) {
        // arm.distanceBetweenShoulders = objArm["distance_between_shoulders"].as<float>();

        // auto& shoulder = arm.shoulder;
        auto& objShoulder = objArm["shoulder"];
        // shoulder.length = objShoulder["length"].as<float>();
        // shoulder.width = objShoulder["width"].as<float>();
        // shoulder.height = objShoulder["height"].as<float>();
        // shoulder.offset = objShoulder["offset"].as<arma::vec2>();

        // auto& upperArm = arm.upperArm;
        auto& objUpperArm = objArm["upper_arm"];
        // upperArm.length = objUpperArm["length"].as<float>();
        // upperArm.offset = objUpperArm["offset"].as<arma::vec2>();

        // auto& lowerArm = arm.lowerArm;
        auto& objLowerArm = objArm["lower_arm"];
        // lowerArm.length = objLowerArm["length"].as<float>();
        // lowerArm.offset = objLowerArm["offset"].as<arma::vec2>();

        model.arm.DISTANCE_BETWEEN_SHOULDERS = objArm["distance_between_shoulders"].as<float>();
        model.arm.SHOULDER_Z_OFFSET = objShoulder["offset"].as<arma::vec2>()[1];
        model.arm.SHOULDER_X_OFFSET = objShoulder["offset"].as<arma::vec2>()[0];
        model.arm.SHOULDER_LENGTH = objShoulder["length"].as<float>();
        model.arm.SHOULDER_WIDTH = objShoulder["width"].as<float>();
        model.arm.SHOULDER_HEIGHT = objShoulder["height"].as<float>();
        model.arm.UPPER_ARM_LENGTH = objUpperArm["length"].as<float>();
        model.arm.UPPER_ARM_Y_OFFSET = objUpperArm["offset"].as<arma::vec2>()[0];
        model.arm.UPPER_ARM_X_OFFSET = objUpperArm["offset"].as<arma::vec2>()[1];
        model.arm.LOWER_ARM_LENGTH = objLowerArm["length"].as<float>();
        model.arm.LOWER_ARM_Y_OFFSET = objLowerArm["offset"].as<arma::vec2>()[0];
        model.arm.LOWER_ARM_Z_OFFSET = objLowerArm["offset"].as<arma::vec2>()[1];
    }

    void KinematicsConfiguration::configureMassModel (KinematicsModel& model, const YAML::Node& objMassModel) {
        // massModel.numberOfMasses = objMassModel["number_of_masses"].as<float>();
     //    massModel.massRepresentationDimension = objMassModel["mass_representation_dimension"].as<float>();

        // auto& masses = massModel.masses;
        auto& objMasses = objMassModel["masses"];

  //       masses.leftShoulderRoll = objMasses[3].as<arma::vec4>();
  //       masses.rightShoulderRoll = objMasses[2].as<arma::vec4>();
  //       masses.leftShoulderPitch = objMasses[1].as<arma::vec4>();
  //       masses.rightShoulderPitch = objMasses[0].as<arma::vec4>();

        // masses.leftElbow = objMasses[5].as<arma::vec4>();
  //       masses.rightElbow = objMasses[4].as<arma::vec4>();

  //       masses.leftHipRoll = objMasses[9].as<arma::vec4>();
  //       masses.rightHipRoll = objMasses[8].as<arma::vec4>();
  //       masses.leftHipPitch = objMasses[11].as<arma::vec4>();
  //       masses.rightHipPitch = objMasses[10].as<arma::vec4>();
  //       masses.leftHipYaw = objMasses[7].as<arma::vec4>();
  //       masses.rightHipYaw = objMasses[6].as<arma::vec4>();

  //       masses.leftKnee = objMasses[13].as<arma::vec4>();
  //       masses.rightKnee = objMasses[12].as<arma::vec4>();

  //       masses.leftAnkleRoll = objMasses[17].as<arma::vec4>();
  //       masses.rightAnkleRoll = objMasses[16].as<arma::vec4>();
  //       masses.leftAnklePitch = objMasses[15].as<arma::vec4>();
  //       masses.rightAnklePitch = objMasses[14].as<arma::vec4>();

  //       masses.headPitch = objMasses[19].as<arma::vec4>();
  //       masses.headYaw = objMasses[18].as<arma::vec4>();

        // masses.torso = objMasses[20].as<arma::vec4>();

        auto masses = objMasses.as<std::vector<arma::vec4>>();
        std::transform(masses.begin(), masses.end(), model.massModel.masses.end(), [] (const arma::vec4& avec) {
            return(convert<double, 4>(avec));
        });
    }

}
}
