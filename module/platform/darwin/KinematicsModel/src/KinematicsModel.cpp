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

#include "KinematicsModel.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "utility/motion/RobotModels.h"

namespace module {
namespace platform {
namespace darwin {

    using message::support::Configuration;
    using message::platform::darwin::DarwinKinematicsModel;

    using utility::support::Expression;
    using utility::motion::kinematics::DarwinModel;

    KinematicsModel::KinematicsModel(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("KinematicsModel.yaml").then([this] (const Configuration& config) {
        	DarwinKinematicsModel darwinKinematicsModel;
        	configure(darwinKinematicsModel, config);
			emit(std::make_unique<DarwinKinematicsModel>(darwinKinematicsModel));
        });
    }

    void KinematicsModel::configure (DarwinKinematicsModel& darwinModel, const Configuration& objDarwinModel) {
        configureLeg(darwinModel.leg, objDarwinModel["leg"]);
        configureHead(darwinModel.head, objDarwinModel["head"]);
        configureArm(darwinModel.arm, objDarwinModel["arm"]);

		darwinModel.teamDarwinChestToOrigin = 0.096 - darwinModel.leg.hipOffset[2];

        configureMassModel(darwinModel.massModel, objDarwinModel["mass_model"]);
    }

    void KinematicsModel::configureLeg (DarwinKinematicsModel::Leg& leg, const YAML::Node& objLeg) {
        leg.hipOffset = objLeg["hip_offset"].as<arma::vec3>();
        DarwinModel::Leg::HIP_OFFSET_X = leg.hipOffset[0];
        DarwinModel::Leg::HIP_OFFSET_Y = leg.hipOffset[1];
        DarwinModel::Leg::HIP_OFFSET_Z = leg.hipOffset[2];

        DarwinModel::Leg::UPPER_LEG_LENGTH = leg.upperLegLength = objLeg["upper_leg_length"].as<float>();
        DarwinModel::Leg::LOWER_LEG_LENGTH = leg.lowerLegLength = objLeg["lower_leg_length"].as<float>();

        DarwinModel::Leg::HEEL_LENGTH = leg.heelLength = objLeg["heel_length"].as<float>();

        DarwinModel::Leg::LENGTH_BETWEEN_LEGS = leg.lengthBetweenLegs = leg.hipOffset[1] * 2;
        DarwinModel::Leg::FOOT_CENTRE_TO_ANKLE_CENTRE = leg.footCentreToAnkleCentre = objLeg["foot_centre_to_ankle_centre"].as<float>();

        auto& foot = leg.foot;
        auto& objFoot = objLeg["foot"];
        DarwinModel::Leg::FOOT_WIDTH = foot.width = objFoot["width"].as<float>();
        DarwinModel::Leg::FOOT_HEIGHT = foot.height = objFoot["height"].as<float>();
        DarwinModel::Leg::FOOT_LENGTH = foot.length = objFoot["length"].as<float>();
        DarwinModel::Leg::TOE_LENGTH = foot.toeLength = objFoot["toe_length"].as<float>();
    }

    void KinematicsModel::configureHead (DarwinKinematicsModel::Head& head, const YAML::Node& objHead) {
        DarwinModel::Head::CAMERA_DECLINATION_ANGLE_OFFSET = head.cameraDeclinationAngleOffset = objHead["camera_declination_angle_offset"].as<float>();

        head.neckToCamera = objHead["neck_to_camera"].as<arma::vec3>();
        DarwinModel::Head::NECK_TO_CAMERA_X = head.neckToCamera[0];
        DarwinModel::Head::NECK_TO_CAMERA_Y = head.neckToCamera[1];
        DarwinModel::Head::NECK_TO_CAMERA_Z = head.neckToCamera[2];

        auto& neck = head.neck;
        auto& objNeck = objHead["neck"];

        DarwinModel::Head::NECK_LENGTH = neck.length = objNeck["length"].as<float>();

        neck.basePositionFromOrigin = objNeck["base_position_from_origin"].as<arma::vec3>();
        DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_X = neck.basePositionFromOrigin[0];
        DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_Y = neck.basePositionFromOrigin[1];
        DarwinModel::Head::NECK_BASE_POS_FROM_ORIGIN_Z = neck.basePositionFromOrigin[2];

        auto& headMovementLimits = head.headMovementLimits;
        auto& objHeadMovementLimits = objHead["limits"];

        headMovementLimits.yaw = objHeadMovementLimits["yaw"].as<arma::vec2>();
        headMovementLimits.pitch = objHeadMovementLimits["pitch"].as<arma::vec2>();
        DarwinModel::Head::MIN_YAW = headMovementLimits.yaw[0];
        DarwinModel::Head::MAX_YAW = headMovementLimits.yaw[1];
        DarwinModel::Head::MIN_PITCH = headMovementLimits.pitch[0];
        DarwinModel::Head::MAX_PITCH = headMovementLimits.pitch[1];

    }

    void KinematicsModel::configureArm (DarwinKinematicsModel::Arm& arm, const YAML::Node& objArm) {
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

        DarwinModel::Arm::DISTANCE_BETWEEN_SHOULDERS = objArm["distance_between_shoulders"].as<float>();
        DarwinModel::Arm::SHOULDER_Z_OFFSET = objShoulder["offset"].as<arma::vec2>()[1];
        DarwinModel::Arm::SHOULDER_X_OFFSET = objShoulder["offset"].as<arma::vec2>()[0];
        DarwinModel::Arm::SHOULDER_LENGTH = objShoulder["length"].as<float>();
        DarwinModel::Arm::SHOULDER_WIDTH = objShoulder["width"].as<float>();
        DarwinModel::Arm::SHOULDER_HEIGHT = objShoulder["height"].as<float>();
        DarwinModel::Arm::UPPER_ARM_LENGTH = objUpperArm["length"].as<float>();
        DarwinModel::Arm::UPPER_ARM_Y_OFFSET = objUpperArm["offset"].as<arma::vec2>()[0];
        DarwinModel::Arm::UPPER_ARM_X_OFFSET = objUpperArm["offset"].as<arma::vec2>()[1];
        DarwinModel::Arm::LOWER_ARM_LENGTH = objLowerArm["length"].as<float>();
        DarwinModel::Arm::LOWER_ARM_Y_OFFSET = objLowerArm["offset"].as<arma::vec2>()[0];
        DarwinModel::Arm::LOWER_ARM_Z_OFFSET = objLowerArm["offset"].as<arma::vec2>()[1];
    }

    void KinematicsModel::configureMassModel (DarwinKinematicsModel::MassModel& massModel, const YAML::Node& objMassModel) {
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
        std::copy(masses.begin(), masses.end(), DarwinModel::MassModel::masses.begin());
    }

}
}
}
