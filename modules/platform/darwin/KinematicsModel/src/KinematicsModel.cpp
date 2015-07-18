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

#include "messages/support/Configuration.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace modules {
namespace platform {
namespace darwin {

    using messages::support::Configuration;
    using utility::support::Expression;

    using messages::platform::darwin::DarwinKinematicsModel;

    KinematicsModel::KinematicsModel(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<KinematicsModel>>>([this] (const Configuration<KinematicsModel>& config) {
        	DarwinKinematicsModel model;

        	model.dimensions = configureDimensions(config["dimensions"]);
        	model.massModel = configureMassModel(config["mass_model"]);

			emit(std::make_unique<DarwinKinematicsModel>(model));
        });
    }

    DarwinKinematicsModel::Dimensions KinematicsModel::configureDimensions (const YAML::Node& objDarwinModel) {
    	DarwinKinematicsModel::Dimensions darwinModel;

        auto& leg = darwinModel.leg;
        auto& objLeg = objDarwinModel["leg"];
        leg.hipOffset = objLeg["hip_offset"].as<arma::vec3>();
		leg.upperLegLength = objLeg["upper_leg_length"].as<Expression>();
		leg.lowerLegLength = objLeg["lower_leg_length"].as<Expression>();
		leg.heelLength = objLeg["heel_length"].as<Expression>();
		leg.lengthBetweenLegs = leg.hipOffset[1] * 2;
		leg.footCentreToAnkleCentre = objLeg["foot_centre_to_ankle_centre"].as<Expression>();

		auto& foot = leg.foot;
		auto& objFoot = objLeg["foot"];
		foot.width = objFoot["width"].as<Expression>();
		foot.height = objFoot["height"].as<Expression>();
		foot.length = objFoot["length"].as<Expression>();
		foot.toeLength = objFoot["toe_length"].as<Expression>();

        auto& head = darwinModel.head;
        auto& objHead = objDarwinModel["head"];
        head.cameraDeclinationAngleOffset = objHead["camera_declination_angle_offset"].as<Expression>();
        head.neckToCamera = objHead["neck_to_camera"].as<arma::vec3>();

        auto& neck = head.neck;
        auto& objNeck = objHead["neck"];
        neck.length = objNeck["length"].as<Expression>();
		neck.basePositionFromOrigin = objNeck["base_position_from_origin"].as<arma::vec3>();

		auto& headMovementLimits = head.headMovementLimits;
        auto& objHeadMovementLimits = objHead["limits"];
		headMovementLimits.yaw = objHeadMovementLimits["yaw"].as<arma::vec2>();
		headMovementLimits.pitch = objHeadMovementLimits["pitch"].as<arma::vec2>();
        
        auto& arm = darwinModel.arm;
        auto& objArm = objDarwinModel["arm"];
        arm.distanceBetweenShoulders = objArm["distance_between_shoulders"].as<Expression>();

		auto& shoulder = arm.shoulder;
		auto& objShoulder = objArm["shoulder"];
		shoulder.length = objShoulder["length"].as<Expression>();
		shoulder.width = objShoulder["width"].as<Expression>();
		shoulder.height = objShoulder["height"].as<Expression>();
		shoulder.offset = objShoulder["offset"].as<arma::vec2>();

		auto& upperArm = arm.upperArm;
		auto& objUpperArm = objArm["upper_arm"];
		upperArm.length = objUpperArm["length"].as<Expression>();
		upperArm.offset = objUpperArm["offset"].as<arma::vec2>();

		auto& lowerArm = arm.lowerArm;
		auto& objLowerArm = objArm["lower_arm"];
		lowerArm.length = objLowerArm["length"].as<Expression>();
		lowerArm.offset = objLowerArm["offset"].as<arma::vec2>();

		darwinModel.teamDarwinChestToOrigin = 0.096 - leg.hipOffset[2];

		return darwinModel;
    }

    DarwinKinematicsModel::MassModel KinematicsModel::configureMassModel (const YAML::Node& objMassModel) {
    	DarwinKinematicsModel::MassModel massModel;

    	massModel.numberOfMasses = objMassModel["number_of_masses"].as<Expression>();
        massModel.massRepresentationDimension = objMassModel["mass_representation_dimension"].as<Expression>();

        auto& masses = massModel.masses;
        auto& objMasses = objMassModel["masses"];

        masses.leftShoulderRoll = objMasses[3].as<arma::vec4>();
        masses.rightShoulderRoll = objMasses[2].as<arma::vec4>();
        masses.leftShoulderPitch = objMasses[1].as<arma::vec4>();
        masses.rightShoulderPitch = objMasses[0].as<arma::vec4>();

		masses.leftElbow = objMasses[5].as<arma::vec4>();
        masses.rightElbow = objMasses[4].as<arma::vec4>();

        masses.leftHipRoll = objMasses[9].as<arma::vec4>();
        masses.rightHipRoll = objMasses[8].as<arma::vec4>();
        masses.leftHipPitch = objMasses[11].as<arma::vec4>();
        masses.rightHipPitch = objMasses[10].as<arma::vec4>();
        masses.leftHipYaw = objMasses[7].as<arma::vec4>();
        masses.rightHipYaw = objMasses[6].as<arma::vec4>();
        
        masses.leftKnee = objMasses[13].as<arma::vec4>();
        masses.rightKnee = objMasses[12].as<arma::vec4>();
        
        masses.leftAnkleRoll = objMasses[17].as<arma::vec4>();
        masses.rightAnkleRoll = objMasses[16].as<arma::vec4>();
        masses.leftAnklePitch = objMasses[15].as<arma::vec4>();
        masses.rightAnklePitch = objMasses[14].as<arma::vec4>();

        masses.headPitch = objMasses[19].as<arma::vec4>();
        masses.headYaw = objMasses[18].as<arma::vec4>();

		masses.torso = objMasses[20].as<arma::vec4>();

		return massModel;
    }

}
}
}
