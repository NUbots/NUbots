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

#ifndef MODULES_PLATFORM_DARWIN_KINEMATICSMODEL_H
#define MODULES_PLATFORM_DARWIN_KINEMATICSMODEL_H

#include <nuclear>
#include <armadillo>
#include <yaml-cpp/yaml.h>

namespace modules {
namespace platform {
namespace darwin {

    class KinematicsModel : public NUClear::Reactor {

    public:

    	struct Foot {
    		float width;
    		float height;
    		float length;
    		float toeLength;
    	};

    	struct Leg {
    		arma::vec3 hipOffset;
    		float upperLegLength;
    		float lowerLegLength;
    		float heelLength;
    		float lengthBetweenLegs;
    		float footCentreToAnkleCentre;
    		Foot foot;
    	};

    	struct Neck {
    		float length;
    		arma::vec3 basePositionFromOrigin;
    	};

    	struct HeadMovementLimits {
    		arma::vec2 yaw;
    		arma::vec2 pitch;
    	};

    	struct Head {
    		float cameraDeclinationAngleOffset;
    		arma::vec3 neckToCamera;
    		Neck neck;
    		HeadMovementLimits headMovementLimits;
    	};

    	struct Shoulder {
    		float length;
    		float width;
    		float height;
    		arma::vec2 offset;
    	};

    	struct UpperArm {
    		float length;
    		arma::vec2 offset;
    	};

    	struct LowerArm {
    		float length;
    		arma::vec2 offset;
    	};

    	struct Arm {
    		float distanceBetweenShoulders;
    		Shoulder shoulder;
    		UpperArm upperArm;
    		LowerArm lowerArm;
    	};

    	struct DarwinModel {
    		Leg leg;
    		Head head;
    		Arm arm;
    		float teamDarwinChestToOrigin;
    	};

    	struct Masses {
    		arma::vec4 leftShoulderRoll;
    		arma::vec4 rightShoulderRoll;

    		arma::vec4 leftShoulderPitch;
    		arma::vec4 rightShoulderPitch;

    		arma::vec4 leftElbow;
    		arma::vec4 rightElbow;

    		arma::vec4 leftHipRoll;
    		arma::vec4 rightHipRoll;

    		arma::vec4 leftHipPitch;
    		arma::vec4 rightHipPitch;

    		arma::vec4 leftHipYaw;
    		arma::vec4 rightHipYaw;

    		arma::vec4 leftKnee;
    		arma::vec4 rightKnee;

    		arma::vec4 leftAnkleRoll;
    		arma::vec4 rightAnkleRoll;

    		arma::vec4 leftAnklePitch;
    		arma::vec4 rightAnklePitch;

    		arma::vec4 headPitch;
    		arma::vec4 headYaw;

    		arma::vec4 torso;
    	};

    	struct MassModel {
    		uint numberOfMasses;
    		uint massRepresentationDimension;
    		Masses masses;
    	};

        /// @brief Called by the powerplant to build and setup the KinematicsModel reactor.
        explicit KinematicsModel(std::unique_ptr<NUClear::Environment> environment);

        /// @brief the path to the configuration file for KinematicsModel
        static constexpr const char* CONFIGURATION_PATH = "KinematicsModel.yaml";

    private:

    	void configureDarwinModel (const YAML::Node& objDarwinModel);
    	void configureMassModel (const YAML::Node& objMassModel);

    	DarwinModel darwinModel;
    	MassModel massModel;

    };

}
}
}

#endif  // MODULES_PLATFORM_DARWIN_KINEMATICSMODEL_H
