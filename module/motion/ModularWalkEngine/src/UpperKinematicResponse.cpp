/*----------------------------------------------DOCUMENT HEADER----------------------------------------------*/
/*===========================================================================================================*/
/*
 * This file is part of ModularWalkEngine.
 *
 * ModularWalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ModularWalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ModularWalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "ModularWalkEngine.h"

#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
namespace module 
{
namespace motion 
{
    /*=======================================================================================================*/
    //      UTILIZATION REFERENCE(S)
    /*=======================================================================================================*/
    using message::input::LimbID;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::nubugger::graph;
    /*=======================================================================================================*/
    //      NAME: updateUpperBody
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
	std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::updateUpperBody(double phase, const Sensors& sensors) 
	{
		//Interpret robot's zero point reference from torso for positional transformation into relative space...
        uTorsoLocal = zmpTorsoCompensation(phase, zmpCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource);

        //Interpret robot's world position from torso as local positional reference...
        Transform2D uTorsoWorld = uTorsoLocal.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});

        //Collect attributed metrics that describe the robot's spatial orientation in environmental space...
        Transform3D torsoWorldMetrics = arma::vec6({uTorsoWorld.x(), uTorsoWorld.y(), bodyHeight, 0, bodyTilt, uTorsoWorld.angle()});

        //DEBUGGING: Emit relative torsoWorldMetrics position with respect to world model... 
        if (emitLocalisation) 
        {
            localise(uTorsoWorld);
        }
        //TODO: improve accuracy of compensation movement in upper body...
        return (motionArms(phase));
    }
    /*=======================================================================================================*/
    //      NAME: stepTorso
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    Transform2D ModularWalkEngine::stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor) 
    {
        Transform2D uLeftFootSupport  = uLeftFoot.localToWorld({-footOffset[0], -footOffset[1], 0});
        Transform2D uRightFootSupport = uRightFoot.localToWorld({-footOffset[0], footOffset[1], 0});
        return uLeftFootSupport.interpolate(shiftFactor, uRightFootSupport);
    }
    /*=======================================================================================================*/
    //      NAME: motionArms
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::motionArms(double phase) 
    {
        // Converts the phase into a sine wave that oscillates between 0 and 1 with a period of 2 phases
        double easing = std::sin(M_PI * phase - M_PI / 2.0) / 2.0 + 0.5;
        if (swingLeg == LimbID::LEFT_LEG) 
        {
            easing = -easing + 1.0; // Gets the 2nd half of the sine wave
        }

        // Linearly interpolate between the start and end positions using the easing parameter
        arma::vec3 qLArmActual = easing * qLArmStart + (1.0 - easing) * qLArmEnd;
        arma::vec3 qRArmActual = (1.0 - easing) * qRArmStart + easing * qRArmEnd;

        // Start arm/leg collision/prevention
        double rotLeftA = normalizeAngle(uLeftFoot.angle() - uTorso.angle());
        double rotRightA = normalizeAngle(uTorso.angle() - uRightFoot.angle());
        Transform2D leftLegTorso = uTorso.worldToLocal(uLeftFoot);
        Transform2D rightLegTorso = uTorso.worldToLocal(uRightFoot);
        double leftMinValue = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2 + std::max(0.0, leftLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2 - std::max(0.0, -rightLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        // update shoulder pitch to move arm away from body
        qLArmActual[1] = std::max(leftMinValue, qLArmActual[1]);
        qRArmActual[1] = std::min(rightMinValue, qRArmActual[1]);
        // End arm/leg collision/prevention

        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(6);

        NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);
        waypoints->push_back({ subsumptionId, time, ServoID::R_SHOULDER_PITCH, float(qRArmActual[0]), jointGains[ServoID::R_SHOULDER_PITCH], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::R_SHOULDER_ROLL,  float(qRArmActual[1]), jointGains[ServoID::R_SHOULDER_ROLL], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::R_ELBOW,          float(qRArmActual[2]), jointGains[ServoID::R_ELBOW], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_SHOULDER_PITCH, float(qLArmActual[0]), jointGains[ServoID::L_SHOULDER_PITCH], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_SHOULDER_ROLL,  float(qLArmActual[1]), jointGains[ServoID::L_SHOULDER_ROLL], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_ELBOW,          float(qLArmActual[2]), jointGains[ServoID::L_ELBOW], 100 });

        return std::move(waypoints);
    }
}  // motion
}  // modules