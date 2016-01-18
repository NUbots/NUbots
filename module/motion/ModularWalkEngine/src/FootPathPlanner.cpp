/*----------------------------------------------DOCUMENT HEADER----------------------------------------------*/
//=============================================================================================================
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
//=============================================================================================================
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
//=============================================================================================================
//      INCLUDE(S)
//=============================================================================================================
#include "ModularWalkEngine.h"
#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
//=============================================================================================================
//      NAMESPACE(S)
//=============================================================================================================
namespace module 
{
namespace motion 
{
    //=========================================================================================================
    //      UTILIZATION REFERENCE(S)
    //=========================================================================================================
    using message::input::LimbID;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::nubugger::graph;
    //=========================================================================================================
    //      NAME: footPhase
    //=========================================================================================================
    //      Input  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Output : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Pre-condition  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Post-condition : <TODO: INSERT DESCRIPTION>
    //=========================================================================================================
    arma::vec3 ModularWalkEngine::footPhase(double phase, double phase1Single, double phase2Single) 
    {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, phaseSingle, zf};
    }
    //=========================================================================================================
    //      NAME: updateStep
    //=========================================================================================================
    //      Input  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Output : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Pre-condition  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Post-condition : <TODO: INSERT DESCRIPTION>
    //=========================================================================================================
    void ModularWalkEngine::updateStep(double phase, const Sensors& sensors) {
        //Get unitless phases for x and z motion
        arma::vec3 foot = footPhase(phase, phase1Single, phase2Single);

        //Lift foot by amount depending on walk speed
        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits); // TODO: use a function instead
        float speed = std::min(1.0, std::max(std::abs(velocityCurrent.x() / limit[0]), std::abs(velocityCurrent.y() / limit[1])));
        float scale = (step_height_fast_fraction - step_height_slow_fraction) * speed + step_height_slow_fraction;
        foot[2] *= scale;


        // don't lift foot at initial step, TODO: review
        if (initialStep > 0) {
            foot[2] = 0;
        }

        //Interpolate Transform2D from start to destination
        if (swingLeg == LimbID::RIGHT_LEG) {
            uRightFoot = uRightFootSource.interpolate(foot[0], uRightFootDestination);
        } else {
            uLeftFoot = uLeftFootSource.interpolate(foot[0], uLeftFootDestination);
        }
        //I hear you like arguments...
        uTorso = zmpCom(phase, zmpCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource);

        Transform3D leftFoot = uLeftFoot;
        Transform3D rightFoot = uRightFoot;

        //Lift swing leg
        if (swingLeg == LimbID::RIGHT_LEG) {
            rightFoot = rightFoot.translateZ(stepHeight * foot[2]);
        } else {
            leftFoot = leftFoot.translateZ(stepHeight * foot[2]);
        }

        Transform2D uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});
        Transform3D torso = arma::vec6({uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()});

        // Transform feet targets to be relative to the torso
        Transform3D leftFootTorso = leftFoot.worldToLocal(torso);
        Transform3D rightFootTorso = rightFoot.worldToLocal(torso);

        //TODO: what is this magic?
        double phaseComp = std::min({1.0, foot[1] / 0.1, (1 - foot[1]) / 0.1});

        // Rotate foot around hip by the given hip roll compensation
        if (swingLeg == LimbID::LEFT_LEG) {
            rightFootTorso = rightFootTorso.rotateZLocal(-hipRollCompensation * phaseComp, sensors.forwardKinematics.find(ServoID::R_HIP_ROLL)->second);
        }
        else {
            leftFootTorso = leftFootTorso.rotateZLocal(hipRollCompensation * phaseComp, sensors.forwardKinematics.find(ServoID::L_HIP_ROLL)->second);
        }

        //TODO:is this a Debug?
        if (emitLocalisation) {
            localise(uTorsoActual);
        }

        if (balanceEnabled) {
            // Apply balance to our support foot
            balancer.balance(swingLeg == LimbID::LEFT_LEG ? rightFootTorso : leftFootTorso
                , swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG
                , sensors);
        }

        // emit(graph("Right foot pos", rightFootTorso.translation()));
        // emit(graph("Left foot pos", leftFootTorso.translation()));

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
        auto waypoints = motionLegs(joints);

        auto arms = motionArms(phase);
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        emit(std::move(waypoints));
    }
}  // motion
}  // modules