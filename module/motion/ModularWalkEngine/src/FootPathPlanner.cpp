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
    //      NAME: calculateNewStep
    //=========================================================================================================
    //      Input  : null
    /*-------------------------------------------------------------------------------------------------------*/
    //      Output : void
    /*-------------------------------------------------------------------------------------------------------*/
    //      Pre-condition  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Post-condition : <TODO: INSERT DESCRIPTION>
    //=========================================================================================================
    void ModularWalkEngine::calculateNewStep() 
    {
        updateVelocity();

        // swap swing and support legs
        swingLeg = swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;

        uLeftFootSource = uLeftFootDestination;
        uRightFootSource = uRightFootDestination;
        uTorsoSource = uTorsoDestination;

        arma::vec2 supportMod = arma::zeros(2); // support point modulation for wallkick

        if (state == State::STOP_REQUEST) 
        {
            log<NUClear::TRACE>("Walk Engine:: Stop requested");
            state = State::LAST_STEP;
            velocityCurrent = arma::zeros(3);
            velocityCommand = arma::zeros(3);

            // Stop with feet together by targetting swing leg next to support leg
            if (swingLeg == LimbID::RIGHT_LEG) 
            {
                uRightFootDestination = uLeftFootSource.localToWorld(-2 * uLRFootOffset);
            }
            else 
            {
                uLeftFootDestination = uRightFootSource.localToWorld( 2 * uLRFootOffset);
            }
        }
        else 
        {
            // normal walk, advance steps
            if (swingLeg == LimbID::RIGHT_LEG) 
            {
                uRightFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
            }
            else 
            {
                uLeftFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
            }

            // velocity-based support point modulation
            /*toeTipCompensation = 0;
            if (velocityDifference[0] > 0) 
            {
                // accelerating to front
                supportMod[0] = supportFront2;
            }
            else if (velocityCurrent[0] > velFastForward) 
            {
                supportMod[0] = supportFront;
                toeTipCompensation = ankleMod[0];
            }
            else if (velocityCurrent[0] < 0) 
            {
                supportMod[0] = supportBack;
            }
            else if (std::abs(velocityCurrent[2]) > velFastTurn) 
            {
                supportMod[0] = supportTurn;
            }
            else 
            {
                if (velocityCurrent[1] > 0.015) 
                {
                    supportMod[0] = supportSideX;
                    supportMod[1] = supportSideY;
                }
                else if (velocityCurrent[1] < -0.015) 
                {
                    supportMod[0] = supportSideX;
                    supportMod[1] = -supportSideY;
                }
            }*/
        }

        uTorsoDestination = stepTorso(uLeftFootDestination, uRightFootDestination, 0.5);

        // apply velocity-based support point modulation for uSupport
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            Transform2D uLeftFootTorso = uTorsoSource.worldToLocal(uLeftFootSource);
            Transform2D uTorsoModded = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uLeftFootModded = uTorsoModded.localToWorld(uLeftFootTorso);
            uSupport = uLeftFootModded.localToWorld({-footOffset[0], -footOffset[1], 0});
        }
        else 
        {
            Transform2D uRightFootTorso = uTorsoSource.worldToLocal(uRightFootSource);
            Transform2D uTorsoModded = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uRightFootModded = uTorsoModded.localToWorld(uRightFootTorso);
            uSupport = uRightFootModded.localToWorld({-footOffset[0], footOffset[1], 0});
        }

        // compute ZMP coefficients
        zmpParams = 
        {
            (uSupport.x() - uTorso.x()) / (stepTime * phase1Single),
            (uTorsoDestination.x() - uSupport.x()) / (stepTime * (1 - phase2Single)),
            (uSupport.y() - uTorso.y()) / (stepTime * phase1Single),
            (uTorsoDestination.y() - uSupport.y()) / (stepTime * (1 - phase2Single)),
        };

        zmpCoefficients.rows(0,1) = zmpSolve(uSupport.x(), uTorsoSource.x(), uTorsoDestination.x(), uTorsoSource.x(), uTorsoDestination.x(), phase1Single, phase2Single, stepTime, zmpTime);
        zmpCoefficients.rows(2,3) = zmpSolve(uSupport.y(), uTorsoSource.y(), uTorsoDestination.y(), uTorsoSource.y(), uTorsoDestination.y(), phase1Single, phase2Single, stepTime, zmpTime);
    }
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
    //      NAME: updateVelocity
    //=========================================================================================================
    //      Input  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Output : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Pre-condition  : <TODO: INSERT DESCRIPTION>
    /*-------------------------------------------------------------------------------------------------------*/
    //      Post-condition : <TODO: INSERT DESCRIPTION>
    //=========================================================================================================
    void ModularWalkEngine::updateVelocity() 
    {
        // slow accelerations at high speed
        auto now = NUClear::clock::now();
        double deltaT = std::chrono::duration_cast<std::chrono::microseconds>(now - lastVeloctiyUpdateTime).count() * 1e-6;
        lastVeloctiyUpdateTime = now;

        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits) * deltaT; // TODO: use a function instead

        velocityDifference.x()     = std::min(std::max(velocityCommand.x()     - velocityCurrent.x(),     -limit[0]), limit[0]);
        velocityDifference.y()     = std::min(std::max(velocityCommand.y()     - velocityCurrent.y(),     -limit[1]), limit[1]);
        velocityDifference.angle() = std::min(std::max(velocityCommand.angle() - velocityCurrent.angle(), -limit[2]), limit[2]);

        velocityCurrent.x()     += velocityDifference.x();
        velocityCurrent.y()     += velocityDifference.y();
        velocityCurrent.angle() += velocityDifference.angle();

        if (initialStep > 0) 
        {
            velocityCurrent = arma::zeros(3);
            initialStep--;
        }
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