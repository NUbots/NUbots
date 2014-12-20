/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "WalkEngine.h"

#include "utility/motion/RobotModels.h"

namespace modules {
namespace motion {

    using messages::behaviour::LimbID;
    using utility::motion::kinematics::DarwinModel;

    void WalkEngine::calculateNewStep() {
        updateVelocity();

        // swap swing and support legs
        swingLeg = swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;

        uLeftFootSource = uLeftFootDestination;
        uRightFootSource = uRightFootDestination;
        uTorsoSource = uTorsoDestination;

        arma::vec2 supportMod = {0, 0}; // support point modulation for wallkick

        if (state == State::STOP_REQUEST) {
            log<NUClear::TRACE>("Walk Engine:: Stop requested");
            state = State::LAST_STEP;
            velocityCurrent = {0, 0, 0};
            velocityCommand = {0, 0, 0};
            if (swingLeg == LimbID::RIGHT_LEG) {
                uRightFootDestination = localToWorld(-2 * uLRFootOffset, uLeftFootSource);
            } else {
                uLeftFootDestination = localToWorld(2 * uLRFootOffset, uRightFootSource);
            }
        } else {
            // normal walk, advance steps
            if (swingLeg == LimbID::RIGHT_LEG) {
                uRightFootDestination = stepRightFootDestination(velocityCurrent, uLeftFootSource, uRightFootSource);
            } else {
                uLeftFootDestination = stepLeftFootDestination(velocityCurrent, uLeftFootSource, uRightFootSource);
            }

            // velocity-based support point modulation
            /*toeTipCompensation = 0;
            if (velocityDifference[0] > 0) {
                // accelerating to front
                supportMod[0] = supportFront2;
            } else if (velocityCurrent[0] > velFastForward) {
                supportMod[0] = supportFront;
                toeTipCompensation = ankleMod[0];
            } else if (velocityCurrent[0] < 0) {
                supportMod[0] = supportBack;
            } else if (std::abs(velocityCurrent[2]) > velFastTurn) {
                supportMod[0] = supportTurn;
            } else {
                if (velocityCurrent[1] > 0.015) {
                    supportMod[0] = supportSideX;
                    supportMod[1] = supportSideY;
                } else if (velocityCurrent[1] < -0.015) {
                    supportMod[0] = supportSideX;
                    supportMod[1] = -supportSideY;
                }
            }*/
        }

        uTorsoDestination = stepTorso(uLeftFootDestination, uRightFootDestination, 0.5);

        // apply velocity-based support point modulation for uSupport
        if (swingLeg == LimbID::RIGHT_LEG) {
            arma::vec3 uLeftFootTorso = worldToLocal(uLeftFootSource, uTorsoSource);
            arma::vec3 uTorsoModded = localToWorld({supportMod[0], supportMod[1], 0}, uTorso);
            arma::vec3 uLeftFootModded = localToWorld(uLeftFootTorso, uTorsoModded);
            uSupport = localToWorld({-footOffset[0], -footOffset[1], 0}, uLeftFootModded);
        } else {
            arma::vec3 uRightFootTorso = worldToLocal(uRightFootSource, uTorso);
            arma::vec3 uTorsoModded = localToWorld({supportMod[0], supportMod[1], 0}, uTorso);
            arma::vec3 uRightFootModded = localToWorld(uRightFootTorso, uTorsoModded);
            uSupport = localToWorld({-footOffset[0], footOffset[1], 0}, uRightFootModded);
        }

        // compute ZMP coefficients
        zmpParams = {
            (uSupport[0] - uTorso[0]) / (stepTime * phase1Single),
            (uTorsoDestination[0] - uSupport[0]) / (stepTime * (1 - phase2Single)),
            (uSupport[1] - uTorso[1]) / (stepTime * phase1Single),
            (uTorsoDestination[1] - uSupport[1]) / (stepTime * (1 - phase2Single)),
        };

        zmpCoefficients.rows(0,1) = zmpSolve(uSupport[0], uTorsoSource[0], uTorsoDestination[0], uTorsoSource[0], uTorsoDestination[0], phase1Single, phase2Single, stepTime, zmpTime);
        zmpCoefficients.rows(2,3) = zmpSolve(uSupport[1], uTorsoSource[1], uTorsoDestination[1], uTorsoSource[1], uTorsoDestination[1], phase1Single, phase2Single, stepTime, zmpTime);
    }

    void WalkEngine::updateVelocity() {
        // slow accelerations at high speed
        auto& limit = (velocityCurrent[0] > velocityHigh ? accelerationLimitsHigh : accelerationLimits); // TODO: use a function instead

        velocityDifference[0] = std::min(std::max(velocityCommand[0] - velocityCurrent[0], -limit[0]), limit[0]);
        velocityDifference[1] = std::min(std::max(velocityCommand[1] - velocityCurrent[1], -limit[1]), limit[1]);
        velocityDifference[2] = std::min(std::max(velocityCommand[2] - velocityCurrent[2], -limit[2]), limit[2]);

        velocityCurrent[0] += velocityDifference[0];
        velocityCurrent[1] += velocityDifference[1];
        velocityCurrent[2] += velocityDifference[2];

        if (initialStep > 0) {
            velocityCurrent = arma::vec3{0, 0, 0};
            initialStep--;
        }
    }



    /**
     * Globals: uLRFootOffset, footSizeX, stanceLimitY, stanceLimitY2, stanceLimitAngle
     */
    arma::vec3 WalkEngine::stepLeftFootDestination(arma::vec3 velocity, arma::vec3 uLeftFoot, arma::vec3 uRightFoot) {
        // Get midpoint between the two feet
        arma::vec3 midPoint = se2Interpolate(0.5, uLeftFoot, uRightFoot);
        // Get midpoint 1.5 steps in future
        arma::vec3 forwardPoint = localToWorld(1.5 * velocity, midPoint);
        // Offset to towards the foot in use to get the target location
        arma::vec3 leftFootTarget = localToWorld(uLRFootOffset, forwardPoint);

        // Start foot collision detection/prevention
        arma::vec3 uLeftFootRight = worldToLocal(leftFootTarget, uRightFoot);
        // Do not pidgeon toe, cross feet:
        // Check toe and heel overlap
        double toeOverlap = -DarwinModel::Leg::FOOT_LENGTH / 2.0 * uLeftFootRight[2];
        double heelOverlap = DarwinModel::Leg::FOOT_LENGTH / 2.0 * uLeftFootRight[2];
        double limitY = std::max(stepLimits(1,0), stanceLimitY2 + std::max(toeOverlap, heelOverlap));
        uLeftFootRight[0] = std::min(std::max(uLeftFootRight[0], stepLimits(0,0)), stepLimits(0,1));
        uLeftFootRight[1] = std::min(std::max(uLeftFootRight[1], limitY), stepLimits(1,1));
        uLeftFootRight[2] = std::min(std::max(uLeftFootRight[2], stepLimits(2,0)), stepLimits(2,1));
        leftFootTarget = localToWorld(uLeftFootRight, uRightFoot);
        // End foot collision detection/prevention

        return leftFootTarget;
    }

    arma::vec3 WalkEngine::stepRightFootDestination(arma::vec3 velocity, arma::vec3 uLeftFoot, arma::vec3 uRightFoot) {
        // Get midpoint between the two feet
        arma::vec3 midPoint = se2Interpolate(0.5, uLeftFoot, uRightFoot);
        // Get midpoint 1.5 steps in future
        arma::vec3 forwardPoint = localToWorld(1.5 * velocity, midPoint);
        // Offset to towards the foot in use to get the target location
        arma::vec3 rightFootTarget = localToWorld(-uLRFootOffset, forwardPoint);

        // Start foot collision detection/prevention
        arma::vec3 uRightFootLeft = worldToLocal(rightFootTarget, uLeftFoot);
        // Do not pidgeon toe, cross feet:
        // Check toe and heel overlap
        double toeOverlap = DarwinModel::Leg::FOOT_LENGTH / 2.0 * uRightFootLeft[2];
        double heelOverlap = -DarwinModel::Leg::FOOT_LENGTH / 2.0 * uRightFootLeft[2];
        double limitY = std::max(stepLimits(1,0), stanceLimitY2 + std::max(toeOverlap, heelOverlap));
        uRightFootLeft[0] = std::min(std::max(uRightFootLeft[0], stepLimits(0,0)), stepLimits(0,1));
        uRightFootLeft[1] = std::min(std::max(uRightFootLeft[1], -stepLimits(1,1)), -limitY);
        uRightFootLeft[2] = std::min(std::max(uRightFootLeft[2], -stepLimits(2,1)), -stepLimits(2,0));
        rightFootTarget = localToWorld(uRightFootLeft, uLeftFoot);
        // End foot collision detection/prevention

        return rightFootTarget;
    }

    /**
    * Global variables used:
    * phase1Single, phase2Single
    */
    arma::vec3 WalkEngine::footPhase(double phase) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, 0, zf};
    }

}
}