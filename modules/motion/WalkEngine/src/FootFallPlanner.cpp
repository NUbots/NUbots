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
                uRightFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
            } else {
                uLeftFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
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

    arma::vec3 WalkEngine::getNewFootTarget(const arma::vec3& velocity, const arma::vec3& leftFoot, const arma::vec3& rightFoot, const LimbID& swingLeg) {
        // Negative if right leg to account for the mirroring of the foot target
        int8_t sign = swingLeg == LimbID::LEFT_LEG ? 1 : -1;
        // Get midpoint between the two feet
        arma::vec3 midPoint = se2Interpolate(0.5, leftFoot, rightFoot);
        // Get midpoint 1.5 steps in future
        // Note: The reason for 1.5 rather than 1 is because it takes an extra 0.5 steps
        // for the torso to reach a given position when you want both feet together
        arma::vec3 forwardPoint = localToWorld(1.5 * velocity, midPoint);
        // Offset to towards the foot in use to get the target location
        arma::vec3 footTarget = localToWorld(sign * uLRFootOffset, forwardPoint);

        // Start applying step limits:
        // Get the vector between the feet and clamp the components between the min and max step limits
        arma::vec3 supportFoot = swingLeg == LimbID::LEFT_LEG ? rightFoot : leftFoot;
        arma::vec3 feetDifference = worldToLocal(footTarget, supportFoot);
        feetDifference[0] = std::min(std::max(feetDifference[0],        stepLimits(0,0)), stepLimits(0,1));
        feetDifference[1] = std::min(std::max(feetDifference[1] * sign, stepLimits(1,0)), stepLimits(1,1)) * sign;
        feetDifference[2] = std::min(std::max(feetDifference[2] * sign, stepLimits(2,0)), stepLimits(2,1)) * sign;
        // end applying step limits

        // Start feet collision detection:
        // Uses a rough measure to detect collision and move feet apart if too close
        double overlap = DarwinModel::Leg::FOOT_LENGTH / 2.0 * std::abs(feetDifference[2]);
        feetDifference[1] = std::max(feetDifference[1] * sign, stanceLimitY2 + overlap) * sign;
        // End feet collision detection

        // Update foot target to be 'feetDistance' away from the support foot
        footTarget = localToWorld(feetDifference, supportFoot);

        return footTarget;
    }

    arma::vec3 WalkEngine::footPhase(double phase, double phase1Single, double phase2Single) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, 0, zf};
    }

}
}