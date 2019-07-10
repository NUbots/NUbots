/*
 * This file is part of OldWalkEngine.
 *
 * OldWalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OldWalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OldWalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "OldWalkEngine.h"

#include "message/motion/KinematicsModel.h"
#include "utility/nusight/NUhelpers.h"

namespace module {
namespace motion {

    using LimbID = utility::input::LimbID;
    using message::motion::KinematicsModel;

    using utility::math::matrix::Transform2D;
    using utility::nusight::graph;

    void OldWalkEngine::calculateNewStep() {
        updateVelocity();

        // swap swing and support legs
        swingLeg = swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;

        uLeftFootSource  = uLeftFootDestination;
        uRightFootSource = uRightFootDestination;
        uTorsoSource     = uTorsoDestination;

        arma::vec2 supportMod = arma::zeros(2);  // support point modulation for wallkick

        if (state == State::STOP_REQUEST) {
            log<NUClear::TRACE>("Walk Engine:: Stop requested");
            state           = State::LAST_STEP;
            velocityCurrent = arma::zeros(3);
            velocityCommand = arma::zeros(3);

            // Stop with feet together by targetting swing leg next to support leg
            if (swingLeg == LimbID::RIGHT_LEG) {
                uRightFootDestination = uLeftFootSource.localToWorld(-2 * uLRFootOffset);
            }
            else {
                uLeftFootDestination = uRightFootSource.localToWorld(2 * uLRFootOffset);
            }
        }
        else {
            // normal walk, advance steps
            if (swingLeg == LimbID::RIGHT_LEG) {
                uRightFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
            }
            else {
                uLeftFootDestination = getNewFootTarget(velocityCurrent, uLeftFootSource, uRightFootSource, swingLeg);
            }
        }

        uTorsoDestination = stepTorso(uLeftFootDestination, uRightFootDestination, 0.5);

        // apply velocity-based support point modulation for uSupport
        if (swingLeg == LimbID::RIGHT_LEG) {
            Transform2D uLeftFootTorso  = uTorsoSource.worldToLocal(uLeftFootSource);
            Transform2D uTorsoModded    = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uLeftFootModded = uTorsoModded.localToWorld(uLeftFootTorso);
            uSupport                    = uLeftFootModded.localToWorld({-footOffset[0], -footOffset[1], 0});
        }
        else {
            Transform2D uRightFootTorso  = uTorsoSource.worldToLocal(uRightFootSource);
            Transform2D uTorsoModded     = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uRightFootModded = uTorsoModded.localToWorld(uRightFootTorso);
            uSupport                     = uRightFootModded.localToWorld({-footOffset[0], footOffset[1], 0});
        }

        // compute ZMP coefficients
        zmpParams = {
            (uSupport.x() - uTorso.x()) / (stepTime * phase1Single),
            (uTorsoDestination.x() - uSupport.x()) / (stepTime * (1 - phase2Single)),
            (uSupport.y() - uTorso.y()) / (stepTime * phase1Single),
            (uTorsoDestination.y() - uSupport.y()) / (stepTime * (1 - phase2Single)),
        };

        zmpCoefficients.rows(0, 1) = zmpSolve(uSupport.x(),
                                              uTorsoSource.x(),
                                              uTorsoDestination.x(),
                                              uTorsoSource.x(),
                                              uTorsoDestination.x(),
                                              phase1Single,
                                              phase2Single,
                                              stepTime,
                                              zmpTime);
        zmpCoefficients.rows(2, 3) = zmpSolve(uSupport.y(),
                                              uTorsoSource.y(),
                                              uTorsoDestination.y(),
                                              uTorsoSource.y(),
                                              uTorsoDestination.y(),
                                              phase1Single,
                                              phase2Single,
                                              stepTime,
                                              zmpTime);
    }

    void OldWalkEngine::updateVelocity() {
        // slow accelerations at high speed
        auto now = NUClear::clock::now();
        double deltaT =
            std::chrono::duration_cast<std::chrono::microseconds>(now - lastVeloctiyUpdateTime).count() * 1e-6;
        lastVeloctiyUpdateTime = now;

        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits)
                      * deltaT;  // TODO: use a function instead


        velocityDifference.x() = std::min(std::max(velocityCommand.x() - velocityCurrent.x(), -limit[0]), limit[0]);
        velocityDifference.y() = std::min(std::max(velocityCommand.y() - velocityCurrent.y(), -limit[1]), limit[1]);
        velocityDifference.angle() =
            std::min(std::max(velocityCommand.angle() - velocityCurrent.angle(), -limit[2]), limit[2]);

        velocityCurrent.x() += velocityDifference.x();
        velocityCurrent.y() += velocityDifference.y();
        velocityCurrent.angle() += velocityDifference.angle();

        if (initialStep > 0) {
            velocityCurrent = arma::zeros(3);
            initialStep--;
        }
    }

    Transform2D OldWalkEngine::getNewFootTarget(const Transform2D& velocity,
                                                const Transform2D& leftFoot,
                                                const Transform2D& rightFoot,
                                                const LimbID& swingLeg) {
        // Negative if right leg to account for the mirroring of the foot target
        int8_t sign = swingLeg == LimbID::LEFT_LEG ? 1 : -1;
        // Get midpoint between the two feet
        Transform2D midPoint = leftFoot.interpolate(0.5, rightFoot);
        // Get midpoint 1.5 steps in future
        // Note: The reason for 1.5 rather than 1 is because it takes an extra 0.5 steps
        // for the torso to reach a given position when you want both feet together
        Transform2D forwardPoint = midPoint.localToWorld(1.5 * velocity);
        // Offset to towards the foot in use to get the target location
        Transform2D footTarget = forwardPoint.localToWorld(sign * uLRFootOffset);

        // Start applying step limits:
        // Get the vector between the feet and clamp the components between the min and max step limits
        Transform2D supportFoot    = swingLeg == LimbID::LEFT_LEG ? rightFoot : leftFoot;
        Transform2D feetDifference = supportFoot.worldToLocal(footTarget);
        feetDifference.x()         = std::min(std::max(feetDifference.x(), stepLimits(0, 0)), stepLimits(0, 1));
        feetDifference.y() = std::min(std::max(feetDifference.y() * sign, stepLimits(1, 0)), stepLimits(1, 1)) * sign;
        feetDifference.angle() =
            std::min(std::max(feetDifference.angle() * sign, stepLimits(2, 0)), stepLimits(2, 1)) * sign;
        // end applying step limits

        // Start feet collision detection:
        // Uses a rough measure to detect collision and move feet apart if too close
        double length_factor = kinematicsModel.leg.TOE_LENGTH;
        double width_factor  = kinematicsModel.leg.FOOT_WIDTH * 0.5;
        // Shift from foot center to ankle center
        if (LimbID::RIGHT_LEG) {
            width_factor += kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE;
        }
        else {
            width_factor -= kinematicsModel.leg.FOOT_CENTRE_TO_ANKLE_CENTRE;
        }

        double overlap = std::sqrt(length_factor * length_factor + width_factor * width_factor)
                             * std::atan(width_factor / length_factor)
                         + feetDifference.angle();
        feetDifference.y() = std::max(feetDifference.y() * sign, stanceLimitY2 + overlap) * sign;
        // End feet collision detection

        // Update foot target to be 'feetDistance' away from the support foot
        footTarget = supportFoot.localToWorld(feetDifference);

        return footTarget;
    }

    arma::vec3 OldWalkEngine::footPhase(double phase, double phase1Single, double phase2Single) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle     = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf              = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf              = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, phaseSingle, zf};
    }
}  // namespace motion
}  // namespace module
