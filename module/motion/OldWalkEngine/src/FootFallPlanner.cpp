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
    using utility::math::transform::worldToLocal;
    using utility::math::transform::localToWorld;
    using utility::math::transform::interpolate;
    using utility::math::transform::angle;

    using utility::nusight::graph;

    void OldWalkEngine::calculateNewStep() {
        updateVelocity();

        // swap swing and support legs
        swingLeg = swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;

        uLeftFootSource  = uLeftFootDestination;
        uRightFootSource = uRightFootDestination;
        uTorsoSource     = uTorsoDestination;

        Eigen::Vector2d supportMod = Eigen::Vector2d::Zero();  // support point modulation for wallkick

        if (state == State::STOP_REQUEST) {
            log<NUClear::TRACE>("Walk Engine:: Stop requested");
            state           = State::LAST_STEP;

            velocityCurrent = Eigen::Affine2d::Identity();
            velocityCommand = Eigen::Affine2d::Identity();

            // Stop with feet together by targetting swing leg next to support leg
            if (swingLeg == LimbID::RIGHT_LEG) {
                Eigen::Affine2d temp;
                temp.translation() = -2 * uLRFootOffset.translation();
                temp.linear() = Eigen::Rotation2Dd(-2 * angle(uLRFootOffset)).toRotationMatrix();
                uRightFootDestination = localToWorld(uLeftFootSource, temp);
            }
            else {
                Eigen::Affine2d temp;
                temp.translation() = 2 * uLRFootOffset.translation();
                temp.linear() = Eigen::Rotation2Dd(2 * angle(uLRFootOffset)).toRotationMatrix();
                uLeftFootDestination = localToWorld(uRightFootSource, temp);
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

        Eigen::Affine2d supportModTranslate = Eigen::Affine2d::Identity();
        supportModTranslate.translation() = supportMod;

        Eigen::Affine2d footOffsetNeg = Eigen::Affine2d::Identity();
        footOffsetNeg.translation() = -footOffset;

        // apply velocity-based support point modulation for uSupport
        if (swingLeg == LimbID::RIGHT_LEG) {
            Eigen::Affine2d uLeftFootTorso  = worldToLocal(uTorsoSource, uLeftFootSource);
            Eigen::Affine2d uTorsoModded    = localToWorld(uTorso, supportModTranslate);
            Eigen::Affine2d uLeftFootModded = localToWorld(uTorsoModded, uLeftFootTorso);
            uSupport                    = localToWorld(uLeftFootModded, footOffsetNeg);
        }
        else {
            Eigen::Affine2d uRightFootTorso  = worldToLocal(uTorsoSource, uRightFootSource);

            Eigen::Affine2d uTorsoModded     = localToWorld(uTorso, supportModTranslate);
            Eigen::Affine2d uRightFootModded = localToWorld(uTorsoModded, uRightFootTorso);
            uSupport                     = localToWorld(uRightFootModded, footOffsetNeg);
        }

        // compute ZMP coefficients
        zmpParams = {
            (uSupport.translation().x() - uTorso.translation().x()) / (stepTime * phase1Single),
            (uTorsoDestination.translation().x() - uSupport.translation().x()) / (stepTime * (1 - phase2Single)),
            (uSupport.translation().y() - uTorso.translation().y()) / (stepTime * phase1Single),
            (uTorsoDestination.translation().y() - uSupport.translation().y()) / (stepTime * (1 - phase2Single)),
        };

        zmpCoefficients.block(0, 0, 1, 0) = zmpSolve(uSupport.translation().x(),
                                              uTorsoSource.translation().x(),
                                              uTorsoDestination.translation().x(),
                                              uTorsoSource.translation().x(),
                                              uTorsoDestination.translation().x(),
                                              phase1Single,
                                              phase2Single,
                                              stepTime,
                                              zmpTime);
        zmpCoefficients.block(2, 0, 3, 0) = zmpSolve(uSupport.translation().y(),
                                              uTorsoSource.translation().y(),
                                              uTorsoDestination.translation().y(),
                                              uTorsoSource.translation().y(),
                                              uTorsoDestination.translation().y(),
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

        auto& limit = (velocityCurrent.translation()[0] > velocityHigh ? accelerationLimitsHigh : accelerationLimits)
                      * deltaT;  // TODO: use a function instead


        velocityDifference.translation().x() = std::min(std::max(velocityCommand.translation().x() - velocityCurrent.translation().x(), -limit[0]), limit[0]);
        velocityDifference.translation().y() = std::min(std::max(velocityCommand.translation().y() - velocityCurrent.translation().y(), -limit[1]), limit[1]);
        velocityDifference.linear() = 
            Eigen::Rotation2Dd(std::min(std::max(angle(velocityCommand) - angle(velocityCurrent), -limit[2]), limit[2])).toRotationMatrix();

        velocityCurrent.translation().x() += velocityDifference.translation().x();
        velocityCurrent.translation().y() += velocityDifference.translation().y();
        velocityCurrent.linear() = velocityCurrent.rotation() * velocityDifference.rotation();
        
        if (initialStep > 0) {
            velocityCurrent = Eigen::Affine2d::Identity();
            initialStep--;
        }
    }

    Eigen::Affine2d OldWalkEngine::getNewFootTarget(const Eigen::Affine2d& velocity,
                                                const Eigen::Affine2d& leftFoot,
                                                const Eigen::Affine2d& rightFoot,
                                                const LimbID& swingLeg) {
        // Negative if right leg to account for the mirroring of the foot target
        int8_t sign = swingLeg == LimbID::LEFT_LEG ? 1 : -1;
        // Get midpoint between the two feet
        Eigen::Affine2d midPoint = interpolate(leftFoot, rightFoot, 0.5);
        // Get midpoint 1.5 steps in future
        // Note: The reason for 1.5 rather than 1 is because it takes an extra 0.5 steps
        // for the torso to reach a given position when you want both feet together
        Eigen::Affine2d scaledVelocity;
        scaledVelocity.translation() = 1.5 * velocity.translation();
        scaledVelocity.linear() = Eigen::Rotation2Dd(1.5 * angle(velocity)).toRotationMatrix();
        Eigen::Affine2d forwardPoint = localToWorld(midPoint, scaledVelocity);

        // Offset to towards the foot in use to get the target location
        Eigen::Affine2d signMultiplieduLRFootOffset;
        signMultiplieduLRFootOffset.translation() = sign * uLRFootOffset.translation();
        signMultiplieduLRFootOffset.linear() = Eigen::Rotation2Dd(sign * angle(uLRFootOffset)).toRotationMatrix();
        Eigen::Affine2d footTarget = localToWorld(forwardPoint, signMultiplieduLRFootOffset);

        // Start applying step limits:
        // Get the vector between the feet and clamp the components between the min and max step limits
        Eigen::Affine2d supportFoot    = swingLeg == LimbID::LEFT_LEG ? rightFoot : leftFoot;
        Eigen::Affine2d feetDifference = worldToLocal(supportFoot, footTarget);
        feetDifference.translation().x()         = std::min(std::max(feetDifference.translation().x(), stepLimits(0, 0)), stepLimits(0, 1));
        feetDifference.translation().y()         = std::min(std::max(feetDifference.translation().y() * sign, stepLimits(1, 0)), stepLimits(1, 1)) * sign;
        feetDifference.linear() =
            Eigen::Rotation2Dd(std::min(std::max(angle(feetDifference) * sign, stepLimits(2, 0)), stepLimits(2, 1)) * sign).toRotationMatrix();
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
                         + angle(feetDifference);
        feetDifference.translation().y() = std::max(feetDifference.translation().y() * sign, stanceLimitY2 + overlap) * sign;
        // End feet collision detection

        // Update foot target to be 'feetDistance' away from the support foot
        footTarget = localToWorld(supportFoot, feetDifference);

        return footTarget;
    }

    Eigen::Vector3d OldWalkEngine::footPhase(double phase, double phase1Single, double phase2Single) {
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
