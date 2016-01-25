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

    FootPlacementPlanner::FootPlacementPlanner()
    {
        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand) {
            auto velocity = walkCommand.command;

            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);

            setVelocity(velocity);
        });
    }

    /*=======================================================================================================*/
    //      NAME: calculateNewStep
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
     */
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
    /*=======================================================================================================*/
    //      NAME: getNewFootTarget
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    Transform2D ModularWalkEngine::getNewFootTarget(const Transform2D& velocity, const Transform2D& leftFoot, const Transform2D& rightFoot, const LimbID& swingLeg) 
    {   
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
        Transform2D supportFoot = swingLeg == LimbID::LEFT_LEG ? rightFoot : leftFoot;
        Transform2D feetDifference = supportFoot.worldToLocal(footTarget);
        feetDifference.x()     = std::min(std::max(feetDifference.x(),            stepLimits(0,0)), stepLimits(0,1));
        feetDifference.y()     = std::min(std::max(feetDifference.y()     * sign, stepLimits(1,0)), stepLimits(1,1)) * sign;
        feetDifference.angle() = std::min(std::max(feetDifference.angle() * sign, stepLimits(2,0)), stepLimits(2,1)) * sign;
        // end applying step limits

        // Start feet collision detection:
        // Uses a rough measure to detect collision and move feet apart if too close
        double overlap = DarwinModel::Leg::FOOT_LENGTH / 2.0 * std::abs(feetDifference.angle());
        feetDifference.y() = std::max(feetDifference.y() * sign, stanceLimitY2 + overlap) * sign;
        // End feet collision detection

        // Update foot target to be 'feetDistance' away from the support foot
        footTarget = supportFoot.localToWorld(feetDifference);

        return footTarget;
    }

    void ModularWalkEngine::updateVelocity() {
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

        if (initialStep > 0) {
            velocityCurrent = arma::zeros(3);
            initialStep--;
        }
    }

    Transform2D ModularWalkEngine::stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor) {
        Transform2D uLeftFootSupport = uLeftFoot.localToWorld({-footOffset[0], -footOffset[1], 0});
        Transform2D uRightFootSupport = uRightFoot.localToWorld({-footOffset[0], footOffset[1], 0});
        return uLeftFootSupport.interpolate(shiftFactor, uRightFootSupport);
    }

    arma::vec2 ModularWalkEngine::zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) {
        /*
        Solves ZMP equations.
        The resulting form of x is
        x(t) = z(t) + aP*exp(t/zmpTime) + aN*exp(-t/zmpTime) - zmpTime*mi*sinh((t-Ti)/zmpTime)
        where the ZMP point is piecewise linear:
        z(0) = z1, z(T1 < t < T2) = zs, z(stepTime) = z2
        */
        double T1 = stepTime * phase1Single;
        double T2 = stepTime * phase2Single;
        double m1 = (zs - z1) / T1;
        double m2 = -(zs - z2) / (stepTime - T2);

        double c1 = x1 - z1 + zmpTime * m1 * std::sinh(-T1 / zmpTime);
        double c2 = x2 - z2 + zmpTime * m2 * std::sinh((stepTime - T2) / zmpTime);
        double expTStep = std::exp(stepTime / zmpTime);
        double aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        double aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }

}  // motion
}  // modules