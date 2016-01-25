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

        on<Trigger<WalkStartCommand>>().then([this] {
            lastVeloctiyUpdateTime = NUClear::clock::now();
            start();
            // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 25, 10 }})); // TODO: config
        });

        on<Trigger<WalkStopCommand>>().then([this] {
            // TODO: This sets STOP_REQUEST, which appears not to be used anywhere.
            // If this is the case, we should delete or rethink the WalkStopCommand.
            requestStop();
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

     void ModularWalkEngine::start() {
        if (state != State::WALKING) {
            swingLeg = swingLegInitial;
            beginStepTime = getTime();
            initialStep = 2;
            state = State::WALKING;
            calculateNewStep();
        }
    }

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
        }

        //emit destinations for fmp and/or zmp
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

    void WalkEngine::setVelocity(Transform2D velocity) {
        // filter the commanded speed
        velocity.x()     = std::min(std::max(velocity.x(),     velocityLimits(0,0)), velocityLimits(0,1));
        velocity.y()     = std::min(std::max(velocity.y(),     velocityLimits(1,0)), velocityLimits(1,1));
        velocity.angle() = std::min(std::max(velocity.angle(), velocityLimits(2,0)), velocityLimits(2,1));

        // slow down when turning
        double vFactor = 1 - std::abs(velocity.angle()) / accelerationTurningFactor;

        double stepMag = std::sqrt(velocity.x() * velocity.x() + velocity.y() * velocity.y());
        double magFactor = std::min(velocityLimits(0,1) * vFactor, stepMag) / (stepMag + 0.000001);

        velocityCommand.x()     = velocity.x() * magFactor;
        velocityCommand.y()     = velocity.y() * magFactor;
        velocityCommand.angle() = velocity.angle();

        velocityCommand.x()     = std::min(std::max(velocityCommand.x(),     velocityLimits(0,0)), velocityLimits(0,1));
        velocityCommand.y()     = std::min(std::max(velocityCommand.y(),     velocityLimits(1,0)), velocityLimits(1,1));
        velocityCommand.angle() = std::min(std::max(velocityCommand.angle(), velocityLimits(2,0)), velocityLimits(2,1));
    }

    Transform2D WalkEngine::getVelocity() {
        return velocityCurrent;
    }
}  // motion
}  // modules