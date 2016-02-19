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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "FootPlacementPlanner.h"

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
    using extension::Configuration;

<<<<<<< 183df72fb88459adef7436f0515b768fde100df7:module/motion/ModularWalkEngine/src/FootPlacementPlanner.cpp
    FootPlacementPlanner::FootPlacementPlanner()
=======
    FootPlacementPlanner::FootPlacementPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
>>>>>>> Further Modularization in development hierarchy, reorganised directory structure:module/motion/FootPlacementPlanner/src/FootPlacementPlanner.cpp
    {
        //Configure foot motion planner...
        on<Configuration>(CONFIGURATION_PATH).then([this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Do we need enable/disable?
        on<Trigger<EnableWalkEngineCommand>>().then([this] (const EnableModularWalkEngineCommand& command) 
        {
            subsumptionId = command.subsumptionId;

            stanceReset(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] 
        {
            // Nobody needs the walk engine, so we stop updating it.
            updateHandle.disable(); 

            // TODO: Also disable the other walk command reactions?
        });

        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand) {
            auto velocity = walkCommand.command;
            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);

            setVelocity(velocity);
        });

        updateHandle = on<Trigger<WalkStartCommand>>().then([this] {
            lastVeloctiyUpdateTime = NUClear::clock::now();
            start();
            // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 25, 10 }})); // TODO: config
        });

        on<Trigger<WalkStopCommand>>().then([this] {
            // TODO: This sets STOP_REQUEST, which appears not to be used anywhere.
            // If this is the case, we should delete or rethink the WalkStopCommand.
            requestStop();
        });

        on updateHandle = on<Trigger<StepCompleted>>().then([this] {
            calculateNewStep();
        }
    }
 	/*=======================================================================================================*/
    //      NAME: configure
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootPlacementPlanner::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArmStart = stance["arms"]["left"]["start"].as<arma::vec>();
        qLArmEnd = stance["arms"]["left"]["end"].as<arma::vec>();
        qRArmStart = stance["arms"]["right"]["start"].as<arma::vec>();
        qRArmEnd = stance["arms"]["right"]["end"].as<arma::vec>();
        footOffset = stance["foot_offset"].as<arma::vec>();
        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();

        auto& gains = stance["gains"];
        gainArms = gains["arms"].as<Expression>();
        gainLegs = gains["legs"].as<Expression>();

        for(ServoID i = ServoID(0); i < ServoID::NUMBER_OF_SERVOS; i = ServoID(int(i)+1))
        {
            if(int(i) < 6)
            {
                jointGains[i] = gainArms;
            } 
            else 
            {
                jointGains[i] = gainLegs;
            }
        }

        auto& walkCycle = config["walk_cycle"];
        stepTime = walkCycle["step_time"].as<Expression>();
        zmpTime = walkCycle["zmp_time"].as<Expression>();
        hipRollCompensation = walkCycle["hip_roll_compensation"].as<Expression>();
        stepHeight = walkCycle["step"]["height"].as<Expression>();
        stepLimits = walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh = velocity["high_speed"].as<Expression>();

        auto& acceleration = walkCycle["acceleration"];
        accelerationLimits = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        auto& balance = walkCycle["balance"];
        balanceEnabled = balance["enabled"].as<bool>();
        // balanceAmplitude = balance["amplitude"].as<Expression>();
        // balanceWeight = balance["weight"].as<Expression>();
        // balanceOffset = balance["offset"].as<Expression>();

        balancer.configure(balance);

        for(auto& gain : balance["servo_gains"])
        {
            float p = gain["p"].as<Expression>();
            ServoID sr = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::RIGHT);
            ServoID sl = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }
        /* TODO
        // gCompensation parameters
        toeTipCompensation = config["toeTipCompensation"].as<Expression>();
        ankleMod = {-toeTipCompensation, 0};

        // gGyro stabilization parameters
        ankleImuParamX = config["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY = config["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX = config["kneeImuParamX"].as<arma::vec>();
        hipImuParamY = config["hipImuParamY"].as<arma::vec>();
        armImuParamX = config["armImuParamX"].as<arma::vec>();
        armImuParamY = config["armImuParamY"].as<arma::vec>();

        // gSupport bias parameters to reduce backlash-based instability
        velFastForward = config["velFastForward"].as<Expression>();
        velFastTurn = config["velFastTurn"].as<Expression>();
        supportFront = config["supportFront"].as<Expression>();
        supportFront2 = config["supportFront2"].as<Expression>();
        supportBack = config["supportBack"].as<Expression>();
        supportSideX = config["supportSideX"].as<Expression>();
        supportSideY = config["supportSideY"].as<Expression>();
        supportTurn = config["supportTurn"].as<Expression>();
        */
        STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }


    /*=======================================================================================================*/
    //      NAME: start
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootPlacementPlanner::start() 
    {
        if (state != State::WALKING) 
        {
            swingLeg = swingLegInitial;
            beginStepTime = getTime();
            initialStep = 2;
            state = State::WALKING;
        }
        calculateNewStep();
    }
    /*=======================================================================================================*/
    //      NAME: requestStop
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootPlacementPlanner::requestStop() 
    {
        // always stops with feet together (which helps transition)
        if (state == State::WALKING) 
        {
            state = State::STOP_REQUEST;
        }
    }
    /*=======================================================================================================*/
    //      NAME: stop
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootPlacementPlanner::stop() 
    {
        state = State::STOPPED;
        // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0 }})); // TODO: config
        log<NUClear::TRACE>("Walk Engine:: Stop request complete");
        emit(std::make_unique<WalkStopped>());
        emit(std::make_unique<std::vector<ServoCommand>>());
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
    void FootPlacementPlanner::calculateNewStep() 
    {
        updateVelocity();

        // swap swing and support legs
        swingLeg = (swingLeg == LimbID::LEFT_LEG) ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;

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
        // apply velocity-based support point modulation for uSupport
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            Transform2D uLeftFootTorso = uTorsoSource.worldToLocal(uLeftFootSource);
            Transform2D uTorsoModded = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uLeftFootModded = uTorsoModded.localToWorld(uLeftFootTorso);
            uSupport = uLeftFootModded.localToWorld({-footOffset[0], -footOffset[1], 0});
            emit(std::make_unique<FootStepTarget(swingLeg, getTime() + stepTime, uRightFootDestination)>); //Trigger NewStep

        }
        else 
        {
            Transform2D uRightFootTorso = uTorsoSource.worldToLocal(uRightFootSource);
            Transform2D uTorsoModded = uTorso.localToWorld({supportMod[0], supportMod[1], 0});
            Transform2D uRightFootModded = uTorsoModded.localToWorld(uRightFootTorso);
            uSupport = uRightFootModded.localToWorld({-footOffset[0], footOffset[1], 0});
            emit(std::make_unique<FootStepTarget(swingLeg, getTime() + stepTime, uLeftFootDestination)>); //Trigger NewStep
        }

        emit(std:make_unique<NewStepTorso>(uLeftFootSource,uRightFootSource,uLeftFootDestination,uRightFootDestination,uSupport); //Torso Information
        //emit destinations for fmp and/or zmp
        //may combine NewStep and NewStepTorso
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
    Transform2D FootPlacementPlanner::getNewFootTarget(const Transform2D& velocity, const Transform2D& leftFoot, const Transform2D& rightFoot, const LimbID& swingLeg) 
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

    void FootPlacementPlanner::updateVelocity() { 
        // slow accelerations at high speed
        //TODO: Add acceleration to velocity (Replace initialStep)
        auto now = NUClear::clock::now();
        double deltaT = std::chrono::duration_cast<std::chrono::microseconds>(now - lastVeloctiyUpdateTime).count() * 1e-6;
        lastVeloctiyUpdateTime = now;

        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits) * deltaT; // TODO: use a function instead

        Transform2D velocityDifference;

        velocityCurrent.x()     = std::min(std::max(velocityCommand.x()     - velocityCurrent.x(),     -limit[0]), limit[0]);
        velocityDifference.y()     = std::min(std::max(velocityCommand.y()     - velocityCurrent.y(),     -limit[1]), limit[1]);
        velocityDifference.angle() = std::min(std::max(velocityCommand.angle() - velocityCurrent.angle(), -limit[2]), limit[2]);

        velocityCurrent.x()     += velocityDifference.x();
        velocityCurrent.y()     += velocityDifference.y();
        velocityCurrent.angle() += velocityDifference.angle();

    }

    void FootPlacementPlanner::setVelocity(Transform2D velocity) {
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

    Transform2D FootPlacementPlanner::getVelocity() {
        return velocityCurrent;
    }

}  // modulesvoid FootPlacementPlanner::stanceReset() 
    {
        // standup/sitdown/falldown handling
        if (startFromStep) 
        {
            uLeftFoot = arma::zeros(3);
            uRightFoot = arma::zeros(3);
            uTorso = arma::zeros(3);

            // start walking asap
            initialStep = 1;
        } 
        else 
        {
            // stance resetted
            uLeftFoot = uTorso.localToWorld({footOffset[0], DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], 0});
            uRightFoot = uTorso.localToWorld({footOffset[0], -DarwinModel::Leg::HIP_OFFSET_Y + footOffset[1], 0});
            initialStep = 2;
        }

        swingLeg = swingLegInitial;

        uLeftFootSource = uLeftFoot;
        uLeftFootDestination = uLeftFoot;

        uRightFootSource = uRightFoot;
        uRightFootDestination = uRightFoot;

        uSupport = uTorso;
        beginStepTime = getTime();
        uLRFootOffset = {0, DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], 0};
        startFromStep = false;

        calculateNewStep();
    }
    /*=======================================================================================================*/
    //      NAME: reset
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootPlacementPlanner::reset() 
    {
        uTorso = {-footOffset[0], 0, 0};
        uLeftFoot = {0, DarwinModel::Leg::HIP_OFFSET_Y, 0};
        uRightFoot = {0, -DarwinModel::Leg::HIP_OFFSET_Y, 0};

        uTorsoSource = arma::zeros(3);
        uTorsoDestination = arma::zeros(3);
        uLeftFootSource = arma::zeros(3);
        uLeftFootDestination = arma::zeros(3);
        uRightFootSource = arma::zeros(3);
        uRightFootDestination = arma::zeros(3);

        velocityCurrent = arma::zeros(3);
        velocityCommand = arma::zeros(3);
        velocityDifference = arma::zeros(3);

        // gZMP exponential coefficients:
        zmpCoefficients = arma::zeros(4);
        zmpParams = arma::zeros(4);

        // gGyro stabilization variables
        swingLeg = swingLegInitial;
        beginStepTime = getTime();
        initialStep = 2;

        // gStandard offset
        uLRFootOffset = {0, DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], 0};

        // gWalking/Stepping transition variables
        startFromStep = false;

        state = State::STOPPED;

        // interrupted = false;
    }
}  // motion    
}  // modules