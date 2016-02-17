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

#include <algorithm>
#include <armadillo>
#include <chrono>
#include <cmath>

#include "message/behaviour/ServoCommand.h"
#include "message/support/Configuration.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/Script.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/localisation/FieldObject.h"

#include "utility/motion/Balance.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"
#include "message/input/PushDetection.h"
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
    using message::input::PushDetection;
    using message::input::ServoID;
    using message::input::Sensors;
    using message::input::LimbID;
    using message::behaviour::ServoCommand;
    using message::behaviour::WalkOptimiserCommand;
    using message::behaviour::WalkConfigSaved;
    // using message::behaviour::RegisterAction;
    // using message::behaviour::ActionPriorites;
    using message::input::LimbID;
    using message::motion::WalkCommand;
    using message::motion::WalkStartCommand;
    using message::motion::WalkStopCommand;
    using message::motion::WalkStopped;
    using message::motion::EnableModularWalkEngineCommand;
    using message::motion::DisableModularWalkEngineCommand;
    using message::motion::ServoTarget;
    using message::motion::Script;
    using message::support::SaveConfiguration;
    using message::support::Configuration;

    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::angle::normalizeAngle;
    using utility::nubugger::graph;
    using utility::support::Expression;
    /*=======================================================================================================*/
    //      NAME: ModularWalkEngine
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    ModularWalkEngine::ModularWalkEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) 
    {
        
        on<Trigger<EnableModularWalkEngineCommand>>().then([this] (const EnableModularWalkEngineCommand& command) 
        {
            subsumptionId = command.subsumptionId;

            stanceReset(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();        });

        on<Trigger<DisableModularWalkEngineCommand>>().then([this] 
        {
            // Nobody needs the walk engine, so we stop updating it.
            updateHandle.disable();

            // TODO: Also disable the other walk command reactions?
        });

        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) 
        {
            update(sensors);
        }).disable();

        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand) 
        {
            auto velocity = walkCommand.command;

            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);

            setVelocity(velocity);
        });

        on<Trigger<WalkStartCommand>>().then([this] 
        {
            lastVeloctiyUpdateTime = NUClear::clock::now();
            start();
            // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 25, 10 }})); // TODO: config
        });

        on<Trigger<WalkStopCommand>>().then([this] 
        {
            // TODO: This sets STOP_REQUEST, which appears not to be used anywhere.
            // If this is the case, we should delete or rethink the WalkStopCommand.
            requestStop();
        });

        on<Configuration>(CONFIGURATION_PATH).then([this] (const Configuration& config) 
        {
            configure(config.config);
        });

        // TODO: finish push detection and compensation
        // pushTime = NUClear::clock::now();
        // on<Trigger<PushDetection>, With<Configuration>>().then([this](const PushDetection& pd, const Configuration& config) 
        // {
        //     balanceEnabled = true;
        //     // balanceAmplitude = balance["amplitude"].as<Expression>();
        //     // balanceWeight = balance["weight"].as<Expression>();
        //     // balanceOffset = balance["offset"].as<Expression>();
        //     balancer.configure(config["walk_cycle"]["balance"]["push_recovery"]);
        //     pushTime = NUClear::clock::now();
        //     // configure(config.config);
        // });

        // on<Every<10, std::chrono::milliseconds>>(With<Configuration<ModularWalkEngine>>>().then([this](const Configuration& config) 
        // {
        //     [this](const WalkOptimiserCommand& command) 
        //     {
        //     if ((NUClear::clock::now() - pushTime) > std::chrono::milliseconds(config["walk_cycle"]["balance"]["balance_time"].as<int>)) 
        //     {
        //         balancer.configure(config["walk_cycle"]["balance"]);
        //     }
        // });


        on<Trigger<WalkOptimiserCommand>>().then([this] (const WalkOptimiserCommand& command) 
        {
            configure(command.walkConfig);
            emit(std::make_unique<WalkConfigSaved>());
        });

        generateStandScriptReaction = on<Trigger<Sensors>, Single>().then([this] (const Sensors& sensors) 
        {
            generateStandScriptReaction.disable();
            //generateAndSaveStandScript(sensors);
            //state = State::LAST_STEP;
            //start();
        });

        reset();
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
    void ModularWalkEngine::configure(const YAML::Node& config)
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
    //      NAME: generateAndSaveStandScript
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void ModularWalkEngine::generateAndSaveStandScript(const Sensors& sensors) 
    {
        reset();
        stanceReset();
        auto waypoints = updateStillWayPoints(sensors);

        Script standScript;
        Script::Frame frame;
        frame.duration = std::chrono::milliseconds(int(round(1000 * STAND_SCRIPT_DURATION)));
        for (auto& waypoint : *waypoints) 
        {
            frame.targets.push_back(Script::Frame::Target({waypoint.id, waypoint.position, std::max(waypoint.gain, 60.0f), 100}));
        }
        standScript.frames.push_back(frame);
        auto saveScript = std::make_unique<SaveConfiguration>();
        saveScript->path = "scripts/Stand.yaml";
        saveScript->config = standScript;
        emit(std::move(saveScript));
        //Try update(); ?
        reset();
        stanceReset();
    }
    /*=======================================================================================================*/
    //      NAME: stanceReset
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    
    
    /*=======================================================================================================*/
    //      NAME: localise
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void ModularWalkEngine::localise(Transform2D position) 
    {
        // emit position as a fake localisation
        auto localisation = std::make_unique<std::vector<message::localisation::Self>>();
        message::localisation::Self self;
        self.position = {position.x(), position.y()};
        self.position_cov = arma::eye(2,2) * 0.1; // made up
        self.heading = {std::cos(position.angle()), std::sin(position.angle())}; // convert to cartesian coordinates
        self.velocity = arma::zeros(2); // not used
        self.robot_to_world_rotation = arma::zeros(2,2); // not used
        localisation->push_back(self);
        emit(std::move(localisation));
    }
    /*=======================================================================================================*/
    //      NAME: update
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void ModularWalkEngine::update(const Sensors& sensors) 
    {
        //double now = getTime();

        if (state == State::STOPPED) 
        {
            updateStill(sensors);
            return;
        }

        /* The phase of the current step, range: [0,1]...
        double phase = (now - beginStepTime) / stepTime;*/

        //bool newStep = false;

        /*Bind phase value to range [0,1], assume initiation new step...
        if (phase > 1) 
        {
            phase = std::fmod(phase, 1);
            beginStepTime += stepTime;
            newStep = true;
        }*/

        if (newStep && state == State::LAST_STEP) 
        {
            stop();
            return;
        }

        /*Compute FootSource and FootDestination for this step
        if (newStep) 
        {
            calculateNewStep();
        }*/

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
        auto robotWaypoints = updateLowerBody(phase, sensors);
        auto upperWaypoints = updateUpperBody(phase, sensors);

        robotWaypoints->insert(robotWaypoints->end(), upperWaypoints->begin(), upperWaypoints->end());

        emit(std::move(robotWaypoints));
    }
    /*=======================================================================================================*/
    //      NAME: updateStillWayPoints
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::updateStillWayPoints(const Sensors& sensors) 
    {
        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);
        
        Transform2D uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});

        Transform3D torso = arma::vec6({uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()});

        // Transform feet targets to be relative to the torso
        Transform3D leftFootTorso = Transform3D(uLeftFoot).worldToLocal(torso);
        Transform3D rightFootTorso = Transform3D(uRightFoot).worldToLocal(torso);

        //DEBUGGING: Emit relative torso position with respect to world model... 
        if (emitLocalisation) 
        {
            localise(uTorsoActual);
        }

        if (balanceEnabled) 
        {
            // Apply balance to both legs when standing still
            balancer.balance(leftFootTorso, LimbID::LEFT_LEG, sensors);
            balancer.balance(rightFootTorso, LimbID::RIGHT_LEG, sensors);
        }

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            emit(graph("Right foot pos", rightFootTorso.translation()));
            emit(graph("Left  foot pos",  leftFootTorso.translation()));
        }

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
        
        auto waypoints = motionLegs(joints);

        auto arms = motionArms(0.5);
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }
    /*=======================================================================================================*/
    //      NAME: updateStill
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void ModularWalkEngine::updateStill(const Sensors& sensors) 
    {
        emit(std::move(updateStillWayPoints(sensors)));
    }
    /*=======================================================================================================*/
    //      NAME: getTime
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double ModularWalkEngine::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
    /*=======================================================================================================*/
    //      NAME: linearInterpolationDeadband
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    double ModularWalkEngine::linearInterpolationDeadband(double value, double deadband, double maxvalue) 
    {
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }
}  // motion
}  // modules
