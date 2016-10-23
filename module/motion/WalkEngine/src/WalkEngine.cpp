/*----------------------------------------------DOCUMENT HEADER----------------------------------------------*/
/*===========================================================================================================*/
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
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "WalkEngine.h"
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
    using message::motion::BalanceBodyUpdate;
    using message::motion::WalkCommand;
    using message::motion::NewWalkCommand;
    using message::motion::WalkStartCommand;
    using message::motion::WalkStopCommand;
    using message::motion::WalkStopped;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableBalanceResponse;
    using message::motion::DisableBalanceResponse;
    using message::motion::EnableTorsoMotion;
    using message::motion::DisableTorsoMotion;
    using message::motion::EnableFootPlacement;
    using message::motion::DisableFootPlacement;
    using message::motion::EnableFootMotion;
    using message::motion::DisableFootMotion;
    using message::motion::ServoTarget;
    using message::motion::Script;
    using message::motion::kinematics::KinematicsModel;
    using message::support::SaveConfiguration;
    using message::support::Configuration;

    using utility::motion::kinematics::calculateLegJoints;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::angle::normalizeAngle;
    using utility::nubugger::graph;
    using utility::support::Expression;  
/*=======================================================================================================*/
//      NAME: WalkEngine
/*=======================================================================================================*/
    WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment) 
    : Reactor(std::move(environment))
        , DEBUG(false), DEBUG_ITER(0), initialStep(0)
        , emitLocalisation(false), emitFootPosition(false)
        , updateHandle(), generateStandScriptReaction(), subsumptionId(1)
        , StateOfWalk()
        , torsoPositionsTransform(), leftFootPositionTransform()
        , rightFootPositionTransform(), uSupportMass()
        , activeForwardLimb(), activeLimbInitial(LimbID::LEFT_LEG)
        , bodyTilt(0.0), bodyHeight(0.0), stanceLimitY2(0.0), stepTime(0.0), stepHeight(0.0)
        , step_height_slow_fraction(0.0f), step_height_fast_fraction(0.0f)
        , gainArms(0.0f), gainLegs(0.0f), stepLimits(arma::fill::zeros)
        , footOffsetCoefficient(arma::fill::zeros), uLRFootOffset()
        , armLPostureTransform(), armRPostureTransform()
        , beginStepTime(0.0), STAND_SCRIPT_DURATION(0.0), pushTime(), lastVeloctiyUpdateTime()
        , velocityHigh(0.0), accelerationTurningFactor(0.0), velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros), accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent(), velocityCommand()
        , kinematicsModel()
        , balanceAmplitude(0.0), balanceWeight(0.0), balanceOffset(0.0)
        , balancePGain(0.0), balanceIGain(0.0), balanceDGain(0.0)
        , jointGains(), servoControlPGains()
        , lastFootGoalRotation(), footGoalErrorSum()       
    {

        //Configure modular walk engine...
        on<Configuration>("WalkEngine.yaml").then("Walk Engine - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);       
        });

        //Define kinematics model for physical calculations...
        on<Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model", [this](const KinematicsModel& model)
        {
            kinematicsModel = model;
        });

        //Update waypoints sensor data at regular intervals...
        updateHandle =on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this] /*(const Sensors& sensors)*/
        {
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(0)"); }
            //emit(std::move(updateWaypoints(/*sensors*/)));
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(1)"); }
        }).disable();

        //Broadcast constrained velocity vector parameter to actuator modules...
        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand)
        {
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand(0)"); }
            setVelocity(walkCommand.command);  
            emit(std::make_unique<NewWalkCommand>(getVelocity()));
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand(1)"); }
        });

        on<Trigger<BalanceBodyUpdate>>().then("Walk Engine - Received update (Balanced Robot Posture) Info", [this](const BalanceBodyUpdate& info)
        {
            if(DEBUG) { NUClear::log("WalkEngine - Trigger BalanceBodyUpdate(0)"); }
            setLeftFootPosition(info.leftFoot);
            setRightFootPosition(info.rightFoot);
            setTorsoPositionArms(info.frameArms);
            setTorsoPositionLegs(info.frameLegs);
            setTorsoPosition3D(info.frame3D);
            if(DEBUG) { NUClear::log("WalkEngine - Trigger BalanceBodyUpdate(1)"); }
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
            //requestStop();
        });

        on<Trigger<WalkOptimiserCommand>>().then([this] (const WalkOptimiserCommand& command) 
        {
            configure(command.walkConfig);
            emit(std::make_unique<WalkConfigSaved>());
        });

        //generateStandScriptReaction = on<Trigger<Sensors>, Single>().then([this] (/*const Sensors& sensors*/) 
        //{
        //    generateStandScriptReaction.disable();
        //    //generateAndSaveStandScript(sensors);
        //    //StateOfWalk = State::LAST_STEP;
        //    start();
        //});

        on<Trigger<EnableWalkEngineCommand>>().then([this] (const EnableWalkEngineCommand& command) 
        {
            subsumptionId = command.subsumptionId;
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableFootPlacement>(command.subsumptionId + 2)));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableFootMotion>(command.subsumptionId + 3)));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableTorsoMotion>(command.subsumptionId + 3)));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableBalanceResponse>(command.subsumptionId + 1)));
            //stanceReset(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] (const DisableWalkEngineCommand& command)
        {
            // If nobody needs the walk engine, stop updating it...
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableFootPlacement>(command.subsumptionId + 2)));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableFootMotion>(command.subsumptionId + 3)));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableTorsoMotion>(command.subsumptionId + 3)));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableBalanceResponse>(command.subsumptionId + 1)));
            updateHandle.disable(); 
        });
    }
/*=======================================================================================================*/
//      NAME: generateAndSaveStandScript
/*=======================================================================================================*/
    void WalkEngine::generateAndSaveStandScript() 
    {
        //reset();
        //stanceReset();
        auto waypoints = updateWaypoints();

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
        //Try updateWaypoints(); ?
        //reset();
        //stanceReset();
    }    
/*=======================================================================================================*/
//      NAME: start
/*=======================================================================================================*/    
    void WalkEngine::start() 
    {
        if (StateOfWalk != State::WALKING) 
        {
            //activeForwardLimb = activeLimbInitial;
            //beginStepTime = getTime();
            //initialStep = 2;
            StateOfWalk = State::WALKING;
        }
    }
/*=======================================================================================================*/
//      NAME: requestStop
/*=======================================================================================================*/
    /*void WalkEngine::requestStop() 
    {
        // always stops with feet together (which helps transition)
        if (StateOfWalk == State::WALKING) 
        {
            StateOfWalk = State::STOP_REQUEST;
        }
    }*/    
/*=======================================================================================================*/
//      NAME: stop
/*=======================================================================================================*/
    void WalkEngine::stop() 
    {
        StateOfWalk = State::STOPPED;
        // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, { 0, 0 }})); // TODO: config
        //log<NUClear::TRACE>("Walk Engine:: Stop request complete");
        emit(std::make_unique<WalkStopped>());
        emit(std::make_unique<std::vector<ServoCommand>>());
    }    
/*=======================================================================================================*/
//      NAME: updateWaypoints
/*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::updateWaypoints() 
    {
        // Received foot positions are mapped relative to robot torso...
        auto joints = calculateLegJoints(kinematicsModel, getLeftFootPosition(), getRightFootPosition());
        auto robotWaypoints = motionLegs(joints);
        auto upperWaypoints = motionArms();

        robotWaypoints->insert(robotWaypoints->end(), upperWaypoints->begin(), upperWaypoints->end());

        return robotWaypoints;
    }
/*=======================================================================================================*/
//      NAME: motionArms
/*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionArms() 
    {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(6);

        NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);
        waypoints->push_back({ subsumptionId, time, ServoID::R_SHOULDER_PITCH, float(getRArmPosition()[0]), jointGains[ServoID::R_SHOULDER_PITCH], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::R_SHOULDER_ROLL,  float(getRArmPosition()[1]), jointGains[ServoID::R_SHOULDER_ROLL], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::R_ELBOW,          float(getRArmPosition()[2]), jointGains[ServoID::R_ELBOW], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_SHOULDER_PITCH, float(getLArmPosition()[0]), jointGains[ServoID::L_SHOULDER_PITCH], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_SHOULDER_ROLL,  float(getLArmPosition()[1]), jointGains[ServoID::L_SHOULDER_ROLL], 100 });
        waypoints->push_back({ subsumptionId, time, ServoID::L_ELBOW,          float(getLArmPosition()[2]), jointGains[ServoID::L_ELBOW], 100 });

        return std::move(waypoints);
    }    
/*=======================================================================================================*/
//      NAME: motionLegs
/*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints) 
    {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) 
        {
            waypoints->push_back({ subsumptionId, time, joint.first, joint.second, jointGains[joint.first], 100 }); 
            // TODO: support separate gains for each leg
        }

        return std::move(waypoints);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Velocity
/*=======================================================================================================*/
    Transform2D WalkEngine::getVelocity() 
    {
        return velocityCurrent;
    }        
    void WalkEngine::setVelocity(Transform2D inVelocityCommand) 
    {
        // hard limit commanded speed ??? not sure if necessary ???
        inVelocityCommand.x()     *= inVelocityCommand.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
        inVelocityCommand.y()     *= inVelocityCommand.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
        inVelocityCommand.angle() *= inVelocityCommand.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);
        if(DEBUG) { NUClear::log("Velocity(hard limit)"); }       
        // filter the commanded speed
        inVelocityCommand.x()     = std::min(std::max(inVelocityCommand.x(),     velocityLimits(0,0)), velocityLimits(0,1));
        inVelocityCommand.y()     = std::min(std::max(inVelocityCommand.y(),     velocityLimits(1,0)), velocityLimits(1,1));
        inVelocityCommand.angle() = std::min(std::max(inVelocityCommand.angle(), velocityLimits(2,0)), velocityLimits(2,1));
        if(DEBUG) { NUClear::log("Velocity(filtered 1)"); }
        // slow down when turning
        double vFactor = 1 - std::abs(inVelocityCommand.angle()) / accelerationTurningFactor;
        double stepMag = std::sqrt(inVelocityCommand.x() * inVelocityCommand.x() + inVelocityCommand.y() * inVelocityCommand.y());
        double magFactor = std::min(velocityLimits(0,1) * vFactor, stepMag) / (stepMag + 0.000001);

        inVelocityCommand.x()     = inVelocityCommand.x() * magFactor;
        inVelocityCommand.y()     = inVelocityCommand.y() * magFactor;
        inVelocityCommand.angle() = inVelocityCommand.angle();
        if(DEBUG) { NUClear::log("Velocity(slow  turn)"); }
        // filter the decelarated speed
        inVelocityCommand.x()     = std::min(std::max(inVelocityCommand.x(),     velocityLimits(0,0)), velocityLimits(0,1));
        inVelocityCommand.y()     = std::min(std::max(inVelocityCommand.y(),     velocityLimits(1,0)), velocityLimits(1,1));
        inVelocityCommand.angle() = std::min(std::max(inVelocityCommand.angle(), velocityLimits(2,0)), velocityLimits(2,1));
        if(DEBUG) { NUClear::log("Velocity(filtered 2)"); }  
        velocityCurrent = inVelocityCommand;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Time
/*=======================================================================================================*/
    double WalkEngine::getTime() 
    {
        if(DEBUG) { NUClear::log("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Arm Position
/*=======================================================================================================*/    
    arma::vec3 WalkEngine::getLArmPosition()
    {
        return (armLPostureTransform);
    }    
    void WalkEngine::setLArmPosition(arma::vec3 inLArm)
    {
        armLPostureTransform = inLArm;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Arm Position
/*=======================================================================================================*/ 
    arma::vec3 WalkEngine::getRArmPosition()
    {
        return (armRPostureTransform);
    } 
    void WalkEngine::setRArmPosition(arma::vec3 inRArm)
    {
        armRPostureTransform = inRArm;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Torso Position
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
    void WalkEngine::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
    Transform2D WalkEngine::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
    void WalkEngine::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }   
    Transform3D WalkEngine::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
    void WalkEngine::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }  
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Support Mass
/*=======================================================================================================*/
    Transform2D WalkEngine::getSupportMass()
    {
        return (uSupportMass);
    }
    void WalkEngine::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }        
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Left Foot Position
/*=======================================================================================================*/
    Transform3D WalkEngine::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
    void WalkEngine::setLeftFootPosition(const Transform3D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: Right Foot Position
/*=======================================================================================================*/
    Transform3D WalkEngine::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
    void WalkEngine::setRightFootPosition(const Transform3D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
//      INITIALISATION METHOD: Configuration
/*=======================================================================================================*/
    void WalkEngine::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        // gToe/heel overlap checking values
        stanceLimitY2 = kinematicsModel.Leg.LENGTH_BETWEEN_LEGS() - stance["limit_margin_y"].as<Expression>();

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
        //hipRollCompensation = walkCycle["hip_roll_compensation"].as<Expression>();
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

        auto& balance = walkCycle["balance"];
        // balanceAmplitude = balance["amplitude"].as<Expression>();
        // balanceWeight = balance["weight"].as<Expression>();
        // balanceOffset = balance["offset"].as<Expression>();

        for(auto& gain : balance["servo_gains"])
        {
            float p = gain["p"].as<Expression>();
            ServoID sr = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::RIGHT);
            ServoID sl = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }
        STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }    
}  // motion
}  // modules
