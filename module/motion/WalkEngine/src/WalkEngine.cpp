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
        , balanceEnabled(0.0), emitLocalisation(false), emitFootPosition(false)
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
        , zmpCoefficients(arma::fill::zeros), zmpParameters(arma::fill::zeros)
        , zmpTime(0.0), phase1Single(0.0), phase2Single(0.0)
        , balancer(), kinematicsModel()
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

        //Update waypoints sensor data at regular intervals...
        updateHandle =on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this] /*(const Sensors& sensors)*/
        {
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(0)"); }
            emit(std::move(updateWaypoints(/*sensors*/)));
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(1)"); }
        }).disable();

        //Broadcast constrained velocity vector parameter to actuator modules...
        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand)
        {
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand(0)"); }
            auto velocity = walkCommand.command;
            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);
            setVelocity(velocity);
            emit(std::make_unique<NewWalkCommand>(getVelocity()));
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand(1)"); }
        });

        on<Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model", [this](const KinematicsModel& model)
        {
            kinematicsModel = model;
        });

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

        // on<Every<10, std::chrono::milliseconds>>(With<Configuration<WalkEngine>>>().then([this](const Configuration& config) 
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

        //generateStandScriptReaction = on<Trigger<Sensors>, Single>().then([this] (/*const Sensors& sensors*/) 
        //{
        //    generateStandScriptReaction.disable();
        //    //generateAndSaveStandScript(sensors);
        //    //StateOfWalk = State::LAST_STEP;
        //    start();
        //});

        //reset();
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
//      NAME: localise
/*=======================================================================================================*/
    void WalkEngine::localise(Transform2D position) 
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
    /*void FootPlacementPlanner::requestStop() 
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
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    Transform2D WalkEngine::getVelocity() 
    {
        return velocityCurrent;
    }      
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setVelocity
/*=======================================================================================================*/    
    void WalkEngine::setVelocity(Transform2D velocity) 
    {
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
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    double WalkEngine::getTime() 
    {
        if(DEBUG) { printf("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmPosition
/*=======================================================================================================*/    
    arma::vec3 WalkEngine::getLArmPosition()
    {
        return (armLPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmPosition
/*=======================================================================================================*/ 
    arma::vec3 WalkEngine::getRArmPosition()
    {
        return (armRPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D WalkEngine::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D WalkEngine::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
//      METHOD: configure
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
        zmpTime = walkCycle["zmp_time"].as<Expression>();
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
}  // motion
}  // modules
