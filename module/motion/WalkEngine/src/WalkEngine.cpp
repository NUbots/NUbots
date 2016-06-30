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
    using message::input::LimbID;
    using message::motion::WalkCommand;
    using message::motion::NewWalkCommand;
    using message::motion::WalkStartCommand;
    using message::motion::WalkStopCommand;
    using message::motion::WalkStopped;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::ServoTarget;
    using message::motion::Script;
    using message::support::SaveConfiguration;
    using message::support::Configuration;

    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::angle::normalizeAngle;
    using utility::nubugger::graph;
    using utility::support::Expression;       
/*=======================================================================================================*/
//      NAME: WalkEngine
/*=======================================================================================================*/
    WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) 
    {
        //Configure modular walk engine...
        on<Configuration>("WalkEngine.yaml").then("Walk Engine - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Automate walk engine command for testing...
        updateHandle = on<Every<1 /*RESTORE AFTER DEBUGGING: UPDATE_FREQUENCY*/, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) 
        {
            if(DEBUG) { NUClear::log("WalkEngine - Emit WalkCommand(0)"); }
            if((counter_auto++)%15 == 0)
            {
                emit(std::make_unique<WalkCommand>(1, Transform2D({0.1, 0.05, 0.2}))); //debugging...
            }
            if(DEBUG) { NUClear::log("WalkEngine - Emit WalkCommand(1)"); }
        });//RESTORE AFTER DEBUGGING: .disable();

        //Broadcast constrained velocity vector parameter to actuator modules...
        on<Trigger<WalkCommand>>().then([this] (const WalkCommand& walkCommand)
        {
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand (0)"); }
            auto velocity = walkCommand.command;
            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);
            setVelocity(velocity);
            emit(std::make_unique<NewWalkCommand>(getVelocity()));
            if(DEBUG) { NUClear::log("WalkEngine - Trigger WalkCommand (1)"); }
        });

        //Update waypoints sensor data at regular intervals...
        updateHandle = on<Every<1 /*RESTORE AFTER DEBUGGING: UPDATE_FREQUENCY*/, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) 
        {
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(0)"); }
            //emit(std::move(updateWaypoints(sensors)));
            if(DEBUG) { NUClear::log("WalkEngine - Update Waypoints(1)"); }
        });//RESTORE AFTER DEBUGGING: .disable();

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

        generateStandScriptReaction = on<Trigger<Sensors>, Single>().then([this] (const Sensors& sensors) 
        {
            generateStandScriptReaction.disable();
            //generateAndSaveStandScript(sensors);
            //StateOfWalk = State::LAST_STEP;
            start();
        });

        //Do we need enable/disable?
        on<Trigger<EnableWalkEngineCommand>>().then([this] (const EnableWalkEngineCommand& command) 
        {
            subsumptionId = command.subsumptionId;
            //stanceReset(); // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] 
        {
            // Nobody needs the walk engine, so we stop updating it.
            updateHandle.disable(); 

            // TODO: Also disable the other walk command reactions?
        });

        //reset();
    }
/*=======================================================================================================*/
//      NAME: generateAndSaveStandScript
/*=======================================================================================================*/
    void WalkEngine::generateAndSaveStandScript(const Sensors& sensors) 
    {
        //reset();
        //stanceReset();
        auto waypoints = updateWaypoints(sensors);

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
            //swingLeg = swingLegInitial;
            //beginStepTime = getTime();
            //initialStep = 2;
            StateOfWalk = State::WALKING;
        }
    }
/*=======================================================================================================*/
/*      NAME: requestStop
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
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::updateWaypoints(const Sensors& sensors) 
    {
        if (StateOfWalk == State::STOPPED) 
        {
            //identify state of robot, walk engine handles actual posture...
        }

        //get relevant torso models from TorsoMotionUpdate...
        /*
        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);
        Transform2D uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});
        Transform3D torso = arma::vec6({uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()});
        */

        // Transform feet targets to be relative to the torso...
        Transform3D leftFootTorso  = Transform3D(getLeftFootPosition()).worldToLocal(getTorsoPosition3D());
        Transform3D rightFootTorso = Transform3D(getRightFootPosition()).worldToLocal(getTorsoPosition3D());
        Transform2D uTorsoWorld    = getTorsoPositionArms().localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});

        //DEBUGGING: Emit relative torso position with respect to world model... 
        if (emitLocalisation) 
        {
            localise(uTorsoWorld);
        }

        if (balanceEnabled) 
        {
            // Apply balance to both legs when standing still
            balancer.balance(leftFootTorso,  LimbID::LEFT_LEG,  sensors);
            balancer.balance(rightFootTorso, LimbID::RIGHT_LEG, sensors);
        }

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            emit(graph("Right foot pos", arma::vec(rightFootTorso.translation())));
            emit(graph("Left  foot pos",  arma::vec(leftFootTorso.translation())));
        }

        auto joints = calculateLegJoints<DarwinModel>(leftFootTorso, rightFootTorso);
        auto robotWaypoints = motionLegs(joints);
        auto upperWaypoints = motionArms(0.5);

        robotWaypoints->insert(robotWaypoints->end(), upperWaypoints->begin(), upperWaypoints->end());

        return robotWaypoints;
    }
/*=======================================================================================================*/
//      NAME: motionArms
/*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionArms(double phase) 
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
//      NAME: linearInterpolationDeadband
/*=======================================================================================================*/
    double WalkEngine::linearInterpolationDeadband(double value, double deadband, double maxvalue) 
    {
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
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
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmPosition
/*=======================================================================================================*/    
    arma::vec3 WalkEngine::getLArmPosition()
    {
        return (armLPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmPosition
/*=======================================================================================================*/     
    void WalkEngine::setLArmPosition(arma::vec3 inLArm)
    {
        armLPostureTransform = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmSource
/*=======================================================================================================*/     
    arma::vec3 WalkEngine::getLArmSource()
    {
        return (armLPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmSource
/*=======================================================================================================*/     
    void WalkEngine::setLArmSource(arma::vec3 inLArm)
    {
        armLPostureSource = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmDestination
/*=======================================================================================================*/     
    arma::vec3 WalkEngine::getLArmDestination()
    {
        return (armLPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmDestination
/*=======================================================================================================*/     
    void WalkEngine::setLArmDestination(arma::vec3 inLArm)
    {
        armLPostureDestination = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmPosition
/*=======================================================================================================*/ 
    arma::vec3 WalkEngine::getRArmPosition()
    {
        return (armRPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmPosition
/*=======================================================================================================*/     
    void WalkEngine::setRArmPosition(arma::vec3 inRArm)
    {
        armRPostureTransform = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmSource
/*=======================================================================================================*/     
    arma::vec3 WalkEngine::getRArmSource()
    {
        return (armRPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmSource
/*=======================================================================================================*/     
    void WalkEngine::setRArmSource(arma::vec3 inRArm)
    {
        armRPostureSource = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmDestination
/*=======================================================================================================*/     
    arma::vec3 WalkEngine::getRArmDestination()
    {
        return (armRPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmDestination
/*=======================================================================================================*/     
    void WalkEngine::setRArmDestination(arma::vec3 inRArm)
    {
        armRPostureDestination = inRArm;
    }  
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D WalkEngine::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionLegs
/*=======================================================================================================*/
    void WalkEngine::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionArms
/*=======================================================================================================*/
    void WalkEngine::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPosition3D
/*=======================================================================================================*/
    void WalkEngine::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoSource
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoSource()
    {
        return (torsoPositionSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoSource
/*=======================================================================================================*/
    void WalkEngine::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoDestination
/*=======================================================================================================*/
    Transform2D WalkEngine::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoDestination
/*=======================================================================================================*/
    void WalkEngine::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D WalkEngine::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setSupportMass
/*=======================================================================================================*/
    void WalkEngine::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootPosition
/*=======================================================================================================*/
    void WalkEngine::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D WalkEngine::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootPosition
/*=======================================================================================================*/
    void WalkEngine::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D WalkEngine::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void WalkEngine::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D WalkEngine::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootSource
/*=======================================================================================================*/
    void WalkEngine::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }        
/*=======================================================================================================*/
//      METHOD: getLeftFootDestination
/*=======================================================================================================*/
    Transform2D WalkEngine::getLeftFootDestination()
    {
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
//      METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void WalkEngine::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        leftFootDestination.push(inLeftFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
    Transform2D WalkEngine::getRightFootDestination()
    {
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void WalkEngine::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        rightFootDestination.push(inRightFootDestination);
    }        
/*=======================================================================================================*/
/*      METHOD: configure
/*=======================================================================================================*/
    void WalkEngine::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        setLArmSource(stance["arms"]["left"]["start"].as<arma::vec>());
        setLArmDestination(stance["arms"]["left"]["end"].as<arma::vec>());
        setRArmSource(stance["arms"]["right"]["start"].as<arma::vec>());
        setRArmDestination(stance["arms"]["right"]["end"].as<arma::vec>());
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
