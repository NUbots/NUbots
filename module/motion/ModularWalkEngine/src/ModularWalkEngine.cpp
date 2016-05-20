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
//      NAME: ModularWalkEngine
/*=======================================================================================================*/
    ModularWalkEngine::ModularWalkEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) 
    {
        //Configure modular walk engine...
        on<Configuration>("ModularWalkEngine.yaml").then("Modular Walk Engine - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //updateWaypoints sensor data at regular intervals...
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then([this](const Sensors& sensors) 
        {
            emit(std::move(updateWaypoints(sensors)));
        }).disable();

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
//      NAME: generateAndSaveStandScript
/*=======================================================================================================*/
    void ModularWalkEngine::generateAndSaveStandScript(const Sensors& sensors) 
    {
        reset();
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
        reset();
        //stanceReset();
    }    
/*=======================================================================================================*/
//      NAME: localise
/*=======================================================================================================*/
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
//      NAME: updateWaypoints
/*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::updateWaypoints(const Sensors& sensors) 
    {
        if (state == State::STOPPED) 
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
            emit(graph("Right foot pos", rightFootTorso.translation()));
            emit(graph("Left  foot pos",  leftFootTorso.translation()));
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
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::motionArms(double phase) 
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
    std::unique_ptr<std::vector<ServoCommand>> ModularWalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints) 
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
    double ModularWalkEngine::linearInterpolationDeadband(double value, double deadband, double maxvalue) 
    {
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    double ModularWalkEngine::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmPosition
/*=======================================================================================================*/    
    arma::vec3 ModularWalkEngine::getLArmPosition()
    {
        return (armLPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmPosition
/*=======================================================================================================*/     
    void ModularWalkEngine::setLArmPosition(arma::vec3 inLArm)
    {
        armLPostureTransform = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmSource
/*=======================================================================================================*/     
    arma::vec3 ModularWalkEngine::getLArmSource()
    {
        return (armLPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmSource
/*=======================================================================================================*/     
    void ModularWalkEngine::setLArmSource(arma::vec3 inLArm)
    {
        armLPostureSource = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmDestination
/*=======================================================================================================*/     
    arma::vec3 ModularWalkEngine::getLArmDestination()
    {
        return (armLPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmDestination
/*=======================================================================================================*/     
    void ModularWalkEngine::setLArmDestination(arma::vec3 inLArm)
    {
        armLPostureDestination = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmPosition
/*=======================================================================================================*/ 
    arma::vec3 ModularWalkEngine::getRArmPosition()
    {
        return (armRPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmPosition
/*=======================================================================================================*/     
    void ModularWalkEngine::setRArmPosition(arma::vec3 inRArm)
    {
        armRPostureTransform = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmSource
/*=======================================================================================================*/     
    arma::vec3 ModularWalkEngine::getRArmSource()
    {
        return (armRPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmSource
/*=======================================================================================================*/     
    void ModularWalkEngine::setRArmSource(arma::vec3 inRArm)
    {
        armRPostureSource = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmDestination
/*=======================================================================================================*/     
    arma::vec3 ModularWalkEngine::getRArmDestination()
    {
        return (armRPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmDestination
/*=======================================================================================================*/     
    void ModularWalkEngine::setRArmDestination(arma::vec3 inRArm)
    {
        armRPostureDestination = inRArm;
    }  
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D ModularWalkEngine::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionLegs
/*=======================================================================================================*/
    void ModularWalkEngine::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionArms
/*=======================================================================================================*/
    void ModularWalkEngine::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPosition3D
/*=======================================================================================================*/
    void ModularWalkEngine::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoSource
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getTorsoSource()
    {
        return (torsoPositionSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoSource
/*=======================================================================================================*/
    void ModularWalkEngine::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoDestination
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoDestination
/*=======================================================================================================*/
    void ModularWalkEngine::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setSupportMass
/*=======================================================================================================*/
    void ModularWalkEngine::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootPosition
/*=======================================================================================================*/
    void ModularWalkEngine::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootPosition
/*=======================================================================================================*/
    void ModularWalkEngine::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void ModularWalkEngine::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootSource
/*=======================================================================================================*/
    void ModularWalkEngine::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }        
/*=======================================================================================================*/
//      METHOD: getLeftFootDestination
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getLeftFootDestination()
    {
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
//      METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void ModularWalkEngine::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        leftFootDestination.push(inLeftFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
    Transform2D ModularWalkEngine::getRightFootDestination()
    {
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void ModularWalkEngine::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        rightFootDestination.push(inRightFootDestination);
    }        
/*=======================================================================================================*/
/*      METHOD: configure
/*=======================================================================================================*/
    void ModularWalkEngine::configure(const YAML::Node& config)
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
