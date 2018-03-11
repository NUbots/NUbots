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
 * Copyright 2013 NUbots <nubots@nubots.net>
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
namespace module {
namespace motion {
    /*=======================================================================================================*/
    //      UTILIZATION REFERENCE(S)
    /*=======================================================================================================*/
    using ServoID = utility::input::ServoID;
    using message::input::Sensors;

    using message::behaviour::ServoCommand;
    using message::behaviour::WalkConfigSaved;
    using message::behaviour::WalkOptimiserCommand;
    // using message::behaviour::RegisterAction;
    // using message::behaviour::ActionPriorites;

    using message::motion::BalanceBodyUpdate;
    using message::motion::DisableBalanceResponse;
    using message::motion::DisableFootMotion;
    using message::motion::DisableFootPlacement;
    using message::motion::DisableTorsoMotion;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableBalanceResponse;
    using message::motion::EnableFootMotion;
    using message::motion::EnableFootPlacement;
    using message::motion::EnableTorsoMotion;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::KinematicsModel;
    using message::motion::NewWalkCommand;
    using message::motion::ServoTarget;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStarted;
    using message::motion::WalkStopped;
    using utility::motion::kinematics::calculateLegJointsTeamDarwin;  // TODO: advised to change to calculateLegJoints
                                                                      // (no TeamDarwin)

    using extension::Configuration;
    using extension::Script;

    using utility::support::Expression;

    using utility::math::angle::normalizeAngle;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;

    using utility::nubugger::graph;
    /*=======================================================================================================*/
    //      NAME: WalkEngine
    /*=======================================================================================================*/
    WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , DEBUG(false)
        , DEBUG_ITER(0)
        , newPostureReceived(false)
        , handleUpdate()
        , handleStandScript()
        , subsumptionId(1)
        , leftFootPositionTransform()
        , rightFootPositionTransform()
        , gainRArm(0.0f)
        , gainRLeg(0.0f)
        , gainLArm(0.0f)
        , gainLLeg(0.0f)
        , gainHead(0.0f)
        , armLPostureTransform()
        , armRPostureTransform()
        , STAND_SCRIPT_DURATION(0.0)
        , velocityHigh(0.0)
        , accelerationTurningFactor(0.0)
        , velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros)
        , accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent()
        , velocityCommand()
        , velFastForward(0.0)
        , velFastTurn(0.0)
        , kinematicsModel()
        , jointGains()
        , servoControlPGains()
        , lastFootGoalRotation()
        , footGoalErrorSum() {

        // Configure modular walk engine...
        on<Configuration>("WalkEngine.yaml").then("Walk Engine - Configure", [this](const Configuration& config) {
            configure(config.config);
        });

        // Define kinematics model for physical calculations...
        on<Startup, Trigger<KinematicsModel>>().then("WalkEngine - Update Kinematics Model",
                                                     [this](const KinematicsModel& model) { kinematicsModel = model; });

        // Broadcast constrained velocity vector parameter to actuator modules...
        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            if (!handleUpdate.enabled()) {
                emit(std::make_unique<EnableWalkEngineCommand>(walkCommand.subsumptionId));  // TODO Subsumtion variable
            }
            if (DEBUG) {
                log<NUClear::TRACE>("WalkEngine - Trigger WalkCommand(0)");
            }
            setVelocity(convert<double, 3>(walkCommand.command));
            emit(std::make_unique<NewWalkCommand>(convert<double, 3>(getVelocity())));
            // Notify behavioural modules of current standstill...
            emit(std::make_unique<WalkStarted>());
            if (DEBUG) {
                log<NUClear::TRACE>("WalkEngine - Trigger WalkCommand(1)");
            }
        });

        // If override stop command is issued, signal zero velocity command...
        on<Trigger<StopCommand>>().then([this] {
            if (DEBUG) {
                log<NUClear::TRACE>("WalkEngine - Trigger StopCommand(0)");
            }
            // Emit zero velocity command to trigger final adjustment step...
            emit(std::make_unique<NewWalkCommand>(convert<double, 3>(Transform2D({0, 0, 0}))));
            // Notify behavioural modules of current standstill...
            // emit(std::make_unique<WalkStopped>()); //moved to fpp when walk actually stops
            // emit(std::make_unique<std::vector<ServoCommand>>());
            if (DEBUG) {
                log<NUClear::TRACE>("WalkEngine - Trigger WalkCommand(1)");
            }
        });

        // Update goal robot posture given new balance information...
        handleUpdate = on<Trigger<BalanceBodyUpdate>>()
                           .then("Walk Engine - Received update (Balanced Robot Posture) Info",
                                 [this](const BalanceBodyUpdate& info) {
                                     if (DEBUG) {
                                         log<NUClear::TRACE>("WalkEngine - Trigger BalanceBodyUpdate(0)");
                                     }
                                     setLeftFootPosition(convert<double, 4, 4>(info.leftFoot));
                                     setRightFootPosition(convert<double, 4, 4>(info.rightFoot));
                                     setLArmPosition(convert<double, 3>(info.armLPosition));
                                     setRArmPosition(convert<double, 3>(info.armRPosition));

                                     emit(graph("WE: Left  Foot Joint Position", getLeftFootPosition()));
                                     emit(graph("WE: Right Foot Joint Position", getRightFootPosition()));
                                     emit(std::move(updateWaypoints()));

                                     if (DEBUG) {
                                         log<NUClear::TRACE>("WalkEngine - Trigger BalanceBodyUpdate(1)");
                                     }
                                 })
                           .disable();

        // Update walk configuration with optimiser parameters...
        on<Trigger<WalkOptimiserCommand>>().then([this](const WalkOptimiserCommand& command) {
            configure(YAML::Load(command.walkConfig));
            emit(std::make_unique<WalkConfigSaved>());
        });

        // Update stand configuration with active walk posture...
        handleStandScript = on<Trigger<Sensors>, Single>().then([this] {
            scriptStandAndSave();
            handleStandScript.unbind();
        });

        // Activation of WalkEngine (and default) subordinate actuator modules...
        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            // If the walk engine is required, enable relevant submodules and award subsumption...
            subsumptionId = command.subsumptionId;
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableFootPlacement>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableFootMotion>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableTorsoMotion>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<EnableBalanceResponse>()));
            handleUpdate.enable();
        });

        // If WalkEngine no longer requested, cease updating...
        on<Trigger<DisableWalkEngineCommand>>().then([this] {
            // If nobody needs the walk engine, stop updating dependancies...
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableFootPlacement>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableFootMotion>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableTorsoMotion>()));
            emit<Scope::DIRECT>(std::move(std::make_unique<DisableBalanceResponse>()));
            handleUpdate.disable();
        });
    }
    /*=======================================================================================================*/
    //      NAME: scriptStandAndSave
    /*=======================================================================================================*/
    void WalkEngine::scriptStandAndSave() {
        Script standScript;
        Script::Frame frame;
        auto waypoints = updateWaypoints();
        frame.duration = std::chrono::milliseconds(int(round(1000 * STAND_SCRIPT_DURATION)));
        for (auto& waypoint : *waypoints) {
            frame.targets.push_back(
                Script::Frame::Target({waypoint.id, waypoint.position, std::max(waypoint.gain, 60.0f), 100}));
        }
        standScript.frames.push_back(frame);
        standScript.save("Stand.yaml");
    }
    /*=======================================================================================================*/
    //      NAME: updateWaypoints
    /*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::updateWaypoints() {
        // Received foot positions are mapped relative to robot torso...
        auto joints = calculateLegJointsTeamDarwin(
            kinematicsModel,
            getLeftFootPosition(),
            getRightFootPosition());  // TODO: advised to change to calculateLegJoints (no TeamDarwin)
        auto robotWaypoints = motionLegs(joints);
        auto upperWaypoints = motionArms();

        robotWaypoints->insert(robotWaypoints->end(), upperWaypoints->begin(), upperWaypoints->end());

        return robotWaypoints;
    }
    /*=======================================================================================================*/
    //      NAME: motionArms
    /*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionArms() {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(6);

        NUClear::clock::time_point time =
            NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::R_SHOULDER_PITCH,
                              float(getRArmPosition()[0]),
                              jointGains[ServoID::R_SHOULDER_PITCH],
                              100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::R_SHOULDER_ROLL,
                              float(getRArmPosition()[1]),
                              jointGains[ServoID::R_SHOULDER_ROLL],
                              100});
        waypoints->push_back(
            {subsumptionId, time, ServoID::R_ELBOW, float(getRArmPosition()[2]), jointGains[ServoID::R_ELBOW], 100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::L_SHOULDER_PITCH,
                              float(getLArmPosition()[0]),
                              jointGains[ServoID::L_SHOULDER_PITCH],
                              100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::L_SHOULDER_ROLL,
                              float(getLArmPosition()[1]),
                              jointGains[ServoID::L_SHOULDER_ROLL],
                              100});
        waypoints->push_back(
            {subsumptionId, time, ServoID::L_ELBOW, float(getLArmPosition()[2]), jointGains[ServoID::L_ELBOW], 100});

        return std::move(waypoints);
    }
    /*=======================================================================================================*/
    //      NAME: motionLegs
    /*=======================================================================================================*/
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time =
            NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) {
            // Supports seperate parameterised gains for each leg...
            waypoints->push_back({subsumptionId, time, joint.first, joint.second, jointGains[joint.first], 100});
        }

        return std::move(waypoints);
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Velocity
    /*=======================================================================================================*/
    Transform2D WalkEngine::getVelocity() {
        return velocityCurrent;
    }
    void WalkEngine::setVelocity(Transform2D inVelocityCommand) {
        // hard limit commanded speed ??? not sure if necessary ???
        inVelocityCommand.x() *= inVelocityCommand.x() > 0 ? velocityLimits(0, 1) : -velocityLimits(0, 0);
        inVelocityCommand.y() *= inVelocityCommand.y() > 0 ? velocityLimits(1, 1) : -velocityLimits(1, 0);
        inVelocityCommand.angle() *= inVelocityCommand.angle() > 0 ? velocityLimits(2, 1) : -velocityLimits(2, 0);
        if (DEBUG) {
            log<NUClear::TRACE>("Velocity(hard limit)");
        }

        // filter the commanded speed
        inVelocityCommand.x() = std::min(std::max(inVelocityCommand.x(), velocityLimits(0, 0)), velocityLimits(0, 1));
        inVelocityCommand.y() = std::min(std::max(inVelocityCommand.y(), velocityLimits(1, 0)), velocityLimits(1, 1));
        inVelocityCommand.angle() =
            std::min(std::max(inVelocityCommand.angle(), velocityLimits(2, 0)), velocityLimits(2, 1));
        if (DEBUG) {
            log<NUClear::TRACE>("Velocity(filtered 1)");
        }

        // slow down when turning
        double vFactor = 1 - std::abs(inVelocityCommand.angle()) / accelerationTurningFactor;
        double stepMag =
            std::sqrt(inVelocityCommand.x() * inVelocityCommand.x() + inVelocityCommand.y() * inVelocityCommand.y());
        double magFactor = std::min(velocityLimits(0, 1) * vFactor, stepMag) / (stepMag + 0.000001);

        inVelocityCommand.x()     = inVelocityCommand.x() * magFactor;
        inVelocityCommand.y()     = inVelocityCommand.y() * magFactor;
        inVelocityCommand.angle() = inVelocityCommand.angle();
        if (DEBUG) {
            log<NUClear::TRACE>("Velocity(slow  turn)");
        }

        // filter the decelarated speed
        inVelocityCommand.x() = std::min(std::max(inVelocityCommand.x(), velocityLimits(0, 0)), velocityLimits(0, 1));
        inVelocityCommand.y() = std::min(std::max(inVelocityCommand.y(), velocityLimits(1, 0)), velocityLimits(1, 1));
        inVelocityCommand.angle() =
            std::min(std::max(inVelocityCommand.angle(), velocityLimits(2, 0)), velocityLimits(2, 1));
        if (DEBUG) {
            log<NUClear::TRACE>("Velocity(filtered 2)");
        }

        velocityCurrent = inVelocityCommand;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Time
    /*=======================================================================================================*/
    double WalkEngine::getTime() {
        if (DEBUG) {
            log<NUClear::TRACE>(
                "System Time:%f\n\r",
                double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
        }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: New Step Received
    /*=======================================================================================================*/
    bool WalkEngine::isNewPostureReceived() {
        return (newPostureReceived);
    }
    void WalkEngine::setNewPostureReceived(bool inNewPostureReceived) {
        newPostureReceived = inNewPostureReceived;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Arm Position
    /*=======================================================================================================*/
    arma::vec3 WalkEngine::getLArmPosition() {
        return (armLPostureTransform);
    }
    void WalkEngine::setLArmPosition(arma::vec3 inLArm) {
        armLPostureTransform = inLArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Arm Position
    /*=======================================================================================================*/
    arma::vec3 WalkEngine::getRArmPosition() {
        return (armRPostureTransform);
    }
    void WalkEngine::setRArmPosition(arma::vec3 inRArm) {
        armRPostureTransform = inRArm;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Left Foot Position
    /*=======================================================================================================*/
    Transform3D WalkEngine::getLeftFootPosition() {
        return (leftFootPositionTransform);
    }
    void WalkEngine::setLeftFootPosition(const Transform3D& inLeftFootPosition) {
        leftFootPositionTransform = inLeftFootPosition;
    }
    /*=======================================================================================================*/
    //      ENCAPSULATION METHOD: Right Foot Position
    /*=======================================================================================================*/
    Transform3D WalkEngine::getRightFootPosition() {
        return (rightFootPositionTransform);
    }
    void WalkEngine::setRightFootPosition(const Transform3D& inRightFootPosition) {
        rightFootPositionTransform = inRightFootPosition;
    }
    /*=======================================================================================================*/
    //      INITIALISATION METHOD: Configuration
    /*=======================================================================================================*/
    void WalkEngine::configure(const YAML::Node& config) {
        if (DEBUG) {
            log<NUClear::TRACE>("Configure WalkEngine - Start");
        }
        auto& wlk = config["walk_engine"];

        auto& debug = wlk["debugging"];
        DEBUG       = debug["enabled"].as<bool>();

        auto& servos      = wlk["servos"];
        auto& servos_gain = servos["gain"];
        gainLArm          = servos_gain["left_arm"].as<Expression>();
        gainRArm          = servos_gain["right_arm"].as<Expression>();
        gainLLeg          = servos_gain["left_leg"].as<Expression>();
        gainRLeg          = servos_gain["right_leg"].as<Expression>();
        gainHead          = servos_gain["head"].as<Expression>();

        for (int i = 0; i < ServoID::NUMBER_OF_SERVOS;) {
            if (i < 6) {
                jointGains[i] = gainRArm;
                ++i;
                jointGains[i] = gainLArm;
            }
            else if (i < 18) {
                jointGains[i] = gainRLeg;
                ++i;
                jointGains[i] = gainLLeg;
            }
            else {
                jointGains[i] = gainHead;
            }
        }

        for (auto& gain : servos["gains"]) {
            float p = gain["p"].as<Expression>();
            ServoID sr(gain["id"].as<std::string>(), utility::input::ServoSide::RIGHT);
            ServoID sl(gain["id"].as<std::string>(), utility::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }

        auto& stance          = wlk["stance"];
        STAND_SCRIPT_DURATION = stance["STAND_SCRIPT_DURATION"].as<Expression>();

        auto& walkCycle = wlk["walk_cycle"];
        auto& velocity  = walkCycle["velocity"];
        velocityLimits  = velocity["limits"].as<arma::mat::fixed<3, 2>>();
        velocityHigh    = velocity["high_speed"].as<Expression>();

        auto& acceleration        = walkCycle["acceleration"];
        accelerationLimits        = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh    = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();
        if (DEBUG) {
            log<NUClear::TRACE>("Configure WalkEngine - Finish");
        }
    }
}  // namespace motion
}  // namespace module
