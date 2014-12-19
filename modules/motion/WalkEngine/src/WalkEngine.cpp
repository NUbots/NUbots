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

#include "WalkEngine.h"

#include <algorithm>
#include <armadillo>
#include <chrono>
#include <cmath>

#include "messages/support/Configuration.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/ServoTarget.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/FixedWalkCommand.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/support/YamlArmadillo.h"
#include "utility/support/YamlExpression.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/matrix.h"
#include "utility/math/angle.h"

namespace modules {
namespace motion {

    using messages::input::ServoID;
    using messages::input::Sensors;
    using messages::behaviour::LimbID;
    using messages::behaviour::ServoCommand;
    using messages::behaviour::WalkOptimiserCommand;
    using messages::behaviour::WalkConfigSaved;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::LimbID;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::WalkStopped;
    using messages::motion::ServoTarget;
    using messages::motion::Script;
    using messages::support::SaveConfiguration;
    using messages::support::Configuration;

    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::angle::normalizeAngle;
    using utility::math::matrix::vec6ToMatrix;
    using utility::math::matrix::orthonormal44Inverse;
    using utility::nubugger::graph;
    using utility::support::Expression;

    WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Walk Engine",
            {
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // legs are available, start
                    //stanceReset();
                    //updateHandle.enable();
                    interrupted = false;
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // legs are no longer available, reset walking (too late to stop walking)
                    //updateHandle.disable();
                    interrupted = true;
                }
            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));

        updateHandle = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds> > >, With<Sensors>, Options< Single, Priority<NUClear::HIGH>> >([this](const time_t&, const Sensors& sensors) {
            emit(update(sensors));
        });

        //updateHandle.disable();

        on<Trigger<WalkCommand>>([this](const WalkCommand& walkCommand) {
            setVelocity(walkCommand.velocity[0] * (walkCommand.velocity[0] > 0 ? velocityLimits(0,1) : -velocityLimits(0,0)),
                        walkCommand.velocity[1] * (walkCommand.velocity[1] > 0 ? velocityLimits(1,1) : -velocityLimits(1,0)),
                        walkCommand.rotationalSpeed * (walkCommand.rotationalSpeed > 0 ? velocityLimits(2,1) : -velocityLimits(2,0)));
        });

        on<Trigger<WalkStartCommand>>([this](const WalkStartCommand&) {
            start();
            emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 25, 10 }})); // TODO: config
        });

        on<Trigger<WalkStopCommand>>([this](const WalkStopCommand&) {
            requestStop();
        });

        on<Trigger<Configuration<WalkEngine>> >([this](const Configuration<WalkEngine>& config) {
            configureWalk(config.config);
        });
        on<Trigger<WalkOptimiserCommand> >([this](const WalkOptimiserCommand& command) {
            configureWalk(command.walkConfig);
            emit(std::make_unique<WalkConfigSaved>());
        });

        on<Trigger<Startup>>([this](const Startup&) {
            generateAndSaveStandScript();
            reset();
            state = State::LAST_STEP;
            //start();
        });

    }

    void WalkEngine::configureWalk(const YAML::Node& config){

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArm = stance["arms"]["left"].as<arma::vec>();
        qRArm = stance["arms"]["right"].as<arma::vec>();
        footOffset = stance["foot_offset"].as<arma::vec>();

        auto& gains = stance["gains"];
        gainArms = gains["arms"].as<Expression>();
        gainLegs = gains["legs"].as<Expression>();

        auto& walkCycle = config["walk_cycle"];
        tStep = walkCycle["step_time"].as<Expression>();
        tZmp = walkCycle["zmp_time"].as<Expression>();
        stepHeight = walkCycle["step"]["height"].as<Expression>();
        stepLimits = walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh = velocity["high_speed"].as<Expression>();

        auto& acceleration = walkCycle["acceleration"];
        accelerationLimits = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - config["stanceLimitMarginY"].as<Expression>();

        // gCompensation parameters
        hipRollCompensation = config["hipRollCompensation"].as<Expression>();
        toeTipCompensation = config["toeTipCompensation"].as<Expression>();
        ankleMod = {-toeTipCompensation, 0};

        turnCompThreshold = config["turnCompThreshold"].as<Expression>();
        turnComp = config["turnComp"].as<Expression>();

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

        frontComp = config["frontComp"].as<Expression>();
        accelComp = config["accelComp"].as<Expression>();

        balanceWeight = config["balanceWeight"].as<Expression>();

        // gInitial body swing
        supportModYInitial = config["supportModYInitial"].as<Expression>();

        STAND_SCRIPT_DURATION_MILLISECONDS = config["STAND_SCRIPT_DURATION_MILLISECONDS"].as<int>();
    }

    void WalkEngine::generateAndSaveStandScript(){
        reset();
        stanceReset();
        auto waypoints = updateStill();

        Script standScript;
        Script::Frame frame;
        frame.duration = std::chrono::milliseconds(STAND_SCRIPT_DURATION_MILLISECONDS);
        for(auto& waypoint : *waypoints){
            frame.targets.push_back(Script::Frame::Target({waypoint.id,
                                                           waypoint.position,
                                                           waypoint.gain}
                                                          )
                                   );
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

    void WalkEngine::reset() {
            // Global walk state variables

            uTorso = {-footOffset[0], 0, 0};
            uLeftFoot = {0, DarwinModel::Leg::HIP_OFFSET_Y, 0};
            uRightFoot = {0, -DarwinModel::Leg::HIP_OFFSET_Y, 0};

            velocityCurrent = {0, 0, 0};
            velocityCommand = {0, 0, 0};
            velocityDifference = {0, 0, 0};

            // gZMP exponential coefficients:
            zmpCoefficients = arma::zeros(4);
            zmpParams = arma::zeros(4);

            // gGyro stabilization variables
            ankleShift = {0, 0};
            kneeShift = 0;
            hipShift = {0, 0};
            armShift = {0, 0};

            active = false;
            started = false;
            swingLeg = swingLegInitial;
            beginStepTime = getTime();
            phase=0;
            currentStepType = 0;

            initialStep = 2;

            phaseSingle = 0;

            // gStandard offset
            uLRFootOffset = {0, DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], 0};

            // gWalking/Stepping transition variables
            startFromStep = false;

            state = State::STOPPED;

            interrupted = false;

            stanceReset();
    }

    void WalkEngine::start() {
        if (state != State::WALKING) {
            active = true;
            started = false;
            swingLeg = swingLegInitial;
            beginStepTime = getTime();
            initialStep = 2;
            state = State::WALKING;
        }
    }

    void WalkEngine::requestStop() {
        // always stops with feet together (which helps transition)
        if (state == State::WALKING) {
            state = State::STOP_REQUEST;
        }
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::stop() {
        state = State::STOPPED;
        active = false;
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 0, 0 }})); // TODO: config
        log<NUClear::TRACE>("Walk Engine:: Stop request complete");
        emit(std::make_unique<WalkStopped>());

        return std::make_unique<std::vector<ServoCommand>>();
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::update(const Sensors& sensors) {
        double time = getTime();

        if (!active) {
            return updateStill(sensors);
        }

        if (!started) {
            started = true;
            beginStepTime = time;
        }

        // phase of step
        phase = (time - beginStepTime) / tStep;

        bool newStep = false;

        if (phase > 1) {
            phase = std::fmod(phase, 1);
            beginStepTime += tStep;
            newStep = true;
        }

        if (newStep && state == State::LAST_STEP) {
            return stop();
        }

        if (newStep) {
            calculateNewStep();
        }

        return updateStep(sensors);
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::updateStep(const Sensors& sensors) {
        arma::vec3 foot = footPhase(phase);
        if (initialStep > 0) {
            foot[2] = 0; // don't lift foot at initial step
        }
        if (swingLeg == LimbID::RIGHT_LEG) {
            uRightFoot = se2Interpolate(foot[0], uRightFootSource, uRightFootDestination);
        } else {
            uLeftFoot = se2Interpolate(foot[0], uLeftFootSource, uLeftFootDestination);
        }

        uTorso = zmpCom(phase, zmpCoefficients, zmpParams, tStep, tZmp, phase1Single, phase2Single);

        // turning
        double turnCompX = 0;
        if (std::abs(velocityCurrent[2]) > turnCompThreshold && velocityCurrent[0] > -0.01) {
            turnCompX = turnComp;
        }

        // walking front
        double frontCompX = 0;
        if (velocityCurrent[0] > 0.04) {
            frontCompX = frontComp;
        }
        if (velocityDifference[0] > 0.02) {
            frontCompX = frontCompX + accelComp;
        }

        arma::vec3 uTorsoActual = localToWorld({-DarwinModel::Leg::HIP_OFFSET_X + frontCompX + turnCompX, 0, 0}, uTorso);
        arma::vec6 pTorso = {uTorsoActual[0], uTorsoActual[1], bodyHeight, 0, bodyTilt, uTorsoActual[2]};
        arma::vec6 pLeftFoot = {uLeftFoot[0], uLeftFoot[1], 0, 0, 0, uLeftFoot[2]};
        arma::vec6 pRightFoot = {uRightFoot[0], uRightFoot[1], 0, 0, 0, uRightFoot[2]};

        if (swingLeg == LimbID::RIGHT_LEG) {
            pRightFoot[2] = stepHeight * foot[2];
        } else {
            pLeftFoot[2] = stepHeight * foot[2];
        }

        arma::mat44 torso = vec6ToMatrix(pTorso);
        arma::mat44 torsoInv = orthonormal44Inverse(torso);
        arma::mat44 leftFoot = torsoInv * vec6ToMatrix(pLeftFoot);
        arma::mat44 rightFoot = torsoInv * vec6ToMatrix(pRightFoot);

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFoot, rightFoot);
        auto waypoints = motionLegs(joints, sensors);
        //std::vector<std::pair<messages::input::ServoID, double>>

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::updateStill(const Sensors& sensors) {
        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);
        uTorsoActual = localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0}, uTorso);
        arma::vec6 pTorso = {uTorsoActual[0], uTorsoActual[1], bodyHeight, 0, bodyTilt, uTorsoActual[2]};
        arma::vec6 pLeftFoot = {uLeftFoot[0], uLeftFoot[1], 0, 0, 0, uLeftFoot[2]};
        arma::vec6 pRightFoot = {uRightFoot[0], uRightFoot[1], 0, 0, 0, uRightFoot[2]};

        arma::mat44 torso = vec6ToMatrix(pTorso);
        arma::mat44 torsoInv = orthonormal44Inverse(torso);
        arma::mat44 leftFoot = torsoInv * vec6ToMatrix(pLeftFoot);
        arma::mat44 rightFoot = torsoInv * vec6ToMatrix(pRightFoot);

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFoot, rightFoot);
        auto waypoints = motionLegs(joints, sensors);

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }
    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints, const Sensors& sensors) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        // balance(qLegs, sensors);

        time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) {
            waypoints->push_back({id, time, joint.first, joint.second, gainLegs}); // TODO: support separate gains for each leg
        }

        return std::move(waypoints);
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionArms() {

        arma::vec3 qLArmActual = qLArm;
        arma::vec3 qRArmActual = qRArm;

        qLArmActual.rows(0,1) += armShift;
        qRArmActual.rows(0,1) += armShift;

        // Start arm/leg collision/prevention
        double rotLeftA = normalizeAngle(uLeftFoot[2] - uTorso[2]);
        double rotRightA = normalizeAngle(uTorso[2] - uRightFoot[2]);
        arma::vec3 leftLegTorso = worldToLocal(uLeftFoot, uTorso);
        arma::vec3 rightLegTorso = worldToLocal(uRightFoot, uTorso);
        double leftMinValue = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2 + std::max(0.0, leftLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2 - std::max(0.0, -rightLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180);
        // update shoulder pitch to move arm away from body
        qLArmActual[1] = std::max(leftMinValue, qLArmActual[1]);
        qRArmActual[1] = std::min(rightMinValue, qRArmActual[1]);
        // End arm/leg collision/prevention

        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(6);

        time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);
        waypoints->push_back({id, time, ServoID::R_SHOULDER_PITCH, float(qRArmActual[0]), gainArms});
        waypoints->push_back({id, time, ServoID::R_SHOULDER_ROLL,  float(qRArmActual[1]), gainArms});
        waypoints->push_back({id, time, ServoID::R_ELBOW,          float(qRArmActual[2]), gainArms});
        waypoints->push_back({id, time, ServoID::L_SHOULDER_PITCH, float(qLArmActual[0]), gainArms});
        waypoints->push_back({id, time, ServoID::L_SHOULDER_ROLL,  float(qLArmActual[1]), gainArms});
        waypoints->push_back({id, time, ServoID::L_ELBOW,          float(qLArmActual[2]), gainArms});

        return std::move(waypoints);
    }

    arma::vec3 WalkEngine::stepTorso(arma::vec3 uLeftFoot, arma::vec3 uRightFoot, double shiftFactor) {
        arma::vec3 uLeftFootSupport = localToWorld({-footOffset[0], -footOffset[1], 0}, uLeftFoot);
        arma::vec3 uRightFootSupport = localToWorld({-footOffset[0], footOffset[1], 0}, uRightFoot);
        return se2Interpolate(shiftFactor, uLeftFootSupport, uRightFootSupport);
    }

    void WalkEngine::setVelocity(double vx, double vy, double va) {
        // filter the commanded speed
        vx = std::min(std::max(vx, velocityLimits(0,0)), velocityLimits(0,1));
        vy = std::min(std::max(vy, velocityLimits(1,0)), velocityLimits(1,1));
        va = std::min(std::max(va, velocityLimits(2,0)), velocityLimits(2,1));

        // slow down when turning
        double vFactor = 1 - std::abs(va) / accelerationTurningFactor;

        double stepMag = std::sqrt(vx * vx + vy * vy);
        double magFactor = std::min(velocityLimits(0,1) * vFactor, stepMag) / (stepMag + 0.000001);

        velocityCommand[0] = vx * magFactor;
        velocityCommand[1] = vy * magFactor;
        velocityCommand[2] = va;

        velocityCommand[0] = std::min(std::max(velocityCommand[0], velocityLimits(0,0)), velocityLimits(0,1));
        velocityCommand[1] = std::min(std::max(velocityCommand[1], velocityLimits(1,0)), velocityLimits(1,1));
        velocityCommand[2] = std::min(std::max(velocityCommand[2], velocityLimits(2,0)), velocityLimits(2,1));
    }

    arma::vec3 WalkEngine::getVelocity() {
        return velocityCurrent;
    }

    void WalkEngine::stanceReset() {
        // standup/sitdown/falldown handling
        if (startFromStep) {
            uLeftFoot = arma::zeros(3);
            uRightFoot = arma::zeros(3);
            uTorso = arma::zeros(3);

            // start walking asap
            initialStep = 1;
        } else {
            // stance resetted
            uLeftFoot = localToWorld({footOffset[0], DarwinModel::Leg::HIP_OFFSET_Y, 0}, uTorso);
            uRightFoot = localToWorld({footOffset[0], -DarwinModel::Leg::HIP_OFFSET_Y, 0}, uTorso);
        }
        swingLeg = swingLegInitial;

        uLeftFootSource = uLeftFoot;
        uLeftFootDestination = uLeftFoot;

        uRightFootSource = uRightFoot;
        uRightFootDestination = uRightFoot;

        uSupport = uTorso;
        beginStepTime = getTime();
        currentStepType = 0;
        uLRFootOffset = {0, DarwinModel::Leg::HIP_OFFSET_Y, 0};
        startFromStep = false;
    }

    /**
    * Global variables used:
    * tStep, phase1Single, phase2Single, tZmp
    */
    arma::vec2 WalkEngine::zmpSolve(double zs, double z1, double z2, double x1, double x2) {
        /*
        Solves ZMP equation:
        x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
        where the ZMP point is piecewise linear:
        z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
        */
        double T1 = tStep * phase1Single;
        double T2 = tStep * phase2Single;
        double m1 = (zs - z1) / T1;
        double m2 = -(zs - z2) / (tStep - T2);

        double c1 = x1 - z1 + tZmp * m1 * std::sinh(-T1 / tZmp);
        double c2 = x2 - z2 + tZmp * m2 * std::sinh((tStep - T2) / tZmp);
        double expTStep = std::exp(tStep / tZmp);
        double aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        double aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }

    /**
    * Global variables used:
    * uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource
    */
    arma::vec3 WalkEngine::zmpCom(double phase, arma::vec4 zmpCoefficients, arma::vec4 zmpParams, double tStep, double tZmp, double phase1Single, double phase2Single) {
        arma::vec3 com = {0, 0, 0};
        double expT = std::exp(tStep * phase / tZmp);
        com[0] = uSupport[0] + zmpCoefficients[0] * expT + zmpCoefficients[1] / expT;
        com[1] = uSupport[1] + zmpCoefficients[2] * expT + zmpCoefficients[3] / expT;
        if (phase < phase1Single) {
            com[0] = com[0] + zmpParams[0] * tStep * (phase - phase1Single) -tZmp * zmpParams[0] * std::sinh(tStep * (phase - phase1Single) / tZmp);
            com[1] = com[1] + zmpParams[1] * tStep * (phase - phase1Single) -tZmp * zmpParams[1] * std::sinh(tStep * (phase - phase1Single) / tZmp);
        } else if (phase > phase2Single) {
            com[0] = com[0] + zmpParams[2] * tStep * (phase - phase2Single) -tZmp * zmpParams[2] * std::sinh(tStep * (phase - phase2Single) / tZmp);
            com[1] = com[1] + zmpParams[3] * tStep * (phase - phase2Single) -tZmp * zmpParams[3] * std::sinh(tStep * (phase - phase2Single) / tZmp);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com[2] = phase * (uLeftFootDestination[2] + uRightFootDestination[2]) / 2 + (1 - phase) * (uLeftFootSource[2] + uRightFootSource[2]) / 2;
        return com;
    }

    double WalkEngine::getTime() {
          struct timeval t;
          gettimeofday(&t, NULL);
          return t.tv_sec + 1E-6 * t.tv_usec;
    }

    double WalkEngine::procFunc(double value, double deadband, double maxvalue) {
        // a function for IMU feedback (originally from teamdarwin2013release/player/util/util.lua)
        // clamp between 0 and maxvalue
        // offset using deadband
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }

    arma::vec3 WalkEngine::localToWorld(arma::vec3 poseRelative, arma::vec3 pose) {
        double ca = std::cos(pose[2]);
        double sa = std::sin(pose[2]);
        // translates to pose + rotZ(pose.angle) * poseRelative
        return {
            pose[0] + ca * poseRelative[0] - sa * poseRelative[1],
            pose[1] + sa * poseRelative[0] + ca * poseRelative[1],
            pose[2] + poseRelative[2] // do not use normalizeAngle here, causes bad things when turning!
        };
    }

    arma::vec3 WalkEngine::worldToLocal(arma::vec3 poseGlobal, arma::vec3 pose) {
        double ca = std::cos(pose[2]);
        double sa = std::sin(pose[2]);
        double px = poseGlobal[0] - pose[0];
        double py = poseGlobal[1] - pose[1];
        double pa = poseGlobal[2] - pose[2];
        // translates to rotZ(pose.angle) * (poseGlobal - pose)
        return {
            ca * px + sa * py,
            -sa * px + ca * py,
            normalizeAngle(pa)
        };
    }

    arma::vec3 WalkEngine::se2Interpolate(double t, arma::vec3 u1, arma::vec3 u2) {
        // helps smooth out the motions using a weighted average
        return {
            u1[0] + t * (u2[0] - u1[0]),
            u1[1] + t * (u2[1] - u1[1]),
            u1[2] + t * normalizeAngle(u2[2] - u1[2])
        };
    }

}  // motion
}  // modules

