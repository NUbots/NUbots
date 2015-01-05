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
#include "messages/localisation/FieldObject.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/matrix/Transform.h"
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
    using utility::math::SE2;
    using utility::math::matrix::Transform;
    using utility::math::angle::normalizeAngle;
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
                    stanceReset(); // reset stance as we don't know where our limbs are
                    interrupted = false;
                    if (state == State::WALKING) {
                        updateHandle.enable();
                    }
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // legs are no longer available, reset walking (too late to stop walking)
                    updateHandle.disable();
                    interrupted = true;
                }
            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));

        updateHandle = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, Options<Single, Priority<NUClear::HIGH>>>([this](const time_t&, const Sensors& sensors) {
            update(sensors);
        }).disable();

        on<Trigger<WalkCommand>>([this](const WalkCommand& walkCommand) {
            auto velocity = walkCommand.command;

            velocity.x()     *= velocity.x()     > 0 ? velocityLimits(0,1) : -velocityLimits(0,0);
            velocity.y()     *= velocity.y()     > 0 ? velocityLimits(1,1) : -velocityLimits(1,0);
            velocity.angle() *= velocity.angle() > 0 ? velocityLimits(2,1) : -velocityLimits(2,0);

            setVelocity(velocity);
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
            //generateAndSaveStandScript();
            //reset();
            //state = State::LAST_STEP;
            //start();
        });

        reset();

    }

    void WalkEngine::configureWalk(const YAML::Node& config){
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArm = stance["arms"]["left"].as<arma::vec>();
        qRArm = stance["arms"]["right"].as<arma::vec>();
        footOffset = stance["foot_offset"].as<arma::vec>();
        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();

        auto& gains = stance["gains"];
        gainArms = gains["arms"].as<Expression>();
        gainLegs = gains["legs"].as<Expression>();

        auto& walkCycle = config["walk_cycle"];
        stepTime = walkCycle["step_time"].as<Expression>();
        zmpTime = walkCycle["zmp_time"].as<Expression>();
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


        // gCompensation parameters
        // hipRollCompensation = config["hipRollCompensation"].as<Expression>();
        // toeTipCompensation = config["toeTipCompensation"].as<Expression>();
        // ankleMod = {-toeTipCompensation, 0};

        /*// gGyro stabilization parameters
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

        balanceWeight = config["balanceWeight"].as<Expression>();*/

        // STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }

    /*void WalkEngine::generateAndSaveStandScript(){
        reset();
        stanceReset();
        auto waypoints = updateStill();

        Script standScript;
        Script::Frame frame;
        frame.duration = std::chrono::milliseconds(int(round(1000 * STAND_SCRIPT_DURATION)));
        for (auto& waypoint : *waypoints) {
            frame.targets.push_back(Script::Frame::Target({waypoint.id, waypoint.position, waypoint.gain}));
        }
        standScript.frames.push_back(frame);
        auto saveScript = std::make_unique<SaveConfiguration>();
        saveScript->path = "scripts/Stand.yaml";
        saveScript->config = standScript;
        emit(std::move(saveScript));
        //Try update(); ?
        reset();
        stanceReset();
    }*/

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
            phase = 0;
            currenstepTimeType = 0;

            initialStep = 2;

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
            updateHandle.enable();
        }
    }

    void WalkEngine::requestStop() {
        // always stops with feet together (which helps transition)
        if (state == State::WALKING) {
            state = State::STOP_REQUEST;
        }
    }

    void WalkEngine::stop() {
        state = State::STOPPED;
        active = false;
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 0, 0 }})); // TODO: config
        updateHandle.disable();
        log<NUClear::TRACE>("Walk Engine:: Stop request complete");
        emit(std::make_unique<WalkStopped>());
        emit(std::make_unique<std::vector<ServoCommand>>());
    }

    void WalkEngine::localise(SE2 position) {
        // emit position as a fake localisation
        auto localisation = std::make_unique<std::vector<messages::localisation::Self>>();
        messages::localisation::Self self;
        self.position = {position.x(), position.y()};
        self.position_cov = arma::eye(2,2) * 0.1; // made up
        self.heading = {std::cos(position.angle()), std::sin(position.angle())}; // convert to cartesian coordinates
        self.velocity = arma::zeros(2); // not used
        self.robot_to_world_rotation = arma::zeros(2,2); // not used
        localisation->push_back(self);
        emit(std::move(localisation));
    }

    void WalkEngine::update(const Sensors& sensors) {
        double now = getTime();

        if (!active) {
            updateStill(sensors);
            return;
        }

        if (!started) {
            started = true;
            beginStepTime = now;
        }

        // phase of step
        phase = (now - beginStepTime) / stepTime;

        bool newStep = false;

        if (phase > 1) {
            phase = std::fmod(phase, 1);
            beginStepTime += stepTime;
            newStep = true;
        }

        if (newStep && state == State::LAST_STEP) {
            stop();
            return;
        }

        if (newStep) {
            calculateNewStep();
        }

        updateStep(sensors);
    }

    void WalkEngine::updateStep(const Sensors&/* sensors*/) {
        arma::vec3 foot = footPhase(phase, phase1Single, phase2Single);
        if (initialStep > 0) {
            foot[2] = 0; // don't lift foot at initial step, TODO: review
        }
        if (swingLeg == LimbID::RIGHT_LEG) {
            uRightFoot = uRightFootSource.interpolate(foot[0], uRightFootDestination);
        } else {
            uLeftFoot = uLeftFootSource.interpolate(foot[0], uLeftFootDestination);
        }

        uTorso = zmpCom(phase, zmpCoefficients, zmpParams, stepTime, zmpTime, phase1Single, phase2Single, uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource);

        Transform leftFoot = uLeftFoot;
        Transform rightFoot = uRightFoot;

        if (swingLeg == LimbID::RIGHT_LEG) {
            rightFoot = rightFoot.translateZ(stepHeight * foot[2]);
        } else {
            leftFoot = leftFoot.translateZ(stepHeight * foot[2]);
        }

        SE2 uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});
        Transform torso = arma::vec6{uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()};

        // Transform feet targets to be relative to the torso
        Transform leftFootTorso = leftFoot.worldToLocal(torso);
        Transform rightFootTorso = rightFoot.worldToLocal(torso);

        if (emitLocalisation) {
            localise(uTorsoActual);
        }

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
        // TODO: balance(joints, sensors);
        auto waypoints = motionLegs(joints);

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        emit(std::move(waypoints));
    }

    void WalkEngine::updateStill(const Sensors&/* sensors*/) {
        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);
        SE2 uTorsoActual = uTorso.localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});

        Transform torso = arma::vec6{uTorsoActual.x(), uTorsoActual.y(), bodyHeight, 0, bodyTilt, uTorsoActual.angle()};

        // Transform feet targets to be relative to the torso
        Transform leftFootTorso = static_cast<Transform>(uLeftFoot).worldToLocal(torso);
        Transform rightFootTorso = static_cast<Transform>(uRightFoot).worldToLocal(torso);

        if (emitLocalisation) {
            localise(uTorsoActual);
        }

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFootTorso, rightFootTorso);
        // TODO: balance(joints, sensors);
        auto waypoints = motionLegs(joints);

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        emit(std::move(waypoints));
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionLegs(std::vector<std::pair<ServoID, float>> joints) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) {
            waypoints->push_back({id, time, joint.first, joint.second, gainLegs}); // TODO: support separate gains for each leg
        }

        return std::move(waypoints);
    }

    std::unique_ptr<std::vector<ServoCommand>> WalkEngine::motionArms() {

        auto qLArmActual = qLArm;
        auto qRArmActual = qRArm;

        qLArmActual.rows(0,1) += armShift;
        qRArmActual.rows(0,1) += armShift;

        // Start arm/leg collision/prevention
        double rotLeftA = normalizeAngle(uLeftFoot.angle() - uTorso.angle());
        double rotRightA = normalizeAngle(uTorso.angle() - uRightFoot.angle());
        SE2 leftLegTorso = uTorso.worldToLocal(uLeftFoot);
        SE2 rightLegTorso = uTorso.worldToLocal(uRightFoot);
        double leftMinValue = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2 + std::max(0.0, leftLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2 - std::max(0.0, -rightLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
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

    SE2 WalkEngine::stepTorso(SE2 uLeftFoot, SE2 uRightFoot, double shiftFactor) {
        SE2 uLeftFootSupport = uLeftFoot.localToWorld({-footOffset[0], -footOffset[1], 0});
        SE2 uRightFootSupport = uRightFoot.localToWorld({-footOffset[0], footOffset[1], 0});
        return uLeftFootSupport.interpolate(shiftFactor, uRightFootSupport);
    }

    void WalkEngine::setVelocity(SE2 velocity) {
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

    SE2 WalkEngine::getVelocity() {
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
            uLeftFoot = uTorso.localToWorld({footOffset[0], DarwinModel::Leg::HIP_OFFSET_Y, 0});
            uRightFoot = uTorso.localToWorld({footOffset[0], -DarwinModel::Leg::HIP_OFFSET_Y, 0});
        }

        swingLeg = swingLegInitial;

        uLeftFootSource = uLeftFoot;
        uLeftFootDestination = uLeftFoot;

        uRightFootSource = uRightFoot;
        uRightFootDestination = uRightFoot;

        uSupport = uTorso;
        beginStepTime = getTime();
        currenstepTimeType = 0;
        uLRFootOffset = {0, DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], 0};
        startFromStep = false;
    }

    arma::vec2 WalkEngine::zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) {
        /*
        Solves ZMP equation:
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

    SE2 WalkEngine::zmpCom(double phase, arma::vec4 zmpCoefficients, arma::vec4 zmpParams, double stepTime, double zmpTime, double phase1Single, double phase2Single, SE2 uSupport, SE2 uLeftFootDestination, SE2 uLeftFootSource, SE2 uRightFootDestination, SE2 uRightFootSource) {
        SE2 com = {0, 0, 0};
        double expT = std::exp(stepTime * phase / zmpTime);
        com.x() = uSupport.x() + zmpCoefficients[0] * expT + zmpCoefficients[1] / expT;
        com.y() = uSupport.y() + zmpCoefficients[2] * expT + zmpCoefficients[3] / expT;
        if (phase < phase1Single) {
            com.x() += zmpParams[0] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[0] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
            com.y() += zmpParams[1] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[1] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
        } else if (phase > phase2Single) {
            com.x() += zmpParams[2] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[2] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
            com.y() += zmpParams[3] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[3] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com.angle() = phase * (uLeftFootDestination.angle() + uRightFootDestination.angle()) / 2 + (1 - phase) * (uLeftFootSource.angle() + uRightFootSource.angle()) / 2;
        return com;
    }

    double WalkEngine::getTime() {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }

    double WalkEngine::procFunc(double value, double deadband, double maxvalue) {
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }

}  // motion
}  // modules

