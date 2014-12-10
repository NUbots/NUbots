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

#include "messages/behaviour/Action.h"
#include "messages/support/Configuration.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"
#include "utility/math/matrix.h"
#include "OPKinematics.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/YamlArmadillo.h"
#include "utility/support/YamlExpression.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/ServoTarget.h"
#include "messages/behaviour/Action.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/FixedWalkCommand.h"

namespace modules {
namespace motion {

    using messages::input::ServoID;
    using messages::behaviour::ServoCommand;
    using messages::behaviour::WalkOptimiserCommand;
    using messages::behaviour::WalkConfigSaved;
    using messages::support::Configuration;
    using utility::motion::kinematics::DarwinModel;
    using utility::nubugger::graph;
    using NUClear::log;
    using NUClear::DEBUG;
    using messages::input::Sensors;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::motion::WalkStopped;
    using messages::motion::ServoTarget;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::LimbID;
    using messages::motion::Script;
    using messages::support::SaveConfiguration;


    WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Walk Engine",
            {
                std::pair<float, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<float, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // legs are available, start
                    //stanceReset();
                    //updateHandle.enable();
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // legs are no longer available, reset walking (too late to stop walking)
                    //updateHandle.disable();
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
            stop();
            // TODO: set priorities to 0 when stopped - somehow
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
            stopRequest = StopRequest::LAST_STEP;
            //start();
        });

    }
    // TODO: add others
    void WalkEngine::configureWalk(const YAML::Node& config){
        // g Walk Parameters
        // g Stance and velocity limit values
        stanceLimits = config["stanceLimits"].as<arma::mat::fixed<3,2>>();
        velocityLimits = config["velocityLimits"].as<arma::mat::fixed<3,2>>();
        velocityDelta = config["velocityDelta"].as<arma::vec>();
        velocityAngleFactor = config["velocityAngleFactor"].as<float>();

        velocityXHigh = config["velocityXHigh"].as<double>();
        velocityDeltaXHigh = config["velocityDeltaXHigh"].as<double>();

        // gToe/heel overlap checking values
        footSizeX = config["footSizeX"].as<arma::vec>();
        stanceLimitMarginY = config["stanceLimitMarginY"].as<float>();
        stanceLimitY2 = 2 * config["footY"].as<double>() - config["stanceLimitMarginY"].as<double>();

        // gOP default stance width: 0.0375*2 = 0.075
        // gHeel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
        // gHeel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

        // gStance parameters
        bodyHeight = config["bodyHeight"].as<float>();
        bodyTilt = config["bodyTilt"].as<float>();
        footX = config["footX"].as<float>();
        footY = config["footY"].as<float>();
        supportX = config["supportX"].as<float>();
        supportY = config["supportY"].as<float>();

        qLArm = config["armPose"][0].as<arma::vec>();
        qRArm = config["armPose"][1].as<arma::vec>();

        // gHardness parameters
        hardnessSupport = config["hardnessSupport"].as<float>();
        hardnessSwing = config["hardnessSwing"].as<float>();
        hardnessArm = config["hardnessArm"].as<float>();

        // gGait parameters

        tStep = config["tStep"].as<float>();
        tStep0 = tStep;
        tZmp = config["tZmp"].as<float>();
        stepHeight = config["stepHeight"].as<float>();
        phase1Single = config["phaseSingle"][0].as<float>();
        phase2Single = config["phaseSingle"][1].as<float>();
        phase1Zmp = phase1Single;
        phase2Zmp = phase2Single;

        // gCompensation parameters
        hipRollCompensation = config["hipRollCompensation"].as<utility::support::Expression>();
        toeTipCompensation = config["toeTipCompensation"].as<utility::support::Expression>();
        ankleMod = {-toeTipCompensation, 0};

        turnCompThreshold = config["turnCompThreshold"].as<float>();
        turnComp = config["turnComp"].as<float>();

        // gGyro stabilization parameters
        ankleImuParamX = config["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY = config["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX = config["kneeImuParamX"].as<arma::vec>();
        hipImuParamY = config["hipImuParamY"].as<arma::vec>();
        armImuParamX = config["armImuParamX"].as<arma::vec>();
        armImuParamY = config["armImuParamY"].as<arma::vec>();

        // gSupport bias parameters to reduce backlash-based instability
        velFastForward = config["velFastForward"].as<float>();
        velFastTurn = config["velFastTurn"].as<float>();
        supportFront = config["supportFront"].as<float>();
        supportFront2 = config["supportFront2"].as<float>();
        supportBack = config["supportBack"].as<float>();
        supportSideX = config["supportSideX"].as<float>();
        supportSideY = config["supportSideY"].as<float>();
        supportTurn = config["supportTurn"].as<float>();

        frontComp = config["frontComp"].as<float>();
        accelComp = config["accelComp"].as<float>();

        balanceWeight = config["balanceWeight"].as<float>();

        // gInitial body swing
        supportModYInitial = config["supportModYInitial"].as<float>();

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

    void WalkEngine::reset(){
            // Global walk state variables

            uTorso = {supportX, 0, 0};
            uLeftFoot = {0, footY, 0};
            uRightFoot = {0, -footY, 0};

            pLLeg = {0, footY, 0, 0, 0, 0};
            pRLeg = {0, -footY, 0, 0, 0, 0};
            pTorso = {supportX, 0, bodyHeight, 0, bodyTilt, 0};

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
            swingLeg = Leg::LEFT;
            tLastStep = getTime();
            phase=0;
            currentStepType = 0;

            initialStep = 2;

            phaseSingle = 0;

            // gStandard offset
            uLRFootOffset = {0, footY + supportY, 0};

            // gWalking/Stepping transition variables
            uLeftFootI = {0, 0, 0};
            uRightFootI = {0, 0, 0};
            uTorsoI = {0, 0, 0};
            supportInitial = Leg::LEFT;
            startFromStep = false;

            stanceReset();
    }

    void WalkEngine::start() {
        stopRequest = StopRequest::NONE;
        if (!active) {
            double now = getTime();

            active = true;
            started = false;
            swingLeg = Leg::LEFT;
            tLastStep = now;
            initialStep = 2;
        }
    }

    void WalkEngine::stop() {
        // always stops with feet together (which helps transition)
        if (stopRequest == StopRequest::NONE) {
            stopRequest = StopRequest::REQUESTED;
        }
    }

    std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::update(const Sensors& sensors) {
        //advanceMotion();
        double time = getTime();

        // TODO: bodyHeightCurrent = vcm.get_camera_bodyHeight();

//            log<DEBUG>("velocityCurrent: ", velocityCurrent);
//            log<DEBUG>("velocityCommand: ", velocityCommand);
        if (!active) {
            return updateStill(sensors);
        }

        if (!started) {
            started = true;
            tLastStep = time;
        }

        // phase of step
        phase = (time - tLastStep) / tStep;

        bool newStep = false;

        if (phase > 1) {
            phase = phase - std::floor(phase);
            tLastStep += tStep;
            newStep = true;
        }

        if (newStep && stopRequest == StopRequest::LAST_STEP) {
            stopRequest = StopRequest::NONE;
            active = false;
            emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 0, 0 }})); // TODO: config
            std::cout << "Walk Engine:: stop request complete" << std::endl;
            emit(std::make_unique<WalkStopped>());

            return std::make_unique<std::vector<ServoCommand>>(); // TODO: return "stop"
        }

        // new step
        if (newStep) {
            updateVelocity();

            // swap swing and support legs
            swingLeg = swingLeg == Leg::LEFT ? Leg::RIGHT : Leg::LEFT;
            supportLeg = swingLeg == Leg::LEFT ? Leg::RIGHT : Leg::LEFT;

            uLeftFootSource = uLeftFootDestination;
            uRightFootSource = uRightFootDestination;
            uTorsoSource = uTorsoDestination;

            supportMod = {0, 0}; // support point modulation for wallkick
            shiftFactor = 0.5; // how much should we shift final torso pose?

            if (stopRequest == StopRequest::REQUESTED) {
                log<DEBUG>("stop request 1");
                stopRequest = StopRequest::LAST_STEP;
                velocityCurrent = {0, 0, 0};
                velocityCommand = {0, 0, 0};
                if (supportLeg == Leg::LEFT) {
                    uRightFootDestination = localToWorld(-2 * uLRFootOffset, uLeftFootSource);
                } else {
                    uLeftFootDestination = localToWorld(2 * uLRFootOffset, uRightFootSource);
                }
            } else {
                // normal walk, advance steps
                tStep = tStep0;
                if (supportLeg == Leg::LEFT) {
                    uRightFootDestination = stepRightFootDestination(velocityCurrent, uLeftFootSource, uRightFootSource);
                } else {
                    uLeftFootDestination = stepLeftFootDestination(velocityCurrent, uLeftFootSource, uRightFootSource);
                }

                // velocity-based support point modulation
                toeTipCompensation = 0;
                if (velocityDifference[0] > 0) {
                    // accelerating to front
                    supportMod[0] = supportFront2;
                } else if (velocityCurrent[0] > velFastForward) {
                    supportMod[0] = supportFront;
                    toeTipCompensation = ankleMod[0];
                } else if (velocityCurrent[0] < 0) {
                    supportMod[0] = supportBack;
                } else if (std::abs(velocityCurrent[2]) > velFastTurn) {
                    supportMod[0] = supportTurn;
                } else {
                    if (velocityCurrent[1] > 0.015) {
                        supportMod[0] = supportSideX;
                        supportMod[1] = supportSideY;
                    } else if (velocityCurrent[1] < -0.015) {
                        supportMod[0] = supportSideX;
                        supportMod[1] = -supportSideY;
                    }
                }
            }

            uTorsoDestination = stepTorso(uLeftFootDestination, uRightFootDestination, shiftFactor);

            // adjustable initial step body swing
            if (initialStep > 0) {
                supportMod[1] = supportModYInitial;
                if (supportLeg == Leg::RIGHT) {
                    supportMod[1] *= -1;
                }
            }

            // apply velocity-based support point modulation for uSupport
            if (supportLeg == Leg::LEFT) {
                arma::vec3 uLeftFootTorso = worldToLocal(uLeftFootSource, uTorsoSource);
                arma::vec3 uTorsoModded = localToWorld({supportMod[0], supportMod[1], 0}, uTorso);
                arma::vec3 uLeftFootModded = localToWorld(uLeftFootTorso, uTorsoModded);
                uSupport = localToWorld({supportX, supportY, 0}, uLeftFootModded);
                leftLegHardness = hardnessSupport;
                rightLegHardness = hardnessSwing;
            } else {
                arma::vec3 uRightFootTorso = worldToLocal(uRightFootSource, uTorso);
                arma::vec3 uTorsoModded = localToWorld({supportMod[0], supportMod[1], 0}, uTorso);
                arma::vec3 uRightFootModded = localToWorld(uRightFootTorso, uTorsoModded);
                uSupport = localToWorld({supportX, -supportY, 0}, uRightFootModded);
                leftLegHardness = hardnessSwing;
                rightLegHardness = hardnessSupport;
            }

            // compute ZMP coefficients
            zmpParams = {
                (uSupport[0] - uTorso[0]) / (tStep * phase1Zmp),
                (uTorsoDestination[0] - uSupport[0]) / (tStep * (1 - phase2Zmp)),
                (uSupport[1] - uTorso[1]) / (tStep * phase1Zmp),
                (uTorsoDestination[1] - uSupport[1]) / (tStep * (1 - phase2Zmp)),
            };

            zmpCoefficients.rows(0,1) = zmpSolve(uSupport[0], uTorsoSource[0], uTorsoDestination[0], uTorsoSource[0], uTorsoDestination[0]);
            zmpCoefficients.rows(2,3) = zmpSolve(uSupport[1], uTorsoSource[1], uTorsoDestination[1], uTorsoSource[1], uTorsoDestination[1]);
        }

        float xFoot, zFoot;
        std::tie(xFoot, zFoot) = footPhase(phase);
        if (initialStep > 0) {
            zFoot = 0; // don't lift foot at initial step
        }
        pLLeg[2] = 0;
        pRLeg[2] = 0;
        if (supportLeg == Leg::LEFT) {
            uRightFoot = se2Interpolate(xFoot, uRightFootSource, uRightFootDestination);
            pRLeg[2] = stepHeight * zFoot;
        } else {
            uLeftFoot = se2Interpolate(xFoot, uLeftFootSource, uLeftFootDestination);
            pLLeg[2] = stepHeight * zFoot;
        }

        // unused: uTorsoOld = uTorso;

        uTorso = zmpCom(phase, zmpCoefficients, zmpParams, tStep, tZmp, phase1Zmp, phase2Zmp);

        // turning
        float turnCompX = 0;
        if (std::abs(velocityCurrent[2]) > turnCompThreshold && velocityCurrent[0] > -0.01) {
            turnCompX = turnComp;
        }

        // walking front
        float frontCompX = 0;
        if (velocityCurrent[0] > 0.04) {
            frontCompX = frontComp;
        }
        if (velocityDifference[0] > 0.02) {
            frontCompX = frontCompX + accelComp;
        }

        float armPosCompX, armPosCompY;

        // arm movement compensation

        armPosCompX = 0;
        armPosCompY = 0;

        pTorso[3] = 0;
        pTorso[4] = bodyTilt;
        pTorso[5] = 0;
        // NUClear::log("uLeftFoot Motion\n", uLeftFoot);
        // NUClear::log("uTorso Motion\n", uTorso);
        // NUClear::log("uRightFoot Motion\n", uRightFoot);

        arma::vec3 uTorsoActual = localToWorld({-footX + frontCompX + turnCompX + armPosCompX, armPosCompY, 0}, uTorso);
        // NUClear::log("uTorsoActual Motion\n", uTorsoActual);
        pTorso[0] = uTorsoActual[0];
        pTorso[1] = uTorsoActual[1];
        pTorso[5] += uTorsoActual[2];

        pLLeg[0] = uLeftFoot[0];
        pLLeg[1] = uLeftFoot[1];
        pLLeg[5] = uLeftFoot[2];

        pRLeg[0] = uRightFoot[0];
        pRLeg[1] = uRightFoot[1];
        pRLeg[5] = uRightFoot[2];

        std::vector<double> qLegs = darwinop_kinematics_inverse_legs_nubots(pLLeg.memptr(), pRLeg.memptr(), pTorso.memptr());
        auto waypoints = motionLegs(qLegs, sensors);

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }

    std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::updateStill(const Sensors& sensors) {
        leftLegHardness = hardnessSupport;
        rightLegHardness = hardnessSupport;

        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);

        float armPosCompX, armPosCompY;

        armPosCompX = 0;
        armPosCompY = 0;

        pTorso[3] = 0;
        pTorso[4] = bodyTilt;
        pTorso[5] = 0;

        uTorsoActual = localToWorld({-footX + armPosCompX, armPosCompY, 0}, uTorso);

        pTorso[0] = uTorsoActual[0];
        pTorso[1] = uTorsoActual[1];
        pTorso[5] += uTorsoActual[2];

        pLLeg[0] = uLeftFoot[0];
        pLLeg[1] = uLeftFoot[1];
        pLLeg[5] = uLeftFoot[2];

        pRLeg[0] = uRightFoot[0];
        pRLeg[1] = uRightFoot[1];
        pRLeg[5] = uRightFoot[2];

        std::vector<double> qLegs = darwinop_kinematics_inverse_legs_nubots(pLLeg.memptr(), pRLeg.memptr(), pTorso.memptr());

        auto waypoints = motionLegs(qLegs, sensors);

        auto arms = motionArms();
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }

    std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::motionLegs(std::vector<double> qLegs, const Sensors& sensors) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        // balance(qLegs, sensors);

        time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);

        waypoints->push_back({id, time, ServoID::L_HIP_YAW,     float(qLegs[0]),  float(leftLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::L_HIP_ROLL,    float(qLegs[1]),  float(leftLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::L_HIP_PITCH,   float(qLegs[2]),  float(leftLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::L_KNEE,        float(qLegs[3]),  float(leftLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::L_ANKLE_PITCH, float(qLegs[4]),  float(leftLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::L_ANKLE_ROLL,  float(qLegs[5]),  float(leftLegHardness * 100)});

        waypoints->push_back({id, time, ServoID::R_HIP_YAW,     float(qLegs[6]),  float(rightLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::R_HIP_ROLL,    float(qLegs[7]),  float(rightLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::R_HIP_PITCH,   float(qLegs[8]),  float(rightLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::R_KNEE,        float(qLegs[9]),  float(rightLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::R_ANKLE_PITCH, float(qLegs[10]), float(rightLegHardness * 100)});
        waypoints->push_back({id, time, ServoID::R_ANKLE_ROLL,  float(qLegs[11]), float(rightLegHardness * 100)});

        return std::move(waypoints);
    }

    std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::motionArms() {

        arma::vec3 qLArmActual = qLArm;
        arma::vec3 qRArmActual = qRArm;

        qLArmActual.rows(0,1) += armShift;
        qRArmActual.rows(0,1) += armShift;

        // Start arm/leg collision/prevention
        double rotLeftA = modAngle(uLeftFoot[2] - uTorso[2]);
        double rotRightA = modAngle(uTorso[2] - uRightFoot[2]);
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
        waypoints->push_back({id, time, ServoID::R_SHOULDER_PITCH, float(qRArmActual[0]), float(hardnessArm * 100)});
        waypoints->push_back({id, time, ServoID::R_SHOULDER_ROLL,  float(qRArmActual[1]), float(hardnessArm * 100)});
        waypoints->push_back({id, time, ServoID::R_ELBOW,          float(qRArmActual[2]), float(hardnessArm * 100)});
        waypoints->push_back({id, time, ServoID::L_SHOULDER_PITCH, float(qLArmActual[0]), float(hardnessArm * 100)});
        waypoints->push_back({id, time, ServoID::L_SHOULDER_ROLL,  float(qLArmActual[1]), float(hardnessArm * 100)});
        waypoints->push_back({id, time, ServoID::L_ELBOW,          float(qLArmActual[2]), float(hardnessArm * 100)});

        return std::move(waypoints);
    }

    void WalkEngine::balance(std::vector<double>& qLegs, const Sensors& sensors) {
        float gyroRoll0 = 0;
        float gyroPitch0 = 0;

        float phaseComp = std::min({1.0, phaseSingle / 0.1, (1 - phaseSingle) / 0.1});

        /* TODO: crashes
        ServoID supportLegID = (supportLeg == Leg::LEFT) ? ServoID::L_ANKLE_PITCH : ServoID::R_ANKLE_PITCH;
        arma::mat33 ankleRotation = sensors.forwardKinematics.find(supportLegID)->second.submat(0,0,2,2);
        // get effective gyro angle considering body angle offset
        arma::mat33 kinematicGyroSORAMatrix = sensors.orientation * ankleRotation;   //DOUBLE TRANSPOSE
        std::pair<arma::vec3, double> axisAngle = utility::math::matrix::axisAngleFromRotationMatrix(kinematicGyroSORAMatrix);
        arma::vec3 kinematicsGyro = axisAngle.first * (axisAngle.second / balanceWeight);

        gyroRoll0 = -kinematicsGyro[0]*180.0/M_PI;
        gyroPitch0 = -kinematicsGyro[1]*180.0/M_PI;
        */

        float yawAngle = 0;
        if (!active) {
            // double support
            yawAngle = (uLeftFoot[2] + uRightFoot[2]) / 2 - uTorsoActual[2];
        } else if (supportLeg == Leg::LEFT) {
            yawAngle = uLeftFoot[2] - uTorsoActual[2];
        } else if (supportLeg == Leg::RIGHT) {
            yawAngle = uRightFoot[2] - uTorsoActual[2];
        }

        float gyroRoll = gyroRoll0 * std::cos(yawAngle) - gyroPitch0 * std::sin(yawAngle);
        float gyroPitch = gyroPitch0 * std::cos(yawAngle) - gyroRoll0 * std::sin(yawAngle);

        float armShiftX = procFunc(gyroPitch * armImuParamY[1], armImuParamY[2], armImuParamY[3]);
        float armShiftY = procFunc(gyroRoll * armImuParamY[1], armImuParamY[2], armImuParamY[3]);

        float ankleShiftX = procFunc(gyroPitch * ankleImuParamX[1], ankleImuParamX[2], ankleImuParamX[3]);
        float ankleShiftY = procFunc(gyroRoll * ankleImuParamY[1], ankleImuParamY[2], ankleImuParamY[3]);
        float kneeShiftX = procFunc(gyroPitch * kneeImuParamX[1], kneeImuParamX[2], kneeImuParamX[3]);
        float hipShiftY = procFunc(gyroRoll * hipImuParamY[1], hipImuParamY[2], hipImuParamY[3]);

        ankleShift[0] += ankleImuParamX[0] * (ankleShiftX - ankleShift[0]);
        ankleShift[1] += ankleImuParamY[0] * (ankleShiftY - ankleShift[1]);
        kneeShift += kneeImuParamX[0] * (kneeShiftX - kneeShift);
        hipShift[1] += hipImuParamY[0] * (hipShiftY - hipShift[1]);
        armShift[0] += armImuParamX[0] * (armShiftX - armShift[0]);
        armShift[1] += armImuParamY[0] * (armShiftY - armShift[1]);

        // TODO: toe/heel lifting

        emit(graph("kneeShift", kneeShift));
        emit(graph("ankleShift", ankleShift[0], ankleShift[1]));
        emit(graph("hipShift", hipShift[0], hipShift[1]));
        emit(graph("armShift", armShift[0], armShift[1]));
        emit(graph("phaseComp", phaseComp));

        if (!active) {
            // Double support, standing still
            // qLegs[1] += hipShift[1]; // Hip roll stabilization
            qLegs[3] += kneeShift; // Knee pitch stabilization
            qLegs[4] += ankleShift[0]; // Ankle pitch stabilization
            // qLegs[5] += ankleShift[1]; // Ankle roll stabilization

            // qLegs[7] += hipShift[1]; // Hip roll stabilization
            qLegs[9] += kneeShift; // Knee pitch stabilization
            qLegs[10] += ankleShift[0]; // Ankle pitch stabilization
            // qLegs[11] += ankleShift[1]; // Ankle roll stabilization
        } else if (supportLeg == Leg::LEFT) {
            qLegs[1] += hipShift[1]; // Hip roll stabilization
            qLegs[3] += kneeShift; // Knee pitch stabilization
            qLegs[4] += ankleShift[0]; // Ankle pitch stabilization
            qLegs[5] += ankleShift[1]; // Ankle roll stabilization

            qLegs[10] += toeTipCompensation * phaseComp; // Lifting toetip
            qLegs[1] += hipRollCompensation * phaseComp; // Hip roll compensation

        } else {
            qLegs[7] += hipShift[1]; // Hip roll stabilization
            qLegs[9] += kneeShift; // Knee pitch stabilization
            qLegs[10] += ankleShift[0]; // Ankle pitch stabilization
            qLegs[11] += ankleShift[1]; // Ankle roll stabilization

            qLegs[4] += toeTipCompensation * phaseComp; // Lifting toetip
            qLegs[7] -= hipRollCompensation * phaseComp; // Hip roll compensation
        }
    }

    arma::vec3 WalkEngine::stepTorso(arma::vec3 uLeftFoot, arma::vec3 uRightFoot, float shiftFactor) {
        arma::vec3 uLeftFootSupport = localToWorld({supportX, supportY, 0}, uLeftFoot);
        arma::vec3 uRightFootSupport = localToWorld({supportX, -supportY, 0}, uRightFoot);
        return se2Interpolate(shiftFactor, uLeftFootSupport, uRightFootSupport);
    }

    void WalkEngine::setVelocity(double vx, double vy, double va) {
        // filter the commanded speed
        vx = std::min(std::max(vx, velocityLimits(0,0)), velocityLimits(0,1));
        vy = std::min(std::max(vy, velocityLimits(1,0)), velocityLimits(1,1));
        va = std::min(std::max(va, velocityLimits(2,0)), velocityLimits(2,1));

        // slow down when turning
        double vFactor = 1 - std::abs(va) / velocityAngleFactor;

        double stepMag = std::sqrt(vx * vx + vy * vy);
        double magFactor = std::min(velocityLimits(0,1) * vFactor, stepMag) / (stepMag + 0.000001);

        velocityCommand[0] = vx * magFactor;
        velocityCommand[1] = vy * magFactor;
        velocityCommand[2] = va;

        velocityCommand[0] = std::min(std::max(velocityCommand[0], velocityLimits(0,0)), velocityLimits(0,1));
        velocityCommand[1] = std::min(std::max(velocityCommand[1], velocityLimits(1,0)), velocityLimits(1,1));
        velocityCommand[2] = std::min(std::max(velocityCommand[2], velocityLimits(2,0)), velocityLimits(2,1));
    }

    void WalkEngine::updateVelocity() {
        if (velocityCurrent[0] > velocityXHigh) {
            // Slower acceleration at high speed
            velocityDifference[0] = std::min(std::max(velocityCommand[0] - velocityCurrent[0], -velocityDelta[0]), velocityDeltaXHigh);
        } else {
            velocityDifference[0] = std::min(std::max(velocityCommand[0] - velocityCurrent[0], -velocityDelta[0]), velocityDelta[0]);
        }

        velocityDifference[1] = std::min(std::max(velocityCommand[1] - velocityCurrent[1], -velocityDelta[1]), velocityDelta[1]);
        velocityDifference[2] = std::min(std::max(velocityCommand[2] - velocityCurrent[2], -velocityDelta[2]), velocityDelta[2]);

        velocityCurrent[0] += velocityDifference[0];
        velocityCurrent[1] += velocityDifference[1];
        velocityCurrent[2] += velocityDifference[2];

        if (initialStep > 0) {
            velocityCurrent = arma::vec3{0, 0, 0};
            initialStep--;
        }
    }

    arma::vec3 WalkEngine::getVelocity() {
        return velocityCurrent;
    }

    void WalkEngine::stanceReset() {
        // standup/sitdown/falldown handling
        if (startFromStep) {
            uLeftFoot = uLeftFootI;
            uRightFoot = uRightFootI;
            uTorso = uTorsoI;
            if (supportInitial == Leg::RIGHT) {
                // start with left support
                swingLeg = Leg::LEFT;
            } else {
                // start with right support
                swingLeg = Leg::RIGHT;
            }
            // start walking asap
            initialStep = 1;
        } else {
            // stance resetted
            uLeftFoot = localToWorld({-supportX, footY, 0}, uTorso);
            uRightFoot = localToWorld({-supportX, -footY, 0}, uTorso);
            swingLeg = Leg::LEFT;
        }

        uLeftFootSource = uLeftFoot;
        uLeftFootDestination = uLeftFoot;

        uRightFootSource = uRightFoot;
        uRightFootDestination = uRightFoot;

        uSupport = uTorso;
        tLastStep = getTime();
        currentStepType = 0;
        uLRFootOffset = {0, footY, 0};
        startFromStep = false;
    }

    /**
    * Global variables used:
    * tStep, phase1Zmp, phase2Zmp, tZmp
    */
    arma::vec2 WalkEngine::zmpSolve(float zs, float z1, float z2, float x1, float x2) {
        /*
        Solves ZMP equation:
        x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
        where the ZMP point is piecewise linear:
        z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
        */
        float T1 = tStep * phase1Zmp;
        float T2 = tStep * phase2Zmp;
        float m1 = (zs - z1) / T1;
        float m2 = -(zs - z2) / (tStep - T2);

        float c1 = x1 - z1 + tZmp * m1 * std::sinh(-T1 / tZmp);
        float c2 = x2 - z2 + tZmp * m2 * std::sinh((tStep - T2) / tZmp);
        float expTStep = std::exp(tStep / tZmp);
        float aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        float aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }

    /**
    * Global variables used:
    * uSupport, uLeftFootDestination, uLeftFootSource, uRightFootDestination, uRightFootSource
    */
    arma::vec3 WalkEngine::zmpCom(float phase, arma::vec4 zmpCoefficients, arma::vec4 zmpParams, float tStep, float tZmp, float phase1Zmp, float phase2Zmp) {
        arma::vec3 com = {0, 0, 0};
        float expT = std::exp(tStep * phase / tZmp);
        com[0] = uSupport[0] + zmpCoefficients[0] * expT + zmpCoefficients[1] / expT;
        com[1] = uSupport[1] + zmpCoefficients[2] * expT + zmpCoefficients[3] / expT;
        if (phase < phase1Zmp) {
            com[0] = com[0] + zmpParams[0] * tStep * (phase - phase1Zmp) -tZmp * zmpParams[0] * std::sinh(tStep * (phase - phase1Zmp) / tZmp);
            com[1] = com[1] + zmpParams[1] * tStep * (phase - phase1Zmp) -tZmp * zmpParams[1] * std::sinh(tStep * (phase - phase1Zmp) / tZmp);
        } else if (phase > phase2Zmp) {
            com[0] = com[0] + zmpParams[2] * tStep * (phase - phase2Zmp) -tZmp * zmpParams[2] * std::sinh(tStep * (phase - phase2Zmp) / tZmp);
            com[1] = com[1] + zmpParams[3] * tStep * (phase - phase2Zmp) -tZmp * zmpParams[3] * std::sinh(tStep * (phase - phase2Zmp) / tZmp);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com[2] = phase * (uLeftFootDestination[2] + uRightFootDestination[2]) / 2 + (1 - phase) * (uLeftFootSource[2] + uRightFootSource[2]) / 2;
        return com;
    }

    /**
     * Globals: uLRFootOffset, footSizeX, stanceLimitY, stanceLimitY2, stanceLimitAngle
     */
    arma::vec3 WalkEngine::stepLeftFootDestination(arma::vec3 velocity, arma::vec3 uLeftFoot, arma::vec3 uRightFoot) {
        // Get midpoint between the two feet
        arma::vec3 midPoint = se2Interpolate(0.5, uLeftFoot, uRightFoot);
        // Get midpoint 1.5 steps in future
        arma::vec3 forwardPoint = localToWorld(1.5 * velocity, midPoint);
        // Offset to towards the foot in use to get the target location
        arma::vec3 leftFootTarget = localToWorld(uLRFootOffset, forwardPoint);

        // Start foot collision detection/prevention
        arma::vec3 uLeftFootRight = worldToLocal(leftFootTarget, uRightFoot);
        // Do not pidgeon toe, cross feet:
        // Check toe and heel overlap
        double toeOverlap = -footSizeX[0] * uLeftFootRight[2];
        double heelOverlap = -footSizeX[1] * uLeftFootRight[2];
        double limitY = std::max(stanceLimits(1,0), stanceLimitY2 + std::max(toeOverlap, heelOverlap));
        uLeftFootRight[0] = std::min(std::max(uLeftFootRight[0], stanceLimits(0,0)), stanceLimits(0,1));
        uLeftFootRight[1] = std::min(std::max(uLeftFootRight[1], limitY), stanceLimits(1,1));
        uLeftFootRight[2] = std::min(std::max(uLeftFootRight[2], stanceLimits(2,0)), stanceLimits(2,1));
        leftFootTarget = localToWorld(uLeftFootRight, uRightFoot);
        // End foot collision detection/prevention

        return leftFootTarget;
    }

    arma::vec3 WalkEngine::stepRightFootDestination(arma::vec3 velocity, arma::vec3 uLeftFoot, arma::vec3 uRightFoot) {
        // Get midpoint between the two feet
        arma::vec3 midPoint = se2Interpolate(0.5, uLeftFoot, uRightFoot);
        // Get midpoint 1.5 steps in future
        arma::vec3 forwardPoint = localToWorld(1.5 * velocity, midPoint);
        // Offset to towards the foot in use to get the target location
        arma::vec3 rightFootTarget = localToWorld(-uLRFootOffset, forwardPoint);

        // Start foot collision detection/prevention
        arma::vec3 uRightFootLeft = worldToLocal(rightFootTarget, uLeftFoot);
        // Do not pidgeon toe, cross feet:
        // Check toe and heel overlap
        double toeOverlap = footSizeX[0] * uRightFootLeft[2];
        double heelOverlap = footSizeX[1] * uRightFootLeft[2];
        double limitY = std::max(stanceLimits(1,0), stanceLimitY2 + std::max(toeOverlap, heelOverlap));
        uRightFootLeft[0] = std::min(std::max(uRightFootLeft[0], stanceLimits(0,0)), stanceLimits(0,1));
        uRightFootLeft[1] = std::min(std::max(uRightFootLeft[1], -stanceLimits(1,1)), -limitY);
        uRightFootLeft[2] = std::min(std::max(uRightFootLeft[2], -stanceLimits(2,1)), -stanceLimits(2,0));
        rightFootTarget = localToWorld(uRightFootLeft, uLeftFoot);
        // End foot collision detection/prevention

        return rightFootTarget;
    }

    /**
    * Global variables used:
    * phase1Single, phase2Single
    */
    std::pair<float, float> WalkEngine::footPhase(float phase) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        phaseSingle = std::min(std::max(phase - phase1Single, 0.0f) / (phase2Single - phase1Single), 1.0f);
        float phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        float xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        float zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return std::make_pair(xf, zf);
    }

    double WalkEngine::getTime() {
          struct timeval t;
          gettimeofday(&t, NULL);
          return t.tv_sec + 1E-6 * t.tv_usec;
    }

    double WalkEngine::procFunc(double value, double deadband, double maxvalue) { //a function for IMU feedback (originally from teamdarwin2013release/player/util/util.lua)
        // clamp between 0 and maxvalue
        // offset using deadband
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }

    double WalkEngine::modAngle(double value) { // reduce an angle to [-pi, pi)
        double angle = std::fmod(value, 2 * M_PI);
        if (angle <= -M_PI) angle += 2 * M_PI;
        else if (angle > M_PI) angle -= 2 * M_PI;

        return angle;
    }

    arma::vec3 WalkEngine::localToWorld(arma::vec3 poseRelative, arma::vec3 pose) { //TEAMDARWIN LUA VECs START INDEXING @ 1 not 0 !!
        double ca = std::cos(pose[2]);
        double sa = std::sin(pose[2]);
        // translates to pose + rotZ(pose.angle) * poseRelative
        return {
            pose[0] + ca * poseRelative[0] - sa * poseRelative[1],
            pose[1] + sa * poseRelative[0] + ca * poseRelative[1],
            pose[2] + poseRelative[2] // do not use modAngle here, causes bad things when turning!
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
            modAngle(pa)
        };
    }

    arma::vec3 WalkEngine::se2Interpolate(double t, arma::vec3 u1, arma::vec3 u2) {
        // helps smooth out the motions using a weighted average
        return {
            u1[0] + t * (u2[0] - u1[0]),
            u1[1] + t * (u2[1] - u1[1]),
            u1[2] + t * modAngle(u2[2] - u1[2])
        };
    }

}  // motion
}  // modules

