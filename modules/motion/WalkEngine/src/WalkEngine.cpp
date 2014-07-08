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
#include "utility/nubugger/NUgraph.h"
#include "utility/support/armayamlconversions.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/ServoTarget.h"
#include "messages/behaviour/Action.h"
#include "messages/motion/Script.h"



namespace modules {
    namespace motion {

        using messages::input::ServoID;
        using messages::behaviour::ServoCommand;
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
                        stanceReset();
                        updateHandle.enable();
                    }
                },
                [this] (const std::set<LimbID>& takenLimbs) {
                    if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                        // legs are no longer available, reset walking (too late to stop walking)
                        updateHandle.disable();
                    }
                },
                [this] (const std::set<ServoID>&) {
                    // nothing
                }
            }));

            updateHandle = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds> > >, With<Sensors>, Options<Single, Priority<NUClear::HIGH>> >([this](const time_t&, const Sensors& sensors) {
                emit(update(sensors));
            });

            updateHandle.disable();

            on<Trigger<WalkCommand>>([this](const WalkCommand& walkCommand) {
                setVelocity(walkCommand.velocity[0] * (walkCommand.velocity[0] > 0 ? velLimitX[1] : -velLimitX[0]),
                            walkCommand.velocity[1] * (walkCommand.velocity[1] > 0 ? velLimitY[1] : -velLimitY[0]),
                            walkCommand.rotationalSpeed * (walkCommand.rotationalSpeed > 0 ? velLimitA[1] : -velLimitA[0]));
            });

            on<Trigger<WalkStartCommand>>([this](const WalkStartCommand&) {
                start();
                emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 25, 10 }})); // TODO: config
            });

            on<Trigger<WalkStopCommand>>([this](const WalkStopCommand&) {
                stop();
                // TODO: set priorities to 0 when stopped - somehow
            });

            on<Trigger<Configuration<WalkEngine> > >([this](const Configuration<WalkEngine>& config) {

                // g Walk Parameters
                // g Stance and velocity limit values
                stanceLimitX = config["stanceLimitX"].as<arma::vec>();
                stanceLimitY = config["stanceLimitY"].as<arma::vec>();
                stanceLimitA = config["stanceLimitA"].as<arma::vec>();
                velLimitX = config["velLimitX"].as<arma::vec>();
                velLimitY = config["velLimitY"].as<arma::vec>();
                velLimitA = config["velLimitA"].as<arma::vec>();
                velDelta = config["velDelta"].as<arma::vec>();
                vaFactor = config["vaFactor"].as<float>();

                velXHigh = config["velXHigh"].as<double>();
                velDeltaXHigh = config["velDeltaXHigh"].as<double>();

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
                qLArm0 = M_PI / 180.0 * arma::vec3{90, 2, -20};
                qRArm0 = M_PI / 180.0 * arma::vec3{90, -2, -20};

                // gHardness parameters
                hardnessSupport = config["hardnessSupport"].as<float>();

                hardnessSwing = config["hardnessSwing"].as<float>();

                hardnessArm0 = config["hardnessArm0"].as<float>();
                hardnessArm = config["hardnessArm"].as<float>();

                // gGait parameters

                tStep = config["tStep"].as<float>();
                tStep0 = tStep;
                tZmp = config["tZmp"].as<float>();
                stepHeight = config["stepHeight"].as<float>();
                stepHeight0 = stepHeight;
                ph1Single = config["phSingle"][0].as<float>();
                ph2Single = config["phSingle"][1].as<float>();
                ph1Zmp = ph1Single;
                ph2Zmp = ph2Single;

                // gCompensation parameters
                //TODO:config hiprollcomp?
                hipRollCompensation = 4 * M_PI / 180;
                ankleMod = arma::vec2{-config["toeTipCompensation"].as<double>(), 0} * 1 * M_PI / 180;
                spreadComp = config["spreadComp"].as<float>();
                turnCompThreshold = config["turnCompThreshold"].as<float>();
                turnComp = config["turnComp"].as<float>();

                float gyroFactor = config["gyroFactor"].as<float>() * 0.273 * M_PI / 180 * 300 / 1024; //dps to rad/s conversion

                // gGyro stabilization parameters
                //TODO:config this?
                ankleImuParamX = {0.5, 0.3 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                ankleImuParamY = {0.5, 1.2 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                kneeImuParamX = {0.5, 0.7 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                hipImuParamY = {0.5, 0.3 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                armImuParamX = {0.5, 10.0 * gyroFactor, 20 * M_PI / 180, 45 * M_PI / 180};
                armImuParamY = {0.5, 0.0 * gyroFactor, 20 * M_PI / 180, 45 * M_PI / 180};

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
                AccelComp = config["AccelComp"].as<float>();

                balanceWeight = config["balanceWeight"].as<float>();

                // gInitial body swing
                supportModYInitial = config["supportModYInitial"].as<float>();

                //XXX: this isn't a real config variable - it derives from akleMod[0]
                toeTipCompensation = config["toeTipCompensation"].as<float>();

                useAlternativeTrajectory = config["useAlternativeTrajectory"].as<bool>();

//                setVelocity(config["velCommandX"], config["velCommandY"], config["velCommandAngular"]);
                //Generate stand script
                reset();
                stanceReset();
                auto waypoints = updateStill();

                Script standScript;
                Script::Frame frame;
                frame.duration = std::chrono::milliseconds(config["STAND_SCRIPT_DURATION_MILLISECONDS"].as<int>());
                for(auto& waypoint : *waypoints){
                    frame.targets.push_back(Script::Frame::Target({waypoint.id,
                                                                   waypoint.position,
                                                                   waypoint.gain}
                                                                  )
                                           );
                }
                standScript.frames.push_back(frame);
                auto saveScript = std::make_unique<SaveConfiguration>();
                saveScript->path = "config/scripts/Stand.yaml";
                saveScript->config = standScript;
                emit(std::move(saveScript));
            });

            on<Trigger<Startup>>([this](const Startup&) {
                stopRequest = 2;
                reset();
                //start();
            });

        }
        // TODO: add others

        void WalkEngine::reset(){
            // g--------------------------------------------------------
                // g Walk state variables
                // g--------------------------------------------------------

                uTorso = {supportX, 0, 0};
                uLeft = {0, footY, 0};
                uRight = {0, -footY, 0};

                pLLeg = {0, footY, 0, 0, 0, 0};
                pRLeg = {0, -footY, 0, 0, 0, 0};
                pTorso = {supportX, 0, bodyHeight, 0, bodyTilt, 0};

                velCurrent = {0, 0, 0};
                velCommand = {0, 0, 0};
                velDiff = {0, 0, 0};

                // gZMP exponential coefficients:
                aXP = 0;
                aXN = 0;
                aYP = 0;
                aYN = 0;

                // gGyro stabilization variables
                ankleShift = {0, 0};
                kneeShift = 0;
                hipShift = {0, 0};
                armShift = {0, 0};

                active = true;
                started = false;
                iStep0 = -1;
                iStep = 0;
                t0 = getTime();
                tLastStep = getTime();
                ph0=0;
                ph=0;

                currentStepType = 0;

                initialStep = 2;

                qLArmOR0 = qLArm0;
                qRArmOR0 = qRArm0;
                bodyRot0 = {0, bodyTilt, 0};

                qLArmOR = qLArm0;
                qRArmOR = qRArm0;
                bodyRot = {0, bodyTilt, 0};

                qLArmOR1 = {0, 0, 0};
                qRArmOR1 = {0, 0, 0};
                bodyRot1 = {0, 0, 0};

                phSingle = 0;

                // gCurrent arm pose
                qLArm = M_PI / 180 * arma::vec3{90, 40, -160};
                qRArm = M_PI / 180 * arma::vec3{90, -40, -160};

                // gqLArm0={qLArm[1],qLArm[2]};
                // gqRArm0={qRArm[1],qRArm[2]};

                // gStandard offset
                uLRFootOffset = {0, footY + supportY, 0};

                // gWalking/Stepping transition variables
                uLeftI = {0, 0, 0};
                uRightI = {0, 0, 0};
                uTorsoI = {0, 0, 0};
                supportI = LEFT;
                startFromStep = false;

                comdot = {0, 0};

                stanceReset();
        }

        void WalkEngine::start() {
            stopRequest = 0;
            if (!active) {
                double now = getTime();

                active = true;
                started = false;
                iStep0 = -1;
                t0 = now;
                tLastStep = now;
                initialStep = 2;
            }
        }

        void WalkEngine::stop() {
            // always stops with feet together (which helps transition)
            stopRequest = std::max(1, stopRequest);
        }

        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::update(const Sensors& sensors) {
            //advanceMotion();
            double time = getTime();


            // TODO: bodyHeightCurrent = vcm.get_camera_bodyHeight();

//            log<DEBUG>("velCurrent: ", velCurrent);
//            log<DEBUG>("velCommand: ", velCommand);
            if (!active) {
                moving = false;
                return updateStill(sensors);
            }

            if (!started) {
                started = true;
                tLastStep = time;
            }

            ph0 = ph;
            moving = true;

            ph = (time - tLastStep) / tStep;

            if (ph > 1) {
                iStep++;
                ph = ph - std::floor(ph);
                tLastStep += tStep;
            }

            if (iStep > iStep0 && stopRequest == 2) {
                stopRequest = 0;
                active = false;
                emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 0, 0 }})); // TODO: config
                emit(std::make_unique<WalkStopped>());

                return std::make_unique<std::vector<ServoCommand>>(); // TODO: return "stop"
            }

            // new step
            if (iStep > iStep0) {
                updateVelocity();
                iStep0 = iStep;
                supportLeg = (iStep % 2 == 0 ? LEFT : RIGHT); // 0 for left support, 1 for right support
                uLeft1 = uLeft2;
                uRight1 = uRight2;
                uTorso1 = uTorso2;

                supportMod = {0, 0}; // support point modulation for wallkick
                shiftFactor = 0.5; // how much should we shift final torso pose?

                if (stopRequest == 1) {
                    log<DEBUG>("stop request 1");
                    stopRequest = 2;
                    velCurrent = {0, 0, 0};
                    velCommand = {0, 0, 0};
                    if (supportLeg == LEFT) {
                        uRight2 = poseGlobal(-2 * uLRFootOffset, uLeft1);
                    } else {
                        uLeft2 = poseGlobal(2 * uLRFootOffset, uRight1);
                    }
                } else {
                    // normal walk, advance steps
                    tStep = tStep0;
                    if (supportLeg == LEFT) {
                        uRight2 = stepRightDestination(velCurrent, uLeft1, uRight1);
                    } else {
                        uLeft2 = stepLeftDestination(velCurrent, uLeft1, uRight1);
                    }

                    // velocity-based support point modulation
                    toeTipCompensation = 0;
                    if (velDiff[0] > 0) {
                        // accelerating to front
                        supportMod[0] = supportFront2;
                    } else if (velCurrent[0] > velFastForward) {
                        supportMod[0] = supportFront;
                        toeTipCompensation = ankleMod[0];
                    } else if (velCurrent[0] < 0) {
                        supportMod[0] = supportBack;
                    } else if (std::abs(velCurrent[2]) > velFastTurn) {
                        supportMod[0] = supportTurn;
                    } else {
                        if (velCurrent[1] > 0.015) {
                            supportMod[0] = supportSideX;
                            supportMod[1] = supportSideY;
                        } else if (velCurrent[1] < -0.015) {
                            supportMod[0] = supportSideX;
                            supportMod[1] = -supportSideY;
                        }
                    }
                }

                uTorso2 = stepTorso(uLeft2, uRight2, shiftFactor);

                // adjustable initial step body swing
                if (initialStep > 0) {
                    supportMod[1] = supportModYInitial;
                    if (supportLeg == RIGHT) {
                        supportMod[1] *= -1;
                    }
                }

                // apply velocity-based support point modulation for uSupport
                if (supportLeg == LEFT) {
                    arma::vec3 uLeftTorso = poseRelative(uLeft1, uTorso1);
                    arma::vec3 uTorsoModded = poseGlobal({supportMod[0], supportMod[1], 0}, uTorso);
                    arma::vec3 uLeftModded = poseGlobal(uLeftTorso, uTorsoModded);
                    uSupport = poseGlobal({supportX, supportY, 0}, uLeftModded);
                    leftLegHardness = hardnessSupport;
                    rightLegHardness = hardnessSwing;
                } else {
                    arma::vec3 uRightTorso = poseRelative(uRight1, uTorso);
                    arma::vec3 uTorsoModded = poseGlobal({supportMod[0], supportMod[1], 0}, uTorso);
                    arma::vec3 uRightModded = poseGlobal(uRightTorso, uTorsoModded);
                    uSupport = poseGlobal({supportX, -supportY, 0}, uRightModded);
                    leftLegHardness = hardnessSwing;
                    rightLegHardness = hardnessSupport;
                }

                // compute ZMP coefficients
                m1X = (uSupport[0] - uTorso[0]) / (tStep * ph1Zmp);
                m2X = (uTorso2[0] - uSupport[0]) / (tStep * (1 - ph2Zmp));
                m1Y = (uSupport[1] - uTorso[1]) / (tStep * ph1Zmp);
                m2Y = (uTorso2[1] - uSupport[1]) / (tStep * (1 - ph2Zmp));
                std::tie(aXP, aXN) = zmpSolve(uSupport[0], uTorso1[0], uTorso2[0], uTorso1[0], uTorso2[0]);
                std::tie(aYP, aYN) = zmpSolve(uSupport[1], uTorso1[1], uTorso2[1], uTorso1[1], uTorso2[1]);

                // compute COM speed at the boundary

//                dx0 = (aXP - aXN) / tZmp + m1X * (1 - std::cosh(ph1Zmp * tStep / tZmp));
//                dy0 = (aYP - aYN) / tZmp + m1Y * (1 - std::cosh(ph1Zmp * tStep / tZmp));

                float dx1 = (aXP * std::exp(tStep / tZmp) - aXN * std::exp(-tStep / tZmp)) / tZmp
                        + m2X * (1 - std::cosh((1 - ph2Zmp) * tStep / tZmp));
                float dy1 = (aYP * std::exp(tStep / tZmp) - aYN * std::exp(-tStep / tZmp)) / tZmp
                        + m2Y * (1 - std::cosh((1 - ph2Zmp) * tStep / tZmp));

                comdot = {dx1, dy1};
            }

            float xFoot, zFoot;
            std::tie(xFoot, zFoot) = footPhase(ph);
            if (initialStep > 0) {
                zFoot = 0; // don't lift foot at initial step
            }
            pLLeg[2] = 0;
            pRLeg[2] = 0;
            if (supportLeg == LEFT) {
                uRight = se2Interpolate(xFoot, uRight1, uRight2);
                pRLeg[2] = stepHeight * zFoot;
            } else {
                uLeft = se2Interpolate(xFoot, uLeft1, uLeft2);
                pLLeg[2] = stepHeight * zFoot;
            }

            // unused: uTorsoOld = uTorso;

            uTorso = zmpCom(ph);

            // turning
            float turnCompX = 0;
            if (std::abs(velCurrent[2]) > turnCompThreshold && velCurrent[0] > -0.01) {
                turnCompX = turnComp;
            }

            // walking front
            float frontCompX = 0;
            if (velCurrent[0] > 0.04) {
                frontCompX = frontComp;
            }
            if (velDiff[0] > 0.02) {
                frontCompX = frontCompX + AccelComp;
            }

            float armPosCompX, armPosCompY;

            // arm movement compensation

            armPosCompX = 0;
            armPosCompY = 0;

            pTorso[3] = 0;
            pTorso[4] = bodyTilt;
            pTorso[5] = 0;
            // NUClear::log("uLeft Motion\n", uLeft);
            // NUClear::log("uTorso Motion\n", uTorso);
            // NUClear::log("uRight Motion\n", uRight);

            arma::vec3 uTorsoActual = poseGlobal({-footX + frontCompX + turnCompX + armPosCompX, armPosCompY, 0}, uTorso);
            // NUClear::log("uTorsoActual Motion\n", uTorsoActual);
            pTorso[0] = uTorsoActual[0];
            pTorso[1] = uTorsoActual[1];
            pTorso[5] += uTorsoActual[2];

            pLLeg[0] = uLeft[0];
            pLLeg[1] = uLeft[1];
            pLLeg[5] = uLeft[2];

            pRLeg[0] = uRight[0];
            pRLeg[1] = uRight[1];
            pRLeg[5] = uRight[2];

            std::vector<double> qLegs = darwinop_kinematics_inverse_legs_nubots(pLLeg.memptr(), pRLeg.memptr(), pTorso.memptr(), supportLeg);
            auto waypoints = motionLegs(qLegs, true, sensors);
            // auto arms = motionArms();
            // waypoints.insert(waypoints->end(), arms->begin(), arms->end());

            return waypoints;
        }

        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::updateStill(const Sensors& sensors) {
            leftLegHardness = hardnessSupport;
            rightLegHardness = hardnessSupport;

            uTorso = stepTorso(uLeft, uRight, 0.5);

            float armPosCompX, armPosCompY;

            armPosCompX = 0;
            armPosCompY = 0;

            pTorso[3] = 0;
            pTorso[4] = bodyTilt;
            pTorso[5] = 0;


            // NUClear::log("uLeft Still\n", uLeft);
            // NUClear::log("uRight Still\n", uRight);
            // NUClear::log("uTorso Still\n", uTorso);
            uTorsoActual = poseGlobal({-footX + armPosCompX, armPosCompY, 0}, uTorso);
            // NUClear::log("uTorsoActual Still\n", uTorsoActual);
            pTorso[0] = uTorsoActual[0];
            pTorso[1] = uTorsoActual[1];
            pTorso[5] += uTorsoActual[2];

            pLLeg[0] = uLeft[0];
            pLLeg[1] = uLeft[1];
            pLLeg[5] = uLeft[2];

            pRLeg[0] = uRight[0];
            pRLeg[1] = uRight[1];
            pRLeg[5] = uRight[2];

            std::vector<double> qLegs = darwinop_kinematics_inverse_legs_nubots(pLLeg.memptr(), pRLeg.memptr(), pTorso.memptr(), supportLeg);

            auto waypoints = motionLegs(qLegs, true, sensors);
            // auto arms = motionArms();
            // waypoints.insert(waypoints->end(), arms->begin(), arms->end());

            return waypoints;
        }

        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> WalkEngine::motionLegs(std::vector<double> qLegs, bool gyroOff, const Sensors& sensors) {
            float gyroRoll0 = 0;
            float gyroPitch0 = 0;

            float phComp = std::min({1.0, phSingle / 0.1, (1 - phSingle) / 0.1});
            if (!gyroOff) {
                ServoID supportLegID = (supportLeg == LEFT) ? ServoID::L_ANKLE_PITCH : ServoID::R_ANKLE_PITCH;
                arma::mat33 ankleRotation = sensors.forwardKinematics.find(supportLegID)->second.submat(0,0,2,2);
                // get effective gyro angle considering body angle offset
                arma::mat33 kinematicGyroSORAMatrix = sensors.orientation * ankleRotation;   //DOUBLE TRANSPOSE
                std::pair<arma::vec3, double> axisAngle = utility::math::matrix::axisAngleFromRotationMatrix(kinematicGyroSORAMatrix);
                arma::vec3 kinematicsGyro = axisAngle.first * (axisAngle.second / balanceWeight);

                gyroRoll0 = -kinematicsGyro[0]*180.0/M_PI;
                gyroPitch0 = -kinematicsGyro[1]*180.0/M_PI;
            }

            float yawAngle = 0;
            if (!active) {
                // double support
                yawAngle = (uLeft[2] + uRight[2]) / 2 - uTorsoActual[2];
            } else if (supportLeg == LEFT) {
                yawAngle = uLeft[2] - uTorsoActual[2];
            } else if (supportLeg == RIGHT) {
                yawAngle = uRight[2] - uTorsoActual[2];
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




            } else if (supportLeg == LEFT) {
                qLegs[1] += hipShift[1]; // Hip roll stabilization
                qLegs[3] += kneeShift; // Knee pitch stabilization
                qLegs[4] += ankleShift[0]; // Ankle pitch stabilization
                qLegs[5] += ankleShift[1]; // Ankle roll stabilization

                qLegs[10] += toeTipCompensation * phComp; // Lifting toetip
                qLegs[1] += hipRollCompensation * phComp; // Hip roll compensation

            } else {
                qLegs[7] += hipShift[1]; // Hip roll stabilization
                qLegs[9] += kneeShift; // Knee pitch stabilization
                qLegs[10] += ankleShift[0]; // Ankle pitch stabilization
                qLegs[11] += ankleShift[1]; // Ankle roll stabilization

                qLegs[4] += toeTipCompensation * phComp; // Lifting toetip
                qLegs[7] -= hipRollCompensation * phComp; // Hip roll compensation

            }

            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(16);

/*
            0 = lefthipyaw // Hip pitch or yaw YAW
            1 = lefthiproll
            2 = lefthippitch // Hip pitch or yaw
            3 = leftknee
            4 = leftanklepitch
            5 = leftankleroll
            6 = righthipyaw
            7 = rightHipRoll
            8 = righthipitch
            9 = rightKnee
            10 = rightAnklePitch
            11 = rightAnkleRoll*/

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

            arma::vec3 qLArmActual = {qLArm0[0] + armShift[0], qLArm0[1] + armShift[1], 0};
            arma::vec3 qRArmActual = {qRArm0[0] + armShift[0], qRArm0[1] + armShift[1], 0};

            // check leg hitting
            float rotLeftA = modAngle(uLeft[2] - uTorso[2]);
            float rotRightA = modAngle(uTorso[2] - uRight[2]);

            arma::vec3 leftLegTorso = poseRelative(uLeft, uTorso);
            arma::vec3 rightLegTorso = poseRelative(uRight, uTorso);

            qLArmActual[1] = std::max(
                    5 * M_PI / 180 + std::max(0.0f, rotLeftA) / 2
                    + std::max(0.0, leftLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180)
                    , qLArmActual[1]);

            qRArmActual[1] = std::max(
                    -5 * M_PI / 180 + std::max(0.0f, rotRightA) / 2
                    - std::max(0.0, rightLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180)
                    , qLArmActual[1]);


            qLArmActual[2] = qLArm[2];
            qRArmActual[2] = qRArm[2];


            auto waypoints = std::make_unique<std::vector<ServoCommand>>();
            waypoints->reserve(6);
            time_t time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);

            waypoints->push_back({id, time, ServoID::R_SHOULDER_PITCH, float(qRArmActual[0]),  float(hardnessArm * 100)});
            waypoints->push_back({id, time, ServoID::R_SHOULDER_ROLL,  float(qRArmActual[1]),  float(hardnessArm * 100)});
            waypoints->push_back({id, time, ServoID::R_ELBOW,          float(qRArmActual[2]),  float(hardnessArm * 100)});
            waypoints->push_back({id, time, ServoID::L_SHOULDER_PITCH, float(qLArmActual[0]),  float(hardnessArm * 100)});
            waypoints->push_back({id, time, ServoID::L_SHOULDER_ROLL,  float(qLArmActual[1]),  float(hardnessArm * 100)});
            waypoints->push_back({id, time, ServoID::L_ELBOW,          float(qLArmActual[2]),  float(hardnessArm * 100)});

            /*emit(graph("L Shoulder Pitch", qLArmActual[0]));
            emit(graph("L Shoulder Roll", qLArmActual[1]));
            emit(graph("L Elbow", qLArmActual[2]));

            emit(graph("R Shoulder Pitch", qRArmActual[0]));
            emit(graph("R Shoulder Roll", qRArmActual[1]));
            emit(graph("R Elbow", qRArmActual[2]));*/

            return std::move(waypoints);
        }

        void WalkEngine::exit() {
            // TODO: empty?
        }

        arma::vec3 WalkEngine::stepLeftDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight) {
            arma::vec3 u0 = se2Interpolate(0.5, uLeft, uRight);
            // Determine nominal midpoint position 1.5 steps in future
            arma::vec3 u1 = poseGlobal(vel, u0);
            arma::vec3 u2 = poseGlobal(0.5 * vel, u1);
            arma::vec3 uLeftPredict = poseGlobal(uLRFootOffset, u2);
            arma::vec3 uLeftRight = poseRelative(uLeftPredict, uRight);
            // Do not pidgeon toe, cross feet:

            // Check toe and heel overlap
            double toeOverlap = -footSizeX[0] * uLeftRight[2];
            double heelOverlap = -footSizeX[1] * uLeftRight[2];
            double limitY = std::max(stanceLimitY[0], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

            // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

            uLeftRight[0] = std::min(std::max(uLeftRight[0], stanceLimitX[0]), stanceLimitX[1]);
            uLeftRight[1] = std::min(std::max(uLeftRight[1], limitY), stanceLimitY[1]);
            uLeftRight[2] = std::min(std::max(uLeftRight[2], stanceLimitA[0]), stanceLimitA[1]);

            return poseGlobal(uLeftRight, uRight);
        }

        arma::vec3 WalkEngine::stepRightDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight) {
            arma::vec3 u0 = se2Interpolate(.5, uLeft, uRight);
            // Determine nominal midpoint position 1.5 steps in future
            arma::vec3 u1 = poseGlobal(vel, u0);
            arma::vec3 u2 = poseGlobal(0.5 * vel, u1);
            arma::vec3 uRightPredict = poseGlobal(-1 * uLRFootOffset, u2);
            arma::vec3 uRightLeft = poseRelative(uRightPredict, uLeft);
            // Do not pidgeon toe, cross feet:

            // Check toe and heel overlap
            double toeOverlap = footSizeX[0] * uRightLeft[2];
            double heelOverlap = footSizeX[1] * uRightLeft[2];
            double limitY = std::max(stanceLimitY[0], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

            // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

            uRightLeft[0] = std::min(std::max(uRightLeft[0], stanceLimitX[0]), stanceLimitX[1]);
            uRightLeft[1] = std::min(std::max(uRightLeft[1], -stanceLimitY[1]), -limitY);
            uRightLeft[2] = std::min(std::max(uRightLeft[2], -stanceLimitA[1]), -stanceLimitA[0]);

            return poseGlobal(uRightLeft, uLeft);
        }

        arma::vec3 WalkEngine::stepTorso(arma::vec3 uLeft, arma::vec3 uRight, float shiftFactor) {
            arma::vec3 u0 = se2Interpolate(0.5, uLeft, uRight);
            arma::vec3 uLeftSupport = poseGlobal({supportX, supportY, 0}, uLeft);
            arma::vec3 uRightSupport = poseGlobal({supportX, -supportY, 0}, uRight);
            return se2Interpolate(shiftFactor, uLeftSupport, uRightSupport);
        }

        void WalkEngine::setVelocity(double vx, double vy, double va) {
            // filter the commanded speed
            vx = std::min(std::max(vx, velLimitX[0]), velLimitX[1]);
            vy = std::min(std::max(vy, velLimitY[0]), velLimitY[1]);
            va = std::min(std::max(va, velLimitA[0]), velLimitA[1]);

            // slow down when turning
            double vFactor = 1 - std::abs(va) / vaFactor;

            double stepMag = std::sqrt(vx * vx + vy * vy);
            double magFactor = std::min(velLimitX[1] * vFactor, stepMag) / (stepMag + 0.000001);

            velCommand[0] = vx * magFactor;
            velCommand[1] = vy * magFactor;
            velCommand[2] = va;

            velCommand[0] = std::min(std::max(velCommand[0], velLimitX[0]), velLimitX[1]);
            velCommand[1] = std::min(std::max(velCommand[1], velLimitY[0]), velLimitY[1]);
            velCommand[2] = std::min(std::max(velCommand[2], velLimitA[0]), velLimitA[1]);
        }

        void WalkEngine::updateVelocity() {
            if (velCurrent[0] > velXHigh) {
                // Slower acceleration at high speed
                velDiff[0] = std::min(std::max(velCommand[0] - velCurrent[0],
                        -velDelta[0]), velDeltaXHigh);
            } else {
                velDiff[0] = std::min(std::max(velCommand[0] - velCurrent[0],
                        -velDelta[0]), velDelta[0]);
            }

            velDiff[1] = std::min(std::max(velCommand[1] - velCurrent[1],
                    -velDelta[1]), velDelta[1]);
            velDiff[2] = std::min(std::max(velCommand[2] - velCurrent[2],
                    -velDelta[2]), velDelta[2]);

            velCurrent[0] += velDiff[0];
            velCurrent[1] += velDiff[1];
            velCurrent[2] += velDiff[2];

            if (initialStep > 0) {
                velCurrent = arma::vec3{0, 0, 0};
                initialStep--;
            }
        }

        arma::vec3 WalkEngine::getVelocity() {
            return velCurrent;
        }

        void WalkEngine::setInitialStance(arma::vec3 uL, arma::vec3 uR, arma::vec3 uT, Leg support) {
            uLeftI = uL;
            uRightI = uR;
            uTorso = uT;
            supportI = support;
            startFromStep = true;
        }

        void WalkEngine::stanceReset() {
            // standup/sitdown/falldown handling
            if (startFromStep) {
                uLeft = uLeftI;
                uRight = uRightI;
                uTorso = uTorsoI;
                if (supportI == 0) {
                    // start with left support
                    iStep0 = -1;
                    iStep = 0;
                } else {
                    // start with right support
                    iStep0 = 0;
                    iStep = 1;
                }
                // start walking asap
                initialStep = 1;
            } else {
                // stance resetted
                uLeft = poseGlobal({-supportX, footY, 0}, uTorso);
                uRight = poseGlobal({-supportX, -footY, 0}, uTorso);
                iStep0 = -1;
                iStep = 0;
            }

            uLeft1 = uLeft;
            uLeft2 = uLeft;

            uRight1 = uRight;
            uRight2 = uRight;

            uSupport = uTorso;
            tLastStep = getTime();
            currentStepType = 0;
            uLRFootOffset = {0, footY, 0};
            startFromStep = false;
        }

        std::pair<float, float> WalkEngine::zmpSolve(float zs, float z1, float z2, float x1, float x2) {
            /*
            Solves ZMP equation:
            x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
            where the ZMP point is piecewise linear:
            z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
            */
            float T1 = tStep * ph1Zmp;
            float T2 = tStep * ph2Zmp;
            float m1 = (zs - z1) / T1;
            float m2 = -(zs - z2) / (tStep - T2);

            float c1 = x1 - z1 + tZmp * m1 * std::sinh(-T1 / tZmp);
            float c2 = x2 - z2 + tZmp * m2 * std::sinh((tStep - T2) / tZmp);
            float expTStep = std::exp(tStep / tZmp);
            float aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
            float aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
            return std::make_pair(aP, aN);
        }

        arma::vec3 WalkEngine::zmpCom(float ph) {
            arma::vec3 com = {0, 0, 0};
            float expT = std::exp(tStep * ph / tZmp);
            com[0] = uSupport[0] + aXP * expT + aXN / expT;
            com[1] = uSupport[1] + aYP * expT + aYN / expT;
            if (ph < ph1Zmp) {
                com[0] = com[0] + m1X * tStep * (ph - ph1Zmp)
                -tZmp * m1X * std::sinh(tStep * (ph - ph1Zmp) / tZmp);
                com[1] = com[1] + m1Y * tStep * (ph - ph1Zmp)
                -tZmp * m1Y * std::sinh(tStep * (ph - ph1Zmp) / tZmp);
            } else if (ph > ph2Zmp) {
                com[0] = com[0] + m2X * tStep * (ph - ph2Zmp)
                -tZmp * m2X * std::sinh(tStep * (ph - ph2Zmp) / tZmp);
                com[1] = com[1] + m2Y * tStep * (ph - ph2Zmp)
                -tZmp * m2Y * std::sinh(tStep * (ph - ph2Zmp) / tZmp);
            }
            // com[2] = .5 * (uLeft[2] + uRight[2]);
            // Linear speed turning
            com[2] = ph* (uLeft2[2] + uRight2[2]) / 2 + (1 - ph) * (uLeft1[2] + uRight1[2]) / 2;
            return com;
        }

        std::pair<float, float> WalkEngine::footPhase(float ph) {
            // Computes relative x,z motion of foot during single support phase
            // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
            phSingle = std::min(std::max(ph - ph1Single, 0.0f) / (ph2Single - ph1Single), 1.0f);
            float phSingleSkew = std::pow(phSingle, 0.8) - 0.17 * phSingle * (1 - phSingle);
            float xf = 0.5 * (1 - std::cos(M_PI * phSingleSkew));
            float zf = 0.5 * (1 - std::cos(2 * M_PI * phSingleSkew));

            return std::make_pair(xf, zf);
        }

        double WalkEngine::getTime() {
              struct timeval t;
              gettimeofday(&t, NULL);
              return t.tv_sec + 1E-6 * t.tv_usec;
        }

		double WalkEngine::procFunc(double a, double deadband, double maxvalue) { //a function for IMU feedback (originally from teamdarwin2013release/player/util/util.lua)
			double   ret  = std::min( std::max(0., std::abs(a)-deadband), maxvalue);
			if(a<=0) ret *= -1.;
			return   ret;
		}

		double WalkEngine::modAngle(double value) { // reduce an angle to [-pi, pi)
            double angle = std::fmod(value, 2 * M_PI);
            if (angle <= -M_PI) angle += 2 * M_PI;
            else if (angle > M_PI) angle -= 2 * M_PI;

            return angle;
		}

		arma::vec3 WalkEngine::poseGlobal(arma::vec3 pRelative, arma::vec3 pose) { //TEAMDARWIN LUA VECs START INDEXING @ 1 not 0 !!
			double ca = std::cos(pose[2]);
			double sa = std::sin(pose[2]);
            return {
                pose[0] + ca * pRelative[0] - sa * pRelative[1],
                pose[1] + sa * pRelative[0] + ca * pRelative[1],
                pose[2] + pRelative[2]
            };
		}

		arma::vec3 WalkEngine::poseRelative(arma::vec3 pGlobal, arma::vec3 pose) {
			double ca = std::cos(pose[2]);
			double sa = std::sin(pose[2]);
			double px = pGlobal[0] - pose[0];
			double py = pGlobal[1] - pose[1];
			double pa = pGlobal[2] - pose[2];
            return {
                ca * px + sa * py,
                -sa * px + ca * py,
                modAngle(pa)
            };
		}

		//should t be an integer???
		arma::vec3 WalkEngine::se2Interpolate(double t, arma::vec3 u1, arma::vec3 u2) { //helps smooth out the motions using a weighted average
            return {
                u1[0] + t * (u2[0] - u1[0]),
                u1[1] + t * (u2[1] - u1[1]),
                u1[2] + t * modAngle(u2[2] - u1[2])
            };
		}

    }  // motion
}  // modules

