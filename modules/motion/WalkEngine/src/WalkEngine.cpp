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

#include "messages/motion/ServoWaypoint.h"
#include "messages/support/Configuration.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/math/matrix.h"

namespace modules {
    namespace motion {

        using messages::support::Configuration;
        
        WalkEngine::WalkEngine(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

			struct WalkCommand {
				float forwardSpeed; // percentage of max speed
				float rotationSpeed; // radians/s, positive = left rotation (right hand rule)
			};

            on<Trigger<Configuration<WalkEngine> > >([this](const Configuration<WalkEngine>& walkEngineConfig) {

                auto config = walkEngineConfig.config;
                // g Walk Parameters
                // g Stance and velocity limit values
                stanceLimitX = config["stanceLimitX"].as<arma::vec>();
                stanceLimitY = config["stanceLimitY"].as<arma::vec>();
                stanceLimitA = config["stanceLimitA"].as<arma::vec>();
                velLimitX = config["velLimitX"].as<arma::vec>();
                velLimitY = config["velLimitY"].as<arma::vec>();
                velLimitA = config["velLimitA"].as<arma::vec>();
                velDelta = config["velDelta"].as<arma::vec>();
                vaFactor = config["vaFactor"];

                velXHigh = config["velXHigh"];
                velDeltaXHigh = config["velDeltaXHigh"];

                // gToe/heel overlap checking values
                footSizeX = config["footSizeX"].as<arma::vec>();
                stanceLimitMarginY = config["stanceLimitMarginY"];
                stanceLimitY2 = 2 * double(config["footY"]) - double(config["stanceLimitMarginY"]);

                // gOP default stance width: 0.0375*2 = 0.075
                // gHeel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
                // gHeel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

                // gStance parameters
                bodyHeight = config["bodyHeight"];
                bodyTilt = config["bodyTilt"];
                // TODO
                //footX = mcm.get_footX();
                footY = config["footY"];
                supportX = config["supportX"];
                supportY = config["supportY"];
                qLArm0 = M_PI / 180.0 * arma::vec3{90, 2, -20};
                qRArm0 = M_PI / 180.0 * arma::vec3{90, -2, -20};
                qLArmKick0 = M_PI / 180 * arma::vec3{90, 30, -60};
                qRArmKick0 = M_PI / 180 * arma::vec3{90, -30, -60};

                // gHardness parameters
                hardnessSupport = config["hardnessSupport"];
                hardnessSwing = config["hardnessSwing"];

                hardnessArm0 = config["hardnessArm"];
                hardnessArm = config["hardnessArm"];

                // gGait parameters
                tStep = config["tStep"];
                tStep0 = tStep;
                tZmp = config["tZmp"];
                stepHeight = config["stepHeight"];
                stepHeight0 = stepHeight;
                ph1Single = config["phSingle"][1];
                ph2Single = config["phSingle"][2];
                ph1Zmp = ph1Single;
                ph2Zmp = ph2Single;

                // gCompensation parameters
                hipRollCompensation = 4 * M_PI / 180;
                ankleMod = arma::vec2{-1, 0} * 1 * M_PI / 180;
                spreadComp = config["spreadComp"];
                turnCompThreshold = config["turnCompThreshold"];
                turnComp = config["turnComp"];

                float gyroFactor = 0.273 * M_PI / 180 * 300 / 1024;

                // gGyro stabilization parameters
                ankleImuParamX = arma::vec3{0.5, 0.3 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                ankleImuParamY = arma::vec3{0.5, 1.2 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                kneeImuParamX = arma::vec3{0.5, 0.7 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                hipImuParamY = arma::vec3{0.5, 0.3 * gyroFactor, 1 * M_PI / 180, 25 * M_PI / 180};
                armImuParamX = arma::vec3{0.5, 10.0 * gyroFactor, 20 * M_PI / 180, 45 * M_PI / 180};
                armImuParamY = arma::vec3{0.5, 0.0 * gyroFactor, 20 * M_PI / 180, 45 * M_PI / 180};

                // gSupport bias parameters to reduce backlash-based instability
                velFastForward = config["velFastForward"];
                velFastTurn = config["velFastTurn"];
                supportFront = config["supportFront"];
                supportFront2 = config["supportFront2"];
                supportBack = config["supportBack"];
                supportSideX = config["supportSideX"];
                supportSideY = config["supportSideY"];
                supportTurn = config["supportTurn"];

                frontComp = config["frontComp"];
                AccelComp = config["AccelComp"];

                // gInitial body swing 
                supportModYInitial = config["supportModYInitial"];

                // gWalkKick parameters
                //walkKickDef = config["walkKickDef"];
                walkKickPh = config["walkKickPh"];
                toeTipCompensation = 0;

                useAlternativeTrajectory = config["useAlternativeTrajectory"];
                
                // g--------------------------------------------------------
                // g Walk state variables
                // g--------------------------------------------------------

                uTorso = {supportX, 0, 0};
                uLeft = {0, footY, 0};
                uRight = {0, -footY, 0};

                pLLeg = {0, footY, 0, 0, 0, 0};
                pRLeg = {0, -footY, 0, 0,0,0};
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

                stopRequest = 2;
                canWalkKick = 1; // gCan we do walkkick with this walk code?
                walkKickRequest = 0; 
                //walkKick = walkKickDef["FrontLeft"];
                currentStepType = 0;

                initialStep = 2;

                upperBodyOverridden = 0;
                motionPlaying = 0;

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
                stepKickReady = false;
                hasBall = 0;
            });

            on<Trigger<Every<1, Per<std::chrono::seconds> > > >([this](const time_t& time) {
                update();
            });
			
        }
        // TODO: add others
        
        void WalkEngine::update() {
            //advanceMotion();

            float footX = getFootX();
            double time = getTime();

            // TODO: bodyHeightCurrent = vcm.get_camera_bodyHeight();

            if (!active) {
                // TODO
                moving = false;
                updateStill();
                return;
            }

            if (!started) {
                started = true;
                tLastStep = time;
            }

            ph0 = ph;
            moving = true;

            // SJ: Variable tStep support for walkkick
            ph = (time - tLastStep) / tStep;

            if (ph > 1) {
                iStep++;
                ph = ph - std::floor(ph);
                tLastStep += tStep;
            }

            if (iStep > iStep0 && stopRequest == 2) {
                stopRequest = 0;
                active = false;
                return; // TODO: return "stop"
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

                //checkWalkKick();
                checkStepKick();

                if (stepKickReady) {
                    // large step init
                    return; // TODO: return "step"
                }

                if (walkKickRequest == 0 && stepKickRequest == 0) {
                    if (stopRequest == 1) {
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
                    // TODO
                    //leftLegHardness = hardnessSupport;
                    //rightLegHardness = hardnessSwing;
                } else {
                    arma::vec3 uRightTorso = poseRelative(uRight1, uTorso);
                    arma::vec3 uTorsoModded = poseGlobal({supportMod[1], supportMod[2], 0}, uTorso);
                    arma::vec3 uRightModded = poseGlobal(uRightTorso, uTorsoModded);
                    uSupport = poseGlobal({supportX, -supportY, 0}, uRightModded);
                    // TODO:
                    //leftLegHardness = hardnessSwing;
                    //rightLegHardness = hardnessSupport;
                }

                // compute ZMP coefficients
                m1X = (uSupport[0] - uTorso[0]) / (tStep * ph1Zmp);
                m2X = (uTorso2[0] - uSupport[0]) / (tStep * (1 - ph2Zmp));
                m1Y = (uSupport[1] - uTorso[1]) / (tStep * ph1Zmp);
                m2Y = (uTorso2[1] - uSupport[1]) / (tStep * (1 - ph2Zmp));
                std::tie(aXP, aXN) = zmpSolve(uSupport[0], uTorso1[0], uTorso[0], uTorso[0], uTorso[0]);
                std::tie(aYP, aYN) = zmpSolve(uSupport[1], uTorso1[1], uTorso[1], uTorso[1], uTorso[1]);

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
                /*if (currentStepType > 1) { // walkkick
                    if (xFoot < walkKickPh) {
                        uRight = se2Interpolate(xFoot * 2, uRight1, uRight15);
                    } else {
                        uRight = se2Interpolate(xFoot * 2 - 1, uRight15, uRight2);
                    }
                } else {
                    uRight = se2Interpolate(xFoot, uRight1, uRight2);
                }*/
                uRight = se2Interpolate(xFoot, uRight1, uRight2);
                pRLeg[2] = stepHeight * zFoot;
            } else {
                /*if (currentStepType > 1) { // walkkick
                    if (xFoot < walkKickPh) {
                        uLeft = se2Interpolate(xFoot * 2, uLeft1, uLeft15);
                    } else {
                        uLeft = se2Interpolate(xFoot * 2 - 1, uLeft15, uLeft2);
                    }
                } else {
                    uLeft = se2Interpolate(xFoot, uLeft1, uLeft2);
                }*/
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
            if (upperBodyOverridden > 0 || motionPlaying > 0) {
                // mass shift to X
                float elbowX = -std::sin(qLArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qLArmOR[1])
                               -std::sin(qRArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qRArmOR[1]);

                // mass shift to Y
                float elbowY = std::sin(qLArmOR[1]) + std::sin(qRArmOR[1]);
                armPosCompX = elbowX * -0.009;
                armPosCompY = elbowY * -0.009;

                pTorso[3] = bodyRot[0];
                pTorso[4] = bodyRot[1];
                pTorso[5] = bodyRot[2];
            } else {
                armPosCompX = 0;
                armPosCompY = 0;

                pTorso[3] = 0;
                pTorso[4] = bodyTilt;
                pTorso[5] = 0;
            }

            // TODO: paramaterize/config
            if (hasBall > 0) {
                turnCompX = turnCompX - 0.01;
            }

            arma::vec3 uTorsoActual = poseGlobal({-footX + frontCompX + turnCompX + armPosCompX, armPosCompY, 0}, uTorso);
            pTorso[0] = uTorsoActual[0];
            pTorso[1] = uTorsoActual[1];
            pTorso[5] += uTorsoActual[2];

            pLLeg[0] = uLeft[0];
            pLLeg[1] = uLeft[1];
            pLLeg[2] = uLeft[2];

            pRLeg[0] = uRight[0];
            pRLeg[1] = uRight[1];
            pRLeg[2] = uRight[2];

            // TODO: qLegs = IK()
            std::vector<double> qLegs;
            motionLegs(qLegs);
            motionArms();
        }

        void WalkEngine::checkStepKick() {
            arma::vec3 uFootErr;
            if (stepKickRequest == 0) {
                stepCheckCount = 0;
                return;
            } else if (stepKickRequest == 1) {
                uFootErr = poseRelative(uLeft1, poseGlobal(2 * uLRFootOffset, uRight1));
                stepCheckCount++;
            }

            if (supportLeg == stepKickSupport) {
                if (stepCheckCount > 2 || 
                        (std::abs(uFootErr[0]) < 0.02 &&
                        std::abs(uFootErr[1]) < 0.01 &&
                        std::abs(uFootErr[2]) < 10 * M_PI / 180.0)) {
                    stepKickReady = true;
                    return;
                }
            }

            if (supportLeg == LEFT) {
                uRight2 = poseGlobal(-2 * uLRFootOffset, uLeft1);
            } else {
                uLeft2 = poseGlobal(2 * uLRFootOffset, uRight1);
            }
        }

        /*void WalkEngine::checkWalkKick() {
            if (walkKickRequest == 0) {
                return;
            }

            if (walkKickRequest > 0 && walkKickRequest > walkKick.size()) {
                walkKickRequest = 0;
                tStep = tStep0;
                stepHeight = stepHeight0;
                currentStepType = 0;
                velCurrent = {0, 0, 0};
                velCommand = {0, 0, 0};
                return;
            }

            if (walkKickRequest == 1) {
                // check current supportLeg and feet positions
                // and advance steps until ready

                arma::vec3 uFootErr = poseRelative(uLeft1, poseGlobal(2 * uLRFootOffset, uRight1));

                if (supportLeg != walkKick[0][2] ||
                        std::abs(uFootErr[0] > 0.02) || 
                        std::abs(uFootErr[1] > 0.01) ||
                        std::abs(uFootErr[2] > 10 * M_PI / 180.0)) {
                    if (supportLeg == LEFT) {
                        // TODO: uRight2 = 
                    } else {
                        // TODO: uLeft2 = 
                    }
                    return;
                }
            }

            tStep = walkKick[walkKickRequest][0];
            currentStepType = walkKick[walkKickRequest][1];
            supportLeg = walkKick[walkKickRequest][2];
            stepHeight = walkKick[walkKickRequest][3];
            supportMod = walkKick[walkKickRequest][4];
            shiftFactor = walkKick[walkKickRequest][5];

            if (walkKick[walkKickRequest].size() <= 7) {
                arma::vec3 footPos1 = walkKick[walkKickRequest][6];
                if (supportLeg == LEFT) {
                    // TODO: look at uLRFootOffset for use here
                    // TODO: uRight2 = 
                } else {
                    // TODO: uLeft2 = 
                }
            } else {
                arma::vec3 footPos1 = walkKick[walkKickRequest][6];
                arma::vec3 footPos2 = walkKick[walkKickRequest][7];
                if (supportLeg == LEFT) {
                    // TODO: uRight15 = 
                    // TODO: uRight2 = 
                } else {
                    // TODO: uLeft15 =
                    // TODO: uLeft2 = 
                }
            }

            walkKickRequest++;
        }*/

        void WalkEngine::updateStill() {
            uTorso = stepTorso(uLeft, uRight, 0.5);

            float armPosCompX, armPosCompY;

            if (upperBodyOverridden > 0 || motionPlaying > 0) {
                // mass shift to X
                float elbowX = -std::sin(qLArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qLArmOR[1])
                               -std::sin(qRArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qRArmOR[1]);

                // mass shift to Y
                float elbowY = std::sin(qLArmOR[1]) + std::sin(qRArmOR[1]);
                armPosCompX = elbowX * -0.007;
                armPosCompY = elbowY * -0.007;

                pTorso[3] = bodyRot[0];
                pTorso[4] = bodyRot[1];
                pTorso[5] = bodyRot[2];
            } else {
                armPosCompX = 0;
                armPosCompY = 0;

                pTorso[3] = 0;
                pTorso[4] = bodyTilt;
                pTorso[5] = 0;
            }

            uTorsoActual = poseGlobal({-footX + armPosCompX, armPosCompY, 0}, uTorso);
            pTorso[0] = uTorsoActual[0];
            pTorso[1] = uTorsoActual[1];
            pTorso[5] += uTorsoActual[2];

            pLLeg[0] = uLeft[0];
            pLLeg[1] = uLeft[1];
            pLLeg[2] = uLeft[2];

            pRLeg[0] = uRight[0];
            pRLeg[1] = uRight[1];
            pRLeg[2] = uRight[2];

            // TODO: qLegs = IK()
            std::vector<double> qLegs;
            motionLegs(qLegs, true);
            motionArms();
        }

        void WalkEngine::motionLegs(std::vector<double> qLegs) {
            motionLegs(qLegs, false);
        }

        void WalkEngine::motionLegs(std::vector<double> qLegs, bool gyroOff) {
            float phComp = std::min({1.0, phSingle / 0.1, (1 - phSingle) / 0.1});

            // ankle stabilization using gyro feedback
            // TODO: imuGyr = 
            arma::vec3 imuGyr;

            float gyroRoll0 = imuGyr[0];
            float gyroPitch0 = imuGyr[1];
            if (gyroOff) {
                gyroRoll0 = 0;
                gyroPitch0 = 0;
            }

            // get effective gyro angle considering body angle offset
            float yawAngle = 0;
            if (!active) {
                // double support
                yawAngle = (uLeft[2] + uRight[2]) / 2 - uTorsoActual[2];
            } else if (supportLeg == LEFT) {
                yawAngle = uLeft[2] - uTorsoActual[2];
            } else if (supportLeg == RIGHT) {
                yawAngle = uRight[2] - uTorsoActual[2];
            }

            float gyroRoll = gyroRoll0 * std::cos(yawAngle) + gyroPitch0 * std::sin(yawAngle);
            float gyroPitch = gyroPitch0 * std::cos(yawAngle) + gyroRoll0 * std::sin(yawAngle);

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

            // TODO:
            //leftLegCommand = qLegs;
        }

        void WalkEngine::motionArms() {
            // TODO: paramaterize/config
            if (hasBall > 0) {
                return;
            }

            arma::vec3 qLArmActual = {qLArm0[0] + armShift[0], qLArm0[1] + armShift[1], 0};
            arma::vec3 qRArmActual = {qRArm0[0] + armShift[0], qRArm0[1] + armShift[1], 0};

            if (upperBodyOverridden > 0 || motionPlaying > 0) {
                qLArmActual = {qLArm0[0], qLArmOR[1], qLArmOR[2]};
                qRArmActual = {qRArm0[0], qRArmOR[1], qRArmOR[2]};
            }

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
            
            if (upperBodyOverridden <= 0 && motionPlaying <= 0) {
                qLArmActual[2] = qLArm[2];
                qRArmActual[2] = qRArm[2];
            }

            // TODO:
//            leftArmCommand = qLArmActual;
//            rightArmCommand = qRArmActual;
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
            double toeOverlap = -footSizeX[1] * uLeftRight[3];
            double heelOverlap = -footSizeX[2] * uLeftRight[3];
            double limitY = std::max(stanceLimitY[1], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

            // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

            uLeftRight[1] = std::min(std::max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2]);
            uLeftRight[2] = std::min(std::max(uLeftRight[2], limitY), stanceLimitY[2]);
            uLeftRight[3] = std::min(std::max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2]);

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
            double toeOverlap = footSizeX[1] * uRightLeft[3];
            double heelOverlap = footSizeX[2] * uRightLeft[3];
            double limitY = std::max(stanceLimitY[1], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

            // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

            uRightLeft[1] = std::min(std::max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2]);
            uRightLeft[2] = std::min(std::max(uRightLeft[2], -stanceLimitY[2]), -limitY);
            uRightLeft[3] = std::min(std::max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

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
                initialStep++;
            }
        }

        arma::vec3 WalkEngine::getVelocity() {
            return velCurrent;
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

        /*void WalkEngine::startMotion(std::string name) {
            if (motionPlaying == 0) {
                motionPlaying = 1;
                // TODO: currentMotion = 
                motionIndex = 1;
                motionStartTime = getTime();

                qLArmOR1 = currentMotion[0][1];
                qRArmOR1 = currentMotion[0][2];
                bodyRot0 = {0, bodyTilt, 0};

                // TODO: investigate port
                if (currentMotion[0] > 3) {
                    bodyRot1 = currentMotion[0][3];
                } else {
                    bodyRot1 = bodyRot0;
                }

                leftArmHardness = {0.7, 0.7, 0.7};
                rightArmHardness = {0.7, 0.7, 0.7};
            }
        }

        void WalkEngine::advanceMotion() {
            if (motionPlaying == 0) {
                return;
            }

            double time = getTime();
            curMotionFrame = currentMotion[motionIndex];
            ph = (time - motionStartTime) / curMotionFrame[0];
            if (ph > 1) {
                // advance frame
                // TODO: investigate port
                if (currentMotion == motionIndex) {
                    motionPlaying = 0;
                    leftArmHardness = hardnessArm;
                    rightArmHardness = hardnessArm;
                } else {
                    motionIndex++;
                    motionStartTime = time;
                    qLArmOR0[0] = qLArmOR1[0];
                    qLArmOR0[1] = qLArmOR1[1];
                    qLArmOR0[2] = qLArmOR1[2];

                    qRArmOR0[0] = qRArmOR1[0];
                    qRArmOR0[1] = qRArmOR1[1];
                    qRArmOR0[2] = qRArmOR1[2];

                    bodyRot0[0] = bodyRot1[0];
                    bodyRot0[1] = bodyRot1[1];
                    bodyRot0[2] = bodyRot1[2];

                    qLArmOR1 = currentMotion[motionIndex][1];
                    qRArmOR1 = currentMotion[motionIndex][2];

                    // TODO: investigate port
                    if (currentMotion[0] > 3) {
                        bodyRot1 = currentMotion[MotionIndex][3];
                    } else {
                        bodyRot1 = bodyRot0;
                    }
                }
            } else {
                    qLArmOR0[0] = (1 - ph) * qLArmOR0[0] + ph * qLArmOR1[0];
                    qLArmOR0[1] = (1 - ph) * qLArmOR0[1] + ph * qLArmOR1[1];
                    qLArmOR0[1] = (1 - ph) * qLArmOR0[2] + ph * qLArmOR1[2];

                    qRArmOR0[0] = (1 - ph) * qRArmOR0[0] + ph * qRArmOR1[0];
                    qRArmOR0[1] = (1 - ph) * qRArmOR0[1] + ph * qRArmOR1[1];
                    qRArmOR0[1] = (1 - ph) * qRArmOR0[2] + ph * qRArmOR1[2];

                    bodyRot0[0] = (1 - ph) * bodyRot0[0] + ph * bodyRot1[0];
                    bodyRot0[1] = (1 - ph) * bodyRot0[1] + ph * bodyRot1[1];
                    bodyRot0[2] = (1 - ph) * bodyRot0[2] + ph * bodyRot1[2];
            }
        }*/

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
            walkKickRequest = 0;
            currentStepType = 0;
            motionPlaying = 0;
            upperBodyOverridden = 0;
            uLRFootOffset = {0, footY, 0};
            startFromStep = false;
        }

        std::pair<arma::vec3, arma::vec3> WalkEngine::getOdometry(arma::vec3 u0) {
            arma::vec3 uFoot = se2Interpolate(0.5, uLeft, uRight);
            return {poseRelative(uFoot, u0), uFoot};
        }

        arma::vec3 WalkEngine::getBodyOffset() {
            arma::vec3 uFoot = se2Interpolate(0.5, uLeft, uRight);
            return poseRelative(uTorso, uFoot);
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


            /*if (useAlternativeTrajectory > 0) {
                float ph1FootPhase = 0.1;
                float ph2FootPhase = 0.5;
                float ph3FootPhase = 0.8;

//                float exp1FootPhase = 2;
//                float exp2FootPhase = 2;
//                float exp3FootPhase = 2;

//                float zFootLand = 0.3;    

                float phZTemp;
                float phXTemp;

                if (phSingle < ph1FootPhase) {
                    phZTemp = phSingle / ph2FootPhase;
                    // xf = 0;
                    // zf = 1 - (1-phZTemp)^exp1FootPhase;
                } else if (phSingle < ph2FootPhase) {
                    phXTemp = (phSingle - ph1FootPhase) / (ph3FootPhase - ph1FootPhase);
                    phZTemp = phSingle / ph2FootPhase;
                    // xf =  .5*(1-math.cos(math.pi*phXTemp));
                    // zf = 1 - (1-phZTemp)^exp1FootPhase;
                } else if (phSingle < ph3FootPhase) {
                    phXTemp = (phSingle - ph1FootPhase) / (ph3FootPhase - ph1FootPhase);
                    phZTemp = (phSingle - ph2FootPhase) / (ph3FootPhase - ph2FootPhase);
                    // xf =  .5*(1-math.cos(math.pi*phXTemp));
                    // zf = 1 - phZTemp^exp2FootPhase*(1-zFootLand);
                } else {
                    phZTemp = (1 - phSingle) / (1 - ph3FootPhase);
                    // xf = 1;
                    // zf = phZTemp^exp3FootPhase*zFootLand;
                }
            }*/

            return std::make_pair(xf, zf);
        }

        float WalkEngine::getFootX() {
            return float(config["footX"]) + float(config["footXComp"]);
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

		double WalkEngine::modAngle(double a) { // reduce an angle to [-pi, pi)
			if(a==0) return 0.;
			a = std::fmod(a, (2. * M_PI)); //fmod(a,b) the same as a%b, but for doubles (fmod() defined in math.h)
			if(a >= M_PI)
				a -= 2. * M_PI;
			return a;
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

