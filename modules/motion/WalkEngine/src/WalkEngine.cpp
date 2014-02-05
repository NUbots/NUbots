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

#include <armadillo>
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
                config = walkEngineConfig.config;
            });

            on<Trigger<Every<1, Per<std::chrono::seconds> > > >([this](const time_t& time) {
                update();
            });
			
        }
        
        void WalkEngine::update() {
            advanceMotion();

            float footX = getFootX();
            time_t time = NUClear::clock::now();

            // TODO: float bodyHeightCurrent = TODO

            if (!active) {
                // TODO
                moving = false;
                // update_still();
                return;
            }

            if (!started) {
                started = true;
                tLastStep = time;
            }

            ph0 = ph;
            moving = true;

            // SJ: Variable tStep support for walkkick
            ph = (time - tLastStep) / config["tStep"];

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

                supportMod = {0,0}; // support point modulation for wallkick
                shiftFactor = 0.5; // how much should we shift final torso pose?

                checkWalkKick();
                checkStepKick();

                if (stepKickReady) {
                    // large step init
                    return; // TODO: return "step"
                }

                if (walkKickRequest == 0 && stepKickRequest == 0) {
                    if (stopRequest == 1) {
                        stopRequest = 2;
                        // TODO: velCurrent
                        // TODO: velCommand
                        if (supportLeg == LEFT) {
                            // TODO: uRight2 = 
                        } else {
                            // TODO: uLeft2 = 
                        }
                    } else {
                        // normal walk, advance steps
                        tStep = tStep0;
                        if (supportLeg == LEFT) {
                            // TODO: uRight2 = 
                        } else {
                            // TODO: uLeft2
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

                // TODO: uTorso2 = 

                // adjustable initial step body swing
                if (initialStep > 0) {
                    supportMod[1] = supportModYInitial;
                    if (supportLeg == RIGHT) {
                        supportMod[1] *= -1;
                    }
                }

                // apply velocity-based support point modulation for uSupport
                if (supportLeg == LEFT) {
                    // TODO: uLeftTorso = 
                    // TODO: uTorsoModded =
                    arma::vec3 uTorso = {supportMod[0], supportMod[1], 0};
                    // TODO: uLeftModded = 
                    // TODO: uSupport = 
                    leftLegHardness = hardnessSupport;
                    rightLegHardness = hardnessSwing;
                } else {
                    // TODO: uRightTorso = 
                    // TODO: uTorsoModded =
                    arma::vec3 uTorso = {supportMod[0], supportMod[1], 0};
                    // TODO: uRightModded = 
                    // TODO: uSupport = 
                    leftLegHardness = hardnessSwing;
                    rightLegHardness = hardnessSupport;
                }

                // compute ZMP coefficients
                m1X = (uSupport[0] - uTorso[0]) / (tStep * ph1Zmp);
                m2X = (uTorso2[0] - uSpport[0]) / (tStep * (1 - ph2Zmp));
                m1Y = (uSupport[1] - uTorso[1]) / (tStep * ph1Zmp);
                m2Y = (uTorso2[1] - uSupport[1]) / (tStep * (1 - ph2Zmp));
                std::map<aXP, aXN> = zmp_solve(uSupport[0], uTorso1[0], uTorso[0], uTorso[0], uTorso[0]);
                std::map<aYP, aYN> = zmp_solve(uSupport[1], uTorso1[1], uTorso[1], uTorso[1], uTorso[1]);

                // compute COM speed at the boundary

                dx0 = (aXP - aXN) / tZmp + m1X * (1 - std::cosh(ph1Zmp * tStep / tZmp));
                dy0 = (aYP - aYN) / tZmp + m1Y * (1 - std::cosh(ph1Zmp * tStep / tZmp));

                dx1 = (aXP * std::exp(tStep / tZmp) - aXN * std::exp(-tStep / tZmp)) / tZmp
                        + m2X * (1 - std::cosh((1 - ph2Zmp) * tStep / tZmp));
                dy1 = (aYP * std::exp(tStep / tZmp) - aYN * std::exp(-tStep / tZmp)) / tZmp
                        + m2Y * (1 - std::cosh((1 - ph2Zmp) * tStep / tZmp));

                comdot = {dx1, dy1};
            }

            std::map<xFoot, zFoot> = footPhase(ph);
            if (initialStep > 0) {
                zFoot = 0; // don't lift foot at initial step
            }
            pLLeg[2] = 0;
            pRLeg[2] = 0;
            if (supportLeg == LEFT) {
                if (currentStepType > 1) { // walkkick
                    if (xFoot < walkKickPh) {
                        // TODO: uRight = 
                    } else {
                        // TODO: uRight = 
                    }
                } else {
                    // TODO: uRight = 
                }
                pRLeg[2] = stepHeigh * zFoot;
            } else {
                if (currentStepType > 1) { // walkkick
                    if (xFoot < walkKickPh) {
                        // TODO: uLeft = 
                    } else {
                        // TODO: uLeft = 
                    }
                } else {
                    // TODO: uLeft = 
                }
                pLLeg[2] = stepHeigh * zFoot;
            }

            uTorsoOld = uTorso;

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

            // arm movement compensation
            if (upperBodyOverridden > 0 || motionPlaying > 0) {
                // mass shift to X
w               elbowX = -std::sin(qLArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qLArmOR[1])
                        - std::sin(qRArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qRArmOR[1]);

                // mass shift to Y
                elbowY = std::sin(qLArmOR[1]) + std::sin(qRArmOR[1]);
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

            // TODO: uTorsoActual = 
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
            motionLegs(qLegs);
            motionArms();
        }

        void WalkEngine::checkStepKick() {
            if (stepKickRequest == 0) {
                stepCheckCount = 0;
                return;
            } else if (stepKickRequest == 1) {
                // tODO: uFootErr = 
                stepCheckCount++;
            }

            if (supportLeg == stepKickSupport) {
                if (stepCheckCount > 2) || 
                        (std::abs(uFootErr[0]) < 0.02 &&
                        std::abs(uFootErr[1]) < 0.01 &&
                        std::abs(uFootErr[2]) < 10 * M_PI / 180.0) {
                    stepKickReady = true;
                    return;
                }
            }

            if (supportLeg == LEFT) {
                // TODO: uRight2 = 
            } else {
                // TODO: uLeft2 = 
            }
        }

        void WalkEngine::checkWalkKick() {
            if (walkKickRequest == 0) {
                return;
            }

            if (walkKickRequest > 0 && walkKickRequest > walkKick) {
                walkKickRequest = 0;
                tStep = tStep0;
                stepHeight = stepHeight0;
                currentStepType = 0;
                velCurrent = {0,0,0};
                velCommad = {0,0,0};
                return;
            }

            if (walkRequestType == 1) {
                // check current supportLeg and feet positions
                // and advance steps until ready

                // TODO: uFootErr = 

                if (supportLeg ~= walkKick[0][2] ||
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

            if (walkKick[walkKickRequest] <= 7) { // TODO: investigate port
                footPos1 = walkKick[walkKickRequest][6];
                if (supportLeg == LEFT) {
                    // TODO: look at uLRFootOffset for use here
                    // TODO: uRight2 = 
                } else {
                    // TODO: uLeft2 = 
                }
            } else {
                footPos1 = walkKick[walkKickRequest][6];
                footPos2 = walkKick[walkKickRequest][7];
                if (supportLeg == LEFT) {
                    // TODO: uRight15 = 
                    // TODO: uRight2 = 
                } else {
                    // TODO: uLeft15 =
                    // TODO: uLeft2 = 
                }
            }

            walkKickRequest++;
        }

        void WalkEngine::updateStill() {
            uTorso = stepTorso(uLeft, uRight, 0.5);

            if (upperBodyOverriden > 0 || motionPlaying > 0) {
                // mass shift to X
                elbowX = -std::sin(qLArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qLArmOR[1])
                        - std::sin(qRArmOR[0] - M_PI_2 + bodyRot[0]) * std::cos(qRArmOR[1]);

                // mass shift to Y
                elbowY = std::sin(qLArmOR[1]) + std::sin(qRArmOR[1]);
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

            // TODO: uTorsoActual = 
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
            motionLegs(qLegs, true);
            motionArms();
        }

        void WalkEngine::motionLegs(qLegs, gyroOff) {
            phComp = std::min(1, phSingle / 0.1, (1 - phSingle) / 0.1);

            // ankle stabilization using gyro feedback
            // TODO: imuGyr = 

            gyroRoll0 = imuGyr[0];
            gyroPitch0 = imuGyr[1];
            if (gyroOff) {
                gyroRoll0 = 0;
                gyroPitch0 = 0;
            }

            // get effective gyro angle considering body angle offset
            if (!active) {
                // double support
                yawAngle = (uLeft[2] + uRight[2]) / 2 - uTorsoActual[2];
            } else if (supportLeg == LEFT) {
                yawAngle = uLeft[2] - uTorsoActual[2];
            } else if (supportLeg == RIGHT) {
                yawAngle = uRight[2] - uTorsoActual[2];
            }

            gyroRoll = gyroRoll0 * std::cos(yawAngle)
                    + gyroPitch0 * std::sin(yawAngle);
            gyroPitch = gyroPitch0 * std::cos(yawAngle)
                    + gyroRoll0 * std::sin(yawAngle);

            // TODO: armShiftX = 
            // TODO: armShiftY = 

            // TODO: ankleShiftX = 
            // TODO: ankleShiftY = 
            // TODO: kneeShiftX = 
            // TODO: hipShiftY = 

            ankleShift[0] = ankleShift[0] + ankleImuParamX[0] * (ankleShiftX - ankleShift[0]);
            ankleShift[1] = ankleShift[1] + ankleImuParamY[0] * (ankleShiftY - ankleShift[1]);
            kneeShift = kneeShift + kneeImuParamX[0] * (kneeShiftX - kneeShift);
            hipShift[1] = hipShift[1] + hipImuParamY[0] * (hipShiftY - hipShift[1]);
            armShift[0] = armShift[0] + armImuParamX[0] * (armShiftX - armShift[0]);
            armShift[1] = armShift[1] + armImuParamY[0] * (armShiftY - armShift[1]);
            
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

            leftLegCommand = qLegs;
        }

        void WalkEngine::motionArms() {
            // TODO: paramaterize/config
            if (hasBall > 0) {
                return;
            }

            arma::vec3 qLArmActual = {qLArm0[0] + armShift[0], qLArm0[1] + armShift[1], 0};
            arma::vec3 qRArmActual = {qRArm0[0] + armShift[0], qRArm0[1] + armShift[1], 0};

            if (upperBodyOverridden > 0 || motionPlaying > 0) {
                qLArmActual = {qLArm0[0], qLArmOR[1], qLArmOR[2};
                qRArmActual = {qRArm0[0], qRArmOR[1], qRArmOR[2]};
            }

            // check leg hitting
            // TODO: rotLeftA = 
            // TODO: rotRightA = 

            // TODO: leftLegTorso = 
            // TODO: rightLegTorso = 

            qLArmActual[1] = std::max(
                    5 * M_PI / 180 + std::max(0, rotLeftA) / 2
                    + std::max(0, leftLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180)
                    , qLArmActual[1]);

            qRArmActual[1] = std::max(
                    -5 * M_PI / 180 + std::max(0, rotRightA) / 2
                    - std::max(0, rightLegTorso[1] - 0.04) / 0.02 * (6 * M_PI / 180)
                    , qLArmActual[1]);
            
            if (uppoerBodyOverridden <= 0 && motionPlaying <= 0) {
                qLArmActual[2] = qLArm[2];
                qRArmActual[2] = qRArm[2];
            }

            leftArmCommand = qLArmActual;
            rightArmCommand = qRArmActual;
        }

        void WalkEngine::exit() {
            // TODO: empty?
        }

        void WalkEngine::stepLeftDestination(vel, uLeft, uRight) {
            // TODO
        }

        void WalkEngine::stepRightDestination(vel, uLeft, uRight) {
            // TODO
        }

        void WalkEngine::stepTorso(uLeft, uRight, shiftFactor) {
            // TODO
        }

        void WalkEngine::setVelocity(vx, vy, va) {
            // filter the commanded speed
            vx = std::min(std::max(vx, velLimitX[0]), velLimitX[1]);
            vy = std::min(std::max(vy, velLimitY[0]), velLimitY[1]);
            va = std::min(std::max(va, velLimitA[0]), velLimitA[1]);
            
            // slow down when turning
            vFactor = 1 - std::abs(va) / vaFactor;

            stepMag = std::sqrt(vx * vx + vy * vy);
            magFactor = std::min(velLimit[1] * vFactor, stepMag) / (stepMag + 0.000001);

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

            if (initial_step > 0) {
                velCurrent = arma::vec3;
                initial_step++;
            }
        }

        void WalkEngine::getVelocity() {
            return velCurrent;
        }

        void WalkEngine::start() {
            stopRequest = 0;
            if (!active) {
                time_t now = NUClear::clock::now();

                active = true;
                started = false;
                iStep0 = -1;
                t0 = now;
                tLastStep = onw;
                initialStep = 2;
            }
        }

        void WalkEngine::stop() {
            // always stops with feet together (which helps transition)
            stopRequest = std::max(1, stopRequest);
        }

        void WalkEngine::startMotion(std::string name) {
            if (motionPlaying == 0) {
                motionPlaying = 1;
                // TODO: currentMotion = 
                motionIndex = 1;
                motionStartTime = NUClear::clock::now();

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

            time_t time = NUClear::clock::now();
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
        }

        void WalkEngine::setInitialStance(uL, uR, uT, support) {
            uLeftI = uL;
            uRightI = uR;
            uTorso = uT;
            supportI = uT;
            startFromStep = true;
        }

        void stanceReset() {
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
                // TODO: uLeft = 
                // TODO: uRight = 
                iStep0 = -1;
                iStep = 0;
            }

            uLeft1 = uLeft;
            uLeft2 = uLeft;

            uRight1 = uRight;
            uRight2 = uRight;

            uSupport = uTorso;
            tLastStep = NUClear::clock::now();
            walkKickRequest = 0;
            currentStepType = 0;
            motionPlaying = 0;
            upperBodyOverridden = 0;
            uLRFootOffset = {0, footY, 0};
            startFromStep = false;
        }

        void WalkEngine::getOdometry(u0) {
            if (!u0) {
                u0 = {0,0,0};
            }
            // TODO: uFoot = 
            // TODO: return
        }

        void WalkEngine::getBodyOffset() {
            // TODO: uFoot = 
            // TODO: return
        }

        std::pair<float, float> WalkEngine::zmpSolve(zs, z1, z2, x1, x2) {
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
        
        void WalkEngine::zmpCom(ph) {
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

        std::pair<float, float> WalkEngine::footPhase(ph) {
            // Computes relative x,z motion of foot during single support phase
            // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
            phSingle = std::min(std::max(ph - ph1Single, 0) / (ph2Single - ph1Single), 1);
            float phSingleSkew = std::pow(phSingle, 0.8) - 0.17 * phSingle * (1 - phSingle);
            float xf = 0.5 * (1 - std::cos(M_PI * phSingleSkew));
            float zf = 0.5 * (1 - std::cos(2 * M_PI * phSingleSkew));


            if (use_alternative_trajectory > 0) {
                ph1FootPhase = 0.1;
                ph2FootPhase = 0.5;
                ph3FootPhase = 0.8;

                exp1FootPhase = 2;
                exp2FootPhase = 2;
                exp3FootPhase = 2;

                zFootLand = 0.3;    

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
            }

            return std::make_pair(xf, zf);
        }

        float WalkEngine::getFootX() {
            return config["footX"] + config["footXComp"];
        }
        
        
    }  // motion
}  // modules
