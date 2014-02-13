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

#ifndef MODULES_MOTION_WALKENGINE_H
#define MODULES_MOTION_WALKENGINE_H

#include <nuclear>
#include <armadillo>

#include "utility/configuration/ConfigurationNode.h"

namespace modules {
    namespace motion {

        /**
         * TODO
         * 
         * @author Trent Houliston
         */
        class WalkEngine : public NUClear::Reactor {
        public:
            static constexpr const char* CONFIGURATION_PATH = "WalkEngine.json";
            explicit WalkEngine(std::unique_ptr<NUClear::Environment> environment);
        private:
            utility::configuration::ConfigurationNode config;

            enum Leg {
                LEFT,
                RIGHT
            };

            // start_config_params

            // Walk Parameters
            // Stance and velocity limit values
            arma::vec2 stanceLimitX;
            arma::vec2 stanceLimitY;
            arma::vec2 stanceLimitA;
            arma::vec2 velLimitX;
            arma::vec2 velLimitY;
            arma::vec2 velLimitA;
            arma::vec3 velDelta;
            float vaFactor;

            double velXHigh;
            double velDeltaXHigh;

            // Toe/heel overlap checking values
            arma::vec2 footSizeX;
            float stanceLimitMarginY;
            float stanceLimitY2;

            // OP default stance width: 0.0375*2 = 0.075
            // Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
            // Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

            // Stance parameters
            float bodyHeight;
            float bodyTilt;
            float footX;
            float footY;
            float supportX;
            float supportY;
            arma::vec3 qLArm0;
            arma::vec3 qRArm0;
            arma::vec3 qLArmKick0;
            arma::vec3 qRArmKick0;

            // Hardness parameters
            float hardnessSupport;
            float hardnessSwing;

            float hardnessArm0;
            float hardnessArm;

            // Gait parameters
            float tStep0;
            float tStep;
            float tZmp;
            float stepHeight0;
            float stepHeight;
            float ph1Single;
            float ph2Single;
            float ph1Zmp;
            float ph2Zmp;

            // Compensation parameters
            float hipRollCompensation;
            arma::vec2 ankleMod;
            float spreadComp;
            float turnCompThreshold;
            float turnComp;

            // Gyro stabilization parameters
            arma::vec4 ankleImuParamX;
            arma::vec4 ankleImuParamY;
            arma::vec4 kneeImuParamX;
            arma::vec4 hipImuParamY;
            arma::vec4 armImuParamX;
            arma::vec4 armImuParamY;

            // Support bias parameters to reduce backlash-based instability
            float velFastForward;
            float velFastTurn;
            float supportFront;
            float supportFront2;
            float supportBack;
            float supportSideX;
            float supportSideY;
            float supportTurn;

            float frontComp;
            float AccelComp;

            // Initial body swing 
            float supportModYInitial;

            // TODO: WalkKick parameters
//            walkKickDef = Config.walk.walkKickDef;
            float walkKickPh;
            float toeTipCompensation;

            bool useAlternativeTrajectory;

            // end_config_params

            // walk state
            arma::vec3 uTorso;
            arma::vec3 uTorso1;
            arma::vec3 uTorso2;
            arma::vec3 uLeft;
            arma::vec3 uLeft1;
            arma::vec3 uLeft2;
            arma::vec3 uRight;
            arma::vec3 uRight1;
            arma::vec3 uRight2;

            arma::vec pLLeg;
            arma::vec pRLeg;
            arma::vec pTorso;

            arma::vec3 velCurrent;
            arma::vec3 velCommand;
            arma::vec3 velDiff;

            // zmp expoential coefficients
            float aXP;
            float aXN;
            float aYP;
            float aYN;

            // gyro stabilization variables
            arma::vec2 ankleShift;
            float kneeShift;
            arma::vec2 hipShift;
            arma::vec2 armShift;

            bool active;
            bool started;
            bool moving;
            int iStep0;
            int iStep;
            float t0;
            float tLastStep;
            float ph0;
            float ph;

            int stopRequest;
            int canWalkKick; // can we do walkkick with this walk code?
            size_t walkKickRequest;
            //std::vector<std::vector<double> > walkKick;
            int currentStepType;

            int initialStep;

            int upperBodyOverridden;
            int motionPlaying;

            // TODO:
            arma::vec3 qLArmOR;
            arma::vec3 qRArmOR;

            arma::vec3 qLArmOR0;
            arma::vec3 qRArmOR0;

            arma::vec3 qLArmOR1;
            arma::vec3 qRArmOR1;

            arma::vec3 bodyRot;
            arma::vec3 bodyRot0;
            arma::vec3 bodyRot1;

            float phSingle;

            // current arm pose
            arma::vec3 qLArm;
            arma::vec3 qRArm;

            // standard offset
            arma::vec3 uLRFootOffset;

            // walking/stepping transition variables
            arma::vec3 uLeftI;
            arma::vec3 uRightI;
            arma::vec3 uTorsoI;
            Leg supportI;
            bool startFromStep;

            arma::vec2 comdot;
            bool stepKickReady;
            int hasBall;

            Leg supportLeg = LEFT;
            Leg stepKickSupport = LEFT;
            arma::vec2 supportMod;
            float shiftFactor;
            int stepKickRequest;

            // TODO: link to actuator
            float leftLegHardness;
            float rightLegHardness;

            // TODO: do these need to be global?
            float m1X;
            float m2X;
            float m1Y;
            float m2Y;

            arma::vec3 uSupport;
            arma::vec3 uTorsoActual;

            // TODO: default 0
            int stepCheckCount;
            int motionIndex;
            float motionStartTime;

            // TODO: sort it out
//            arma::vec3 leftArmCommand;
//            arma::vec3 rightArmCommand;
//            arma::vec3 leftLegCommand;
//            arma::vec3 rightLegCommand;

            void update();
            void checkStepKick();
            void checkWalkKick();
            void updateStill();
            void motionLegs(std::vector<double> qLegs);
            void motionLegs(std::vector<double> qLegs, bool gyroOff);
            void motionArms();
            void exit();
            arma::vec3 stepLeftDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight);
            arma::vec3 stepRightDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight);
            arma::vec3 stepTorso(arma::vec3 uLeft, arma::vec3 uRight, float shiftFactor);
            void setVelocity(double vx, double vy, double va);
            void updateVelocity();
            arma::vec3 getVelocity();
            void start();
            void stop();
//            void startMotion(std::string name);
//            void advanceMotion();
            void setInitialStance(arma::vec3 uL, arma::vec3 uR, arma::vec3 uT, Leg support);
            void stanceReset();
            std::pair<arma::vec3, arma::vec3> getOdometry(arma::vec3 u0);
            arma::vec3 getBodyOffset();
            std::pair<float, float> zmpSolve(float zs, float z1, float z2, float x1, float x2);
            arma::vec3 zmpCom(float ph);
            std::pair<float, float> footPhase(float ph);
            float getFootX();

            double getTime(); // TODO: remove
            double procFunc(double a, double deadband, double maxvalue); //TODO: move documentation from .cpp to .h file
            double modAngle(double a);
            arma::vec3 poseGlobal(arma::vec3 pRelative, arma::vec3 pose);
            arma::vec3 poseRelative(arma::vec3 pGlobal, arma::vec3 pose);
            arma::vec3 se2Interpolate(double t, arma::vec3 u1, arma::vec3 u2);
        };
    
    }  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

