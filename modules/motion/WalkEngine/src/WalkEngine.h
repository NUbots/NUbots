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
            /*bool active;
            bool started;
            bool moving;
            time_t tLastStep;
            float ph0 = 0;
            float ph = 0;
            int iStep0 = -1;
            int iStep = 0;
            int stopRequest = 2;
            enum {
                LEFT,
                RIGHT
            } supportLeg = LEFT;
            // TODO: uLeft1
            // TODO: uRight1
            // TODO: uTorso1
            arma::vec2 supportMod;
            float shiftFactor;

            void update();
            void advanceMotion();
            void updateVelocity();
            float getFootX();*/

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
            int tStep0;
            int tStep;
            time_t t0;
            time_t tLastStep;
            float ph0;
            float ph;

            int stopRequest;
            int canWalkKick; // can we do walkkick with this walk code?
            int walkKickRequest;
            // TODO: walkKick = 
            int currentStepType;

            int initialStep;

            int upperBodyOverriden;
            int motionPlaying;

            // TODO:
            arma::vec3 qLArmOR0;
            arma::vec3 qRArmOR0;

            arma::vec3 qLArmOR1;
            arma::vec3 qRArmOR1;

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
            arma::vec3 supportI;
            bool startFromStep;

            arma::vec2 comdot;
            bool stepKickReady;
            int hasBall;

            enum {
                LEFT,
                RIGHT
            } supportLeg = LEFT;
            arma::vec2 supportMod;
            float shiftFactor;
            int stepKickRequest;

            void update();
            void checkStepKick();
            void checkWalkKick();
            void updateStill();
            void motionLegs(std::vector<double> qLegs, bool gyroOff);
            void motionArms();
            void exit();
            void stepLeftDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight);
            void stepRightDestination(arma::vec3 vel, arma::vec3 uLeft, arma::vec3 uRight);
            void stepTorso(arma::vec3 uLeft, arma::vec3 uRight, arma::vec3 shiftFactor);
            void setVelocity(float vx, float vy, float va);
            void updateVelocity();
            void getVelocity();
            void start();
            void stop();
            void startMotion(std::string name);
            void advanceMotion();
            void setInitialStance(arma::vec3 uL, arma::vec3 uR, arma::vec3 uT, arma::vec3 support);
            void stanceReset();
            void getOdometry(arma::vec3 u0);
            void getBodyOffset();
            std::pair<float, float> zpmSolve(float zs, float z1, float z2, float x1, float x2);
            void zmpCop(float ph);
            std::pair<float, float> footPhase(float ph);
            float getFootX();
        };
    
    }  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

