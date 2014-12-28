/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_WALKENGINE_H
#define MODULES_MOTION_WALKENGINE_H

#include <nuclear>
#include <armadillo>

#include "messages/support/Configuration.h"
#include "messages/behaviour/Action.h"
#include "messages/input/Sensors.h"
#include <yaml-cpp/yaml.h>

namespace modules {
namespace motion {

    /**
     * TODO
     *
     * @author Brendan Annable
     * @author Trent Houliston
     */
    class WalkEngine : public NUClear::Reactor {
    public:
        /**
         * The number of servo updates performnced per second
         */
        static constexpr size_t UPDATE_FREQUENCY = 60;

        static constexpr const char* CONFIGURATION_PATH = "WalkEngine.yaml";
        explicit WalkEngine(std::unique_ptr<NUClear::Environment> environment);
    private:

        enum State {
            STOPPED,
            STOP_REQUEST,
            LAST_STEP,
            WALKING
        };

        State state;
        bool interrupted;

        ReactionHandle updateHandle;

        /// Subsumption ID key to access motors
        const size_t id;
        arma::vec2 footOffset;

        // start_config_params
        bool emitLocalisation;
        // Walk Parameters

        // These limit the distance a footstep will take
        arma::mat::fixed<3,2> stepLimits;
        double stanceLimitY2;

        // Velocity limits for the walk
        arma::mat::fixed<3,2> velocityLimits;
        arma::vec3 accelerationLimits;
        arma::vec3 accelerationLimitsHigh;
        double velocityHigh;
        // Factor to slow down walk when turning
        double accelerationTurningFactor;

        // OP default stance width: 0.0375*2 = 0.075
        // Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
        // Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

        // Stance parameters

        // The body height of the robot, in meters; changing this will alter how high the robot's natural stance is.
        double bodyHeight;
        // Torso Y rotation
        double bodyTilt;

        // Servo gains during walk
        float gainArms;
        float gainLegs;

        // Gait parameters
        // The stepTime defines how long it will take for a robot to take its next step, in seconds.
        double stepTime;
        double zmpTime;
        // The height to which the robot raises its foot at each step. This parameter is very sensitive in terms of balance.
        double stepHeight;

        double phase1Single;
        double phase2Single;

        // Compensation parameters
        // double hipRollCompensation;
        // arma::vec2 ankleMod;

        /*// Gyro stabilization parameters
        arma::vec4 ankleImuParamX;
        arma::vec4 ankleImuParamY;
        arma::vec4 kneeImuParamX;
        arma::vec4 hipImuParamY;
        arma::vec4 armImuParamX;
        arma::vec4 armImuParamY;
        double balanceWeight;

        // Support bias parameters to reduce backlash-based instability
        double velFastForward;
        double velFastTurn;
        double supportFront;
        double supportFront2;
        double supportBack;
        double supportSideX;
        double supportSideY;
        double supportTurn;*/

        // Initial body swing
        // double toeTipCompensation;

        // end_config_params

        // walk state
        arma::vec3 uTorso = arma::zeros(3);
        arma::vec3 uTorsoSource = arma::zeros(3);
        arma::vec3 uTorsoDestination = arma::zeros(3);
        arma::vec3 uLeftFoot = arma::zeros(3);
        arma::vec3 uLeftFootSource = arma::zeros(3);
        arma::vec3 uLeftFootDestination = arma::zeros(3);
        arma::vec3 uRightFoot = arma::zeros(3);
        arma::vec3 uRightFootSource = arma::zeros(3);
        arma::vec3 uRightFootDestination = arma::zeros(3);

        // Current robot velocity
        arma::vec3 velocityCurrent;
        // Current velocity command
        arma::vec3 velocityCommand;
        // Difference between current velocity and commanded velocity
        arma::vec3 velocityDifference;

        // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpCoefficients = arma::zeros(4);
        // zmp params m1X, m2X, m1Y, m2Y
        arma::vec4 zmpParams = arma::zeros(4);

        // gyro stabilization variables
        arma::vec2 ankleShift;
        double kneeShift;
        arma::vec2 hipShift;
        arma::vec2 armShift;

        bool active;
        bool started;
        double beginStepTime;
        double phase;

        int currenstepTimeType;

        // How to begin initial step, unsure of affect
        // Should be an enum
        int initialStep;

        // current arm pose
        arma::vec3 qLArm;
        arma::vec3 qRArm;

        // standard offset
        arma::vec3 uLRFootOffset;

        // walking/stepping transition variables
        bool startFromStep;

        messages::behaviour::LimbID swingLegInitial = messages::behaviour::LimbID::LEFT_LEG;
        messages::behaviour::LimbID swingLeg = swingLegInitial;
        double shiftFactor;

        arma::vec3 uSupport;

        // double STAND_SCRIPT_DURATION;

        //void generateAndSaveStandScript();
        void configureWalk(const YAML::Node& config);

        void update(const messages::input::Sensors& sensors);
        void updateStep(const messages::input::Sensors& sensors);
        void updateStill(const messages::input::Sensors& sensors = messages::input::Sensors());
        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> motionLegs(std::vector<std::pair<messages::input::ServoID, float>> joints);
        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> motionArms();
        void balance(std::vector<double>& qLegs, const messages::input::Sensors& sensors);
        void localise(arma::vec3 position);

        void reset();
        void start();
        void stop();
        void requestStop();
        void calculateNewStep();
        void stanceReset();
        void setVelocity(double vx, double vy, double va);
        void updateVelocity();
        arma::vec3 getNewFootTarget(const arma::vec3& velocity, const arma::vec3& leftFoot, const arma::vec3& rightFoot, const messages::behaviour::LimbID& swingLeg);

        /**
         * Get the next torso position
         */
        arma::vec3 stepTorso(arma::vec3 uLeftFoot, arma::vec3 uRightFoot, double shiftFactor);

        /**
         * @return The current velocity
         */
        arma::vec3 getVelocity();

        /**
         * Solve the ZMP equation
         */
        arma::vec2 zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime);

        /**
         * Uses ZMP to determine the torso position
         *
         * @return The torso position in SE2
         */
        arma::vec3 zmpCom(double phase, arma::vec4 zmpCoefficients, arma::vec4 zmpParams, double stepTime, double zmpTime, double phase1Zmp, double phase2Zmp, arma::vec3 uSupport, arma::vec3 uLeftFootDestination, arma::vec3 uLeftFootSource, arma::vec3 uRightFootDestination, arma::vec3 uRightFootSource);

        /**
         * This is an easing function that returns 3 values {x,y,z} with the range [0,1]
         * This is used to 'ease' the foot path through its trajectory.
         * The params phase1Single and phase2Single are used to tune the amount of time the robot spends on two feet
         * Note: Only x/z are used currently and y is always 0
         * See: http://easings.net/ to reference common easing functions
         *
         * @param phase The input to the easing function, with a range of [0,1].
         * @param phase1Single The phase time between [0,1] to start the step. A value of 0.1 means the step will not start until phase is >= 0.1
         * @param phase2Single The phase time between [0,1] to end the step. A value of 0.9 means the step will end when phase >= 0.9
         */
        arma::vec3 footPhase(double phase, double phase1Single, double phase2Single);

        /**
         * @return get a unix timestamp (in decimal seconds that are accurate to the microsecond)
         */
        double getTime();

        /**
         * @return A clamped between 0 and maxvalue, offset by deadband
         */
        double procFunc(double a, double deadband, double maxvalue);

        /**
         * Local to world transform
         *
         * Transforms the givenpose from local space to world/global space
         * Note: Assumes vec3 are of the form {x, y, angle}
         */
        arma::vec3 localToWorld(arma::vec3 poseRelative, arma::vec3 pose);

        /**
         * World to local transform
         *
         * Transforms the given pose from world/global space to be relative to poseGlobal
         * Note: Assumes vec3 are of the form {x, y, angle}
         */
        arma::vec3 worldToLocal(arma::vec3 poseGlobal, arma::vec3 pose);

        /**
         * Interpolate between two given vectors
         * Note: Assumes vec3 are of the form {x, y, angle}
         * See: Special Euclidean group SE(2).
         * http://en.wikipedia.org/wiki/Euclidean_group
         *
         * @param t A value between 0-1 to interpolate between the two,
         * outside these bounds will extrapolate
         * @param u1 the first vector
         * @param u2 the second vector
         * @return The interpolated vector
         */
        arma::vec3 se2Interpolate(double t, arma::vec3 u1, arma::vec3 u2);
    };

}  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

