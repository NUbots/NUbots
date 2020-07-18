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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_OLDWALKENGINE_H
#define MODULES_MOTION_OLDWALKENGINE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/motion/Balance.h"


namespace module {
namespace motion {

    /**
     * TODO
     *
     * @author Brendan Annable
     * @author Trent Houliston
     */
    class OldWalkEngine : public NUClear::Reactor {
    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;

        static constexpr const char* CONFIGURATION_PATH = "OldWalkEngine.yaml";
        explicit OldWalkEngine(std::unique_ptr<NUClear::Environment> environment);

    private:
        using ServoCommand   = message::behaviour::ServoCommand;
        using Sensors        = message::input::Sensors;
        using LimbID         = utility::input::LimbID;
        using ServoID        = utility::input::ServoID;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        enum State {
            /**
             * Walk engine has completely stopped and standing still
             */
            STOPPED,

            /**
             * A stop request has been made but not received
             */
            STOP_REQUEST,

            /**
             * Stop request has been made and now taking the last step before stopping
             */
            LAST_STEP,

            /**
             * Walk engine is walking as normal
             */
            WALKING
        };

        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
        ReactionHandle updateHandle;

        // start state

        // The state of the current walk
        State state;
        // // Whether subsumption has currently interrupted the walk engine
        // bool interrupted;
        // TODO: ???
        bool startFromStep;
        // The time when the current step begun
        double beginStepTime;
        // How to many 'steps' to take before lifting a foot when starting to walk
        int initialStep;
        // Current torso position
        Eigen::Affine2d uTorso;
        // Pre-step torso position
        Eigen::Affine2d uTorsoSource;
        // Torso step target position
        Eigen::Affine2d uTorsoDestination;
        // Current left foot position
        Eigen::Affine2d uLeftFoot;
        // Pre-step left foot position
        Eigen::Affine2d uLeftFootSource;
        // Left foot step target position
        Eigen::Affine2d uLeftFootDestination;
        // Current right foot position
        Eigen::Affine2d uRightFoot;
        // Pre-step right foot position
        Eigen::Affine2d uRightFootSource;
        // Right foot step target position
        Eigen::Affine2d uRightFootDestination;
        // TODO: ??? Appears to be support foot pre-step position
        Eigen::Affine2d uSupport;
        // Current robot velocity
        Eigen::Affine2d velocityCurrent;
        // Current velocity command
        Eigen::Affine2d velocityCommand;
        // Difference between current velocity and commanded velocity
        Eigen::Affine2d velocityDifference;
        // zmp expoential coefficients aXP aXN aYP aYN
        Eigen::Vector4d zmpCoefficients;
        // zmp params m1X, m2X, m1Y, m2Y
        Eigen::Vector4d zmpParams;
        // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID swingLeg;
        // The last foot goal rotation
        Eigen::Quaternion<double> lastFootGoalRotation;
        Eigen::Quaternion<double> footGoalErrorSum;

        // end state

        // start config, see config file for documentation

        bool use_com;
        double stanceLimitY2;
        Eigen::Matrix<double, 3, 2> stepLimits;
        Eigen::Matrix<double, 3, 2> velocityLimits;
        Eigen::Vector3d accelerationLimits;
        Eigen::Vector3d accelerationLimitsHigh;
        double velocityHigh;
        double accelerationTurningFactor;
        double bodyHeight;
        double bodyTilt;
        float gainArms;
        float gainLegs;
        double stepTime;
        double zmpTime;
        double stepHeight;
        float step_height_slow_fraction;
        float step_height_fast_fraction;
        double phase1Single;
        double phase2Single;
        Eigen::Vector2d footOffset;
        float legYaw;
        // Ankle feedback parameters
        double ankleRollComp;
        double ankleRollLimit;
        double anklePitchComp;
        double anklePitchLimit;
        // standard offset
        Eigen::Affine2d uLRFootOffset;
        // arm poses
        Eigen::Vector3d qLArmStart;
        Eigen::Vector3d qLArmEnd;
        Eigen::Vector3d qRArmStart;
        Eigen::Vector3d qRArmEnd;
        LimbID swingLegInitial = LimbID::LEFT_LEG;

        double balanceEnabled;
        double balanceAmplitude;
        double balanceWeight;
        double balanceOffset;
        double balancePGain;
        double balanceIGain;
        double balanceDGain;

        NUClear::clock::time_point lastVeloctiyUpdateTime;

        // jointGains are the current gains sent to the servos
        std::map<ServoID, float> jointGains;
        // servoControlPGains are the constant proportionality
        // constants which define the current values of jointGains based on the robot's balance state
        std::map<ServoID, float> servoControlPGains;

        utility::motion::Balancer balancer;

        NUClear::clock::time_point pushTime;

        message::motion::KinematicsModel kinematicsModel;


        /*arma::vec4 ankleImuParamX;
        arma::vec4 ankleImuParamY;
        arma::vec4 kneeImuParamX;
        arma::vec4 hipImuParamY;
        arma::vec4 armImuParamX;
        arma::vec4 armImuParamY;

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
        double hipRollCompensation;

        // end config

        double STAND_SCRIPT_DURATION;
        ReactionHandle generateStandScriptReaction;

        void generateAndSaveStandScript(const Sensors& sensors);
        void configure(const YAML::Node& config);

        void reset();
        void start();
        void requestStop();
        void stop();

        void update(const Sensors& sensors);
        void updateStep(double phase, const Sensors& sensors);
        void updateStill(const Sensors& sensors = Sensors());
        std::unique_ptr<std::vector<ServoCommand>> updateStillWayPoints(const Sensors& sensors);

        void calculateNewStep();
        void setVelocity(Eigen::Affine2d velocity);
        void updateVelocity();
        void stanceReset();

        std::unique_ptr<std::vector<ServoCommand>> motionLegs(std::vector<std::pair<ServoID, float>> joints);
        std::unique_ptr<std::vector<ServoCommand>> motionArms(double phase);

        Eigen::Affine2d getNewFootTarget(const Eigen::Affine2d& velocity,
                                         const Eigen::Affine2d& leftFoot,
                                         const Eigen::Affine2d& rightFoot,
                                         const LimbID& swingLeg);

        /**
         * Get the next torso position
         */
        Eigen::Affine2d stepTorso(Eigen::Affine2d uLeftFoot, Eigen::Affine2d uRightFoot, double shiftFactor);

        /**
         * @return The current velocity
         */
        Eigen::Affine2d getVelocity();

        /**
         * Solve the ZMP equation
         */
        Eigen::Vector2d zmpSolve(double zs,
                                 double z1,
                                 double z2,
                                 double x1,
                                 double x2,
                                 double phase1Single,
                                 double phase2Single,
                                 double stepTime,
                                 double zmpTime);

        /**
         * Uses ZMP to determine the torso position
         *
         * @return The torso position in Eigen::Affine2d
         */
        Eigen::Affine2d zmpCom(double phase,
                               Eigen::Vector4d zmpCoefficients,
                               Eigen::Vector4d zmpParams,
                               double stepTime,
                               double zmpTime,
                               double phase1Zmp,
                               double phase2Zmp,
                               Eigen::Affine2d uSupport,
                               Eigen::Affine2d uLeftFootDestination,
                               Eigen::Affine2d uLeftFootSource,
                               Eigen::Affine2d uRightFootDestination,
                               Eigen::Affine2d uRightFootSource);

        /**
         * This is an easing function that returns 3 values {x,y,z} with the range [0,1]
         * This is used to 'ease' the foot path through its trajectory.
         * The params phase1Single and phase2Single are used to tune the amount of time the robot spends on two feet
         * Note: Only x/z are used currently and y is always 0
         * See: http://easings.net/ to reference common easing functions
         *
         * @param phase The input to the easing function, with a range of [0,1].
         * @param phase1Single The phase time between [0,1] to start the step. A value of 0.1 means the step will not
         * start until phase is >= 0.1
         * @param phase2Single The phase time between [0,1] to end the step. A value of 0.9 means the step will end when
         * phase >= 0.9
         */
        Eigen::Vector3d footPhase(double phase, double phase1Single, double phase2Single);

        /**
         * @return get a unix timestamp (in decimal seconds that are accurate to the microsecond)
         */
        double getTime();

        /**
         * @return A clamped between 0 and maxvalue, offset by deadband
         */
        double procFunc(double a, double deadband, double maxvalue);
    };

}  // namespace motion
}  // namespace module

#endif  // MODULES_MOTION_OLDWALKENGINE_H
