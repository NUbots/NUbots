/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_MOTION_FOOTMOTIONPLANNER_H
#define MODULE_MOTION_FOOTMOTIONPLANNER_H

#include <nuclear>
#include <armadillo>

#include <yaml-cpp/yaml.h>

#include "message/support/Configuration.h"
#include "message/behaviour/Action.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/FootMotionCommand.h" 
#include "message/motion/FootPlacementCommand.h" 
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/Balance.h"
#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"

namespace module 
{
namespace motion 
{
    class FootMotionPlanner : public NUClear::Reactor 
    {
    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;
        static constexpr const char* CONFIGURATION_PATH = "FootMotionPlanner.yaml";
        static constexpr const char* CONFIGURATION_MSSG = "Foot Motion Planner - Configure";
        static constexpr const char* ONTRIGGER_FOOT_CMD = "Foot Motion Planner - Update Foot Position";
        static constexpr const char* ONTRIGGER_FOOT_TGT = "Foot Motion Planner - Received Target Foot Position";
        explicit FootMotionPlanner(std::unique_ptr<NUClear::Environment> environment);
    private:
        using LimbID         = message::input::LimbID;
        using ServoCommand   = message::behaviour::ServoCommand;
        using Sensors        = message::input::Sensors;
        using ServoID        = message::input::ServoID;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        //Debug output
        bool DEBUG = true; 

        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
        ReactionHandle updateHandle;

        bool startFromStep;
        // Update to step is received
        bool updateStepInstruction;
        // The time when the current is to be completed
        double destinationTime;
        // How to many 'steps' to take before lifting a foot when starting to walk
        int initialStep;
        // Pre-step left foot position
        Transform2D leftFootSource;
        // Pre-step right foot position
        Transform2D rightFootSource;
        // Destination placement Transform2D left foot positions
        std::queue<Transform2D> leftFootDestination;
        // Destination placement Transform2D right foot positions
        std::queue<Transform2D> rightFootDestination;
        // TODO: ??? Appears to be support foot pre-step position
        Transform2D uSupport;
        // Current robot velocity
        Transform2D velocityCurrent;
        // Current velocity command
        Transform2D velocityCommand;
        // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpCoefficients;
        // zmp params m1X, m2X, m1Y, m2Y
        arma::vec4 zmpParams;
        // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID swingLeg;
        // The last foot goal rotation
        UnitQuaternion lastFootGoalRotation;
        UnitQuaternion footGoalErrorSum;

        // end state

        // start config, see config file for documentation

        double stanceLimitY2;
        arma::mat::fixed<3,2> stepLimits;
        arma::mat::fixed<3,2> velocityLimits;
        arma::vec3 accelerationLimits;
        arma::vec3 accelerationLimitsHigh;
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
        arma::vec2 footOffset;
        // standard offset
        Transform2D uLRFootOffset;
        // arm poses
        arma::vec3 qLArmStart;
        arma::vec3 qLArmEnd;
        arma::vec3 qRArmStart;
        arma::vec3 qRArmEnd;
        LimbID swingLegInitial = LimbID::LEFT_LEG;

        bool balanceEnabled;
        bool emitLocalisation;
        bool emitFootPosition;

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

        /**
         * @return get a unix timestamp (in decimal seconds that are accurate to the microsecond)
         */
        double getTime();
        double getDestinationTime();
        void setDestinationTime(double inDestinationTime);

        bool getNewStepReceived();
        void setNewStepReceived(bool inUpdateStepInstruction);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getLeftFootPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootPosition [description]
         */
        void setLeftFootPosition(const Transform2D& inLeftFootPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getRightFootPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootPosition [description]
         */
        void setRightFootPosition(const Transform2D& inRightFootPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getLeftFootSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootSource [description]
         */
        void setLeftFootSource(const Transform2D& inLeftFootSource);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getRightFootSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootSource [description]
         */
        void setRightFootSource(const Transform2D& inRightFootSource);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getLeftFootDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootDestination [description]
         */
        void setLeftFootDestination(const Transform2D& inLeftFootDestination);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getRightFootDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootDestination [description]
         */
        void setRightFootDestination(const Transform2D& inRightFootDestination);

        double getMotionPhase();
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

        void updateFootPosition(double phase, const Transform2D& leftFootDestination, const Transform2D& rightFootDestination);

        void configure(const YAML::Node& config);
    };

}  // motion
}  // modules

#endif  // MODULE_MOTION_FOOTMOTIONPLANNER_H
