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

#ifndef MODULE_MOTION_FOOTPLACEMENTPLANNER_H
#define MODULE_MOTION_FOOTPLACEMENTPLANNER_H

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

    class FootPlacementPlanner : public NUClear::Reactor 
    {
    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;
        static constexpr const char* CONFIGURATION_PATH = "FootPlacementPlanner.yaml";
        static constexpr const char* CONFIGURATION_MSSG = "Foot Placement Planner - Configure";
        static constexpr const char* ONTRIGGER_FOOT_CMD = "Foot Placement Planner - Update Foot Position";
        static constexpr const char* ONTRIGGER_FOOT_TGT = "Foot Placement Planner - Calculate Target Foot Position";
        explicit FootPlacementPlanner(std::unique_ptr<NUClear::Environment> environment);
    private:
    	using LimbID         = message::input::LimbID;
        using ServoCommand   = message::behaviour::ServoCommand;
        using Sensors        = message::input::Sensors;
        using ServoID        = message::input::ServoID;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
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
        // Active torso position
        Transform2D torsoPositionTransform;
        // Pre-step torso position
        Transform2D torsoPositionSource;
        // Torso step target position
        Transform2D torsoPositionDestination;
        // Active left foot position
        Transform2D leftFootPositionTransform;
        // Pre-step left foot position
        Transform2D leftFootSource;
        // Active right foot position
        Transform2D rightFootPositionTransform;
        // Pre-step right foot position
        Transform2D rightFootSource;
        // Destination placement Transform2D left foot positions
        std::queue<Transform2D> leftFootDestination;
        // Destination placement Transform2D right foot positions
        std::queue<Transform2D> rightFootDestination;
        // Current robot velocity
        Transform2D velocityCurrent;
        // Current velocity command
        Transform2D velocityCommand;
        // ??
        Transform2D velocityDifference;
        // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID swingLeg;
        // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpCoefficients;
        // zmp params m1X, m2X, m1Y, m2Y
        arma::vec4 zmpParameters;
        // ??
        Transform2D uSupportMass;
        // end state

        double STAND_SCRIPT_DURATION;

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
        arma::vec2 footOffsetCoefficient;
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

        double hipRollCompensation;

        NUClear::clock::time_point lastVeloctiyUpdateTime;

        // jointGains are the current gains sent to the servos
        std::map<ServoID, float> jointGains;
        // servoControlPGains are the constant proportionality
        // constants which define the current values of jointGains based on the robot's balance state
        std::map<ServoID, float> servoControlPGains;

        utility::motion::Balancer balancer;

        NUClear::clock::time_point pushTime;
        
        // end config

        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param config [description]
         */
        void configure(const YAML::Node& config);
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void reset();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void start();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void requestStop();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void stop();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void calculateNewStep();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param velocity [description]
         */
        void setVelocity(Transform2D velocity);
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void updateVelocity();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void stanceReset();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getTorsoPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoPosition(const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getTorsoSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoSource(const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getTorsoDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoDestination(const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getSupportMass();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inSupportMass [description]
         */
        void setSupportMass(const Transform2D& inSupportMass);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param index [description]
         * @return [description]
         */
        double getFootOffsetCoefficient(int index);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inFootOffsetCoefficient [description]
         */
        void setFootOffsetCoefficient(const arma::vec2& inFootOffsetCoefficient);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param index [description]
         * @param inValue [description]
         */
        void setFootOffsetCoefficient(int index, double inValue);
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
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param velocity [description]
         * @param swingLeg [description]
         * 
         * @return [description]
         */
        Transform2D getNewFootTarget(const Transform2D& velocity, const LimbID& swingLeg);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getVelocity();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        double getTime();
    };
}  // motion
}  // modules

#endif  // MODULE_MOTION_FOOTPLACEMENTPLANNER_H
