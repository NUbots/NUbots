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
#include "message/behaviour/ServoCommand.h"
#include "message/behaviour/FixedWalkCommand.h"

#include "message/input/Sensors.h"
#include "message/input/PushDetection.h"

#include "message/motion/WalkCommand.h"
#include "message/motion/FootMotionCommand.h" 
#include "message/motion/FootPlacementCommand.h" 
#include "message/motion/ServoTarget.h"
#include "message/motion/Script.h"

#include "message/localisation/FieldObject.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

#include "utility/motion/Balance.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"

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

        /**
         * Temporary debugging variables for local output logging...
         */ 
        bool DEBUG;                 //
        int  DEBUG_ITER;            //

        /**
         * NUsight feedback initialized from configuration script, see config file for documentation...
         */
        bool balanceEnabled;        //
        bool emitLocalisation;      //
        bool emitFootPosition;      //

        /**
         * Resource abstractions for id and handler instances...
         */
        ReactionHandle updateHandle;                    // handle(updateWaypoints), disabling when not moving will save unnecessary CPU resources
        
        /**
         * Anthropomorphic metrics for relevant humanoid joints & actuators...
         */
        Transform2D leftFootPositionTransform;          // Active left foot position
        Transform2D rightFootPositionTransform;         // Active right foot position
        std::queue<Transform2D> activeLimbSource;       // Pre-step active limb position
        std::queue<Transform2D> activeLimbDestination;  // Destination placement Transform2D active foot positions
        std::queue<LimbID> activeForwardLimb;           // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID activeLimbInitial;                       // TODO: Former initial non-support leg for deterministic walking approach

         /**
         * Anthropomorphic metrics initialized from configuration script, see config file for documentation...
         */
        double bodyTilt;                                // 
        double bodyHeight;                              //
        double stepTime;                                //
        double stepHeight;                              //
        float  step_height_slow_fraction;               //
        float  step_height_fast_fraction;               //
        arma::mat::fixed<3,2> stepLimits;               //              
        arma::vec2 footOffset;                          //
        Transform2D uLRFootOffset;                      // standard offset

        /**
         * Internal timing reference variables...
         */
        bool  INITIAL_STEP;                                     // Indicates if the first step has been consumed
        double newStepStartTime;                                // The time when the current step is scheduled
        std::queue<double> destinationTime;                     // The relative time when the current is to be completed
        NUClear::clock::time_point lastVeloctiyUpdateTime;      //

        /**
         * Motion data for relevant humanoid actuators...
         */
        double velocityHigh;                            // 
        double accelerationTurningFactor;               //
        arma::mat::fixed<3,2> velocityLimits;           //
        arma::vec3 accelerationLimits;                  //
        arma::vec3 accelerationLimitsHigh;              //
        std::queue<Transform2D> velocityCurrent;        // Current robot velocity

        /**
         * Dynamic analysis parameters for relevant motion planning...
         */
        arma::vec4 zmpCoefficients;                     // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpParameters;                       // zmp params m1X, m2X, m1Y, m2Y

        /**
         * Dynamic analysis parameters initialized from configuration script, see config file for documentation...
         */
        double zmpTime;                                 // 
        double phase1Single;                            //
        double phase2Single;                            //

        /**
         * Balance & Kinematics module initialization...
         */

        /**
         * The last foot goal rotation...
         */
        UnitQuaternion lastFootGoalRotation;            //
        UnitQuaternion footGoalErrorSum;                //


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        bool startFromStep;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
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
        arma::vec3 getFootPhase(double phase, double phase1Single, double phase2Single);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void updateFootPosition(double inPhase, const Transform2D& inActiveLimbSource, const LimbID& inActiveForwardLimb, const Transform2D& inActiveLimbDestination);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void configure(const YAML::Node& config);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return get a unix timestamp (in decimal seconds that are accurate to the microsecond)
         * 
         * @param inTorsoPosition [description]
         */
        double getTime();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        double getNewStepStartTime();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void setNewStepStartTime(double inNewStartTime);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        double getDestinationTime();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void setDestinationTime(double inDestinationTime);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        bool isNewStepAvailable();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        bool isNewStepReceived();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        LimbID getActiveForwardLimb();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void setActiveForwardLimb(const LimbID& inActiveForwardLimb);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getVelocityCurrent();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param velocity [description]
         */
        void setVelocityCurrent(Transform2D inVelocityCommand);
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
        Transform2D getActiveLimbSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootSource [description]
         */
        void setActiveLimbSource(const Transform2D& inActiveLimbSource);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getActiveLimbDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootDestination [description]
         */
        void setActiveLimbDestination(const Transform2D& inActiveLimbDestination);
    };

}  // motion
}  // modules

#endif  // MODULE_MOTION_FOOTMOTIONPLANNER_H
