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

#include "extension/Configuration.h"

#include "message/behaviour/FixedWalkCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/behaviour/Subsumption.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/motion/KinematicsModels.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/FootMotionCommand.h" 
#include "message/motion/FootPlacementCommand.h"
#include "message/motion/TorsoMotionCommand.h" 
#include "message/motion/ServoTarget.h"

#include "utility/behaviour/Action.h"

#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

#include "utility/motion/Balance.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/Script.h"

#include "utility/nubugger/NUhelpers.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

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
        static constexpr const char* ONTRIGGER_FOOT_CMD = "Foot Placement Planner - Update Foot Target";
        static constexpr const char* ONTRIGGER_FOOT_TGT = "Foot Placement Planner - Calculate Target Foot Position";

        explicit FootPlacementPlanner(std::unique_ptr<NUClear::Environment> environment);
    private:
    	using LimbID         = message::behaviour::Subsumption::Limb::Value;
        using ServoCommand   = message::behaviour::ServoCommand;
        using Sensors        = message::input::Sensors;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        /**
         * Temporary debugging variables for local output logging...
         */ 
        bool DEBUG;                 //
        int  DEBUG_ITER;            //
        double  EPSILON;            //

        /**
         * NUsight feedback initialized from configuration script, see config file for documentation...
         */
        

        /**
         * Resource abstractions for id and handler instances...
         */
        ReactionHandle updateHandle;                    // handle(updateWaypoints), disabling when not moving will save unnecessary CPU resources
        ReactionHandle generateStandScriptReaction;     // handle(generateStandAndSaveScript), disabling when not required for capturing standing phase

        /**
         * Decision abstractions and notify variables...
         */
        bool startFromStep;

        /**
         * Anthropomorphic metrics for relevant humanoid joints & actuators...
         */
        Transform2D torsoPositionTransform;             // Active torso position
        Transform2D torsoPositionSource;                // Pre-step torso position
        Transform2D torsoPositionDestination;           // Torso step target position
        Transform2D leftFootPositionTransform;          // Active left foot position
        Transform2D leftFootSource;                     // Pre-step left foot position
        Transform2D rightFootPositionTransform;         // Active right foot position
        Transform2D rightFootSource;                    // Pre-step right foot position
        Transform2D leftFootDestination;                // Destination placement Transform2D left foot positions
        Transform2D rightFootDestination;               // Destination placement Transform2D right foot positions
        Transform2D uSupportMass;                       // Appears to be support foot pre-step position
        LimbID activeForwardLimb;                       // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID activeLimbInitial;                       // TODO: Former initial non-support leg for deterministic walking approach
        
        /**
         * Anthropomorphic metrics initialized from configuration script, see config file for documentation...
         */
        double bodyTilt;                                // 
        double bodyHeight;                              //
    //  double supportFront;                            //
    //  double supportFront2;                           //
    //  double supportBack;                             //
    //  double supportSideX;                            //
    //  double supportSideY;                            //
    //  double supportTurn;                             //    
        double stanceLimitY2;                           //
        double stepTime;                                //
        double stepHeight;                              //
        float  step_height_slow_fraction;               //
        float  step_height_fast_fraction;               //
        arma::mat::fixed<3,2> stepLimits;               //              
        arma::vec2 footOffsetCoefficient;               //
        Transform2D uLRFootOffset;                      // standard offset

        /**
         * Internal timing reference variables...
         */
        double beginStepTime;                                   // The time when the current step begun
        double STAND_SCRIPT_DURATION;                           //
        NUClear::clock::time_point lastVeloctiyUpdateTime;      //

        /**
         * Motion data for relevant humanoid actuators...
         */
        double velocityHigh;                            // 
        double accelerationTurningFactor;               //
        arma::mat::fixed<3,2> velocityLimits;           //
        arma::vec3 accelerationLimits;                  //
        arma::vec3 accelerationLimitsHigh;              //
        Transform2D velocityCurrent;                    // Current robot velocity
        Transform2D velocityCommand;                    // Current velocity command

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
        message::motion::KinematicsModel kinematicsModel;   //

        /**
         * The last foot goal rotation...
         */
        UnitQuaternion lastFootGoalRotation;            //
        UnitQuaternion footGoalErrorSum;                //

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
        bool isZeroVelocityRequired();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void postureInitialize();  
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param config [description]
         */
        void calculateNewStep(const Transform2D& inVelocityCurrent, const Transform2D& inTorsoSource, const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param velocity [description]
         * @param swingLeg [description]
         * 
         * @return [description]
         */
        Transform2D getNewFootTarget(const Transform2D& inVelocity, const LimbID& inActiveForwardLimb);
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
        Transform2D getVelocityCommand();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param velocity [description]
         */
        void setVelocityCommand(Transform2D inVelocityCommand);
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void updateVelocity();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void configure(const YAML::Node& config);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        double getTime();
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
    };
}  // motion
}  // modules

#endif  // MODULE_MOTION_FOOTPLACEMENTPLANNER_H
