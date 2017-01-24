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
#include <algorithm>
#include <chrono>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include "extension/Configuration.h"

#include "message/behaviour/FixedWalkCommand.h"
#include "message/behaviour/ServoCommand.h"

#include "message/input/PushDetection.h"
#include "message/input/PostureRecognition.h"

#include "message/motion/KinematicsModels.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/BalanceCommand.h"
#include "message/motion/TorsoMotionCommand.h"
#include "message/motion/FootPlacementCommand.h" 
#include "message/motion/FootMotionCommand.h" 
#include "message/motion/ServoTarget.h"

#include "message/localisation/FieldObject.h"

#include "message/support/SaveConfiguration.h"

#include "utility/behaviour/Action.h"

#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

#include "utility/motion/Script.h"

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
    class WalkEngine : public NUClear::Reactor 
    {
    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;

        static constexpr const char* CONFIGURATION_PATH = "WalkEngine.yaml";
        static constexpr const char* CONFIGURATION_MSSG = "Walk Engine - Configure";
        static constexpr const char* ONTRIGGER_BODY_INF = "Walk Engine - Received update (Balanced Robot Posture) Info";

        explicit WalkEngine(std::unique_ptr<NUClear::Environment> environment);
    private:
        using ServoCommand   = message::behaviour::ServoCommand;
        using LimbID         = utility::input::LimbID;
        using ServoID        = utility::input::ServoID;
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
        bool newPostureReceived;    // Identifies the instance of valid posture data for waypoint emission.

        /**
         * Resource abstractions for id and handler instances...
         */
        ReactionHandle handleUpdate;                    // handle(updateWaypoints), disabling when not moving will save unnecessary CPU resources
        ReactionHandle handleStandScript;               // handle(generateStandAndSaveScript), disabling when not required for capturing standing phase
        size_t subsumptionId;                           // subsumption ID key to access motors
        
        /**
         * Anthropomorphic metrics for relevant humanoid joints & actuators...
         */
        Transform3D leftFootPositionTransform;          // Active left foot position
        Transform3D rightFootPositionTransform;         // Active right foot position

        /**
         * Anthropomorphic metrics initialized from configuration script, see config file for documentation...
         */
        float  gainRArm;                                //
        float  gainRLeg;                                //
        float  gainLArm;                                //
        float  gainLLeg;                                //
        float  gainHead;                                //           

        /**
         * Arm Position vectors initialized from configuration script, see config file for documentation...
         */
        arma::vec3 armLPostureTransform;                //
        arma::vec3 armRPostureTransform;                //

        /**
         * Internal timing reference variables...
         */
        double STAND_SCRIPT_DURATION;                           //

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
         * Motion data initialized from configuration script, see config file for documentation...
         */
        double velFastForward;                          //
        double velFastTurn;                             //        
        /**
         * Balance & Kinematics module initialization...
         */
        message::motion::KinematicsModel kinematicsModel;   //

        /**
         * Actuator servo gains...
         */
        std::map<ServoID, float> jointGains;            // current gains sent to the servos
        std::map<ServoID, float> servoControlPGains;    // proportionality constants relating jointGains to humanoid stability
        
        /**
         * The last foot goal rotation...
         */
        UnitQuaternion lastFootGoalRotation;            //
        UnitQuaternion footGoalErrorSum;                //

        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void scriptStandAndSave();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void reset();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        std::unique_ptr<std::vector<ServoCommand>> updateWaypoints();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        std::unique_ptr<std::vector<ServoCommand>> motionLegs(std::vector<std::pair<ServoID, float>> joints);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        std::unique_ptr<std::vector<ServoCommand>> motionArms();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return A clamped between 0 and maxvalue, offset by deadband
         * 
         * @param inTorsoPosition [description]
         */
        void updateVelocity();
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
        bool isNewPostureReceived();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inNewPostureReceived [description]
         */
        void setNewPostureReceived(bool inNewPostureReceived);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return Current velocity
         * 
         * @param inTorsoPosition [description]
         */
        Transform2D getVelocity();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setVelocity(Transform2D velocity);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        arma::vec3 getLArmPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setLArmPosition(arma::vec3 inLArm);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        arma::vec3 getRArmPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setRArmPosition(arma::vec3 inRArm);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        Transform2D getTorsoPositionArms();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        Transform2D getTorsoPositionLegs();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        Transform3D getTorsoPosition3D();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoPositionArms(const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoPositionLegs(const Transform2D& inTorsoPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setTorsoPosition3D(const Transform3D& inTorsoPosition);
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
         * @return [description]
         */
        Transform3D getLeftFootPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootPosition [description]
         */
        void setLeftFootPosition(const Transform3D& inLeftFootPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform3D getRightFootPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootPosition [description]
         */
        void setRightFootPosition(const Transform3D& inRightFootPosition);
    };

}  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

