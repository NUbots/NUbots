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

#include "message/support/Configuration.h"

#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/behaviour/FixedWalkCommand.h"

#include "message/input/Sensors.h"
#include "message/input/PushDetection.h"

#include "message/motion/KinematicsModels.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/BalanceCommand.h"
#include "message/motion/TorsoMotionCommand.h"
#include "message/motion/FootPlacementCommand.h" 
#include "message/motion/FootMotionCommand.h" 
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
        int  initialStep;           // TODO: How to many 'steps' to take before lifting a foot when starting to walk

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
        ReactionHandle generateStandScriptReaction;     // handle(generateStandAndSaveScript), disabling when not required for capturing standing phase
        size_t subsumptionId;                           // subsumption ID key to access motors

        /**
         * The current abstract state of WalkEngine...
         */
        enum State {
            STOPPED,            // Walk engine has completely stopped and standing still
            STOP_REQUEST,       // A stop request has been made but not received
            LAST_STEP,          // Stop request has been made and now taking the last step before stopping
            WALKING             // Walk engine is walking as normal
        } StateOfWalk;
        
        /**
         * Anthropomorphic metrics for relevant humanoid joints & actuators...
         */
        struct TorsoPositions                           // Active torso relative positions struct
        {
            TorsoPositions() 
            : FrameArms()
            , FrameLegs()
            , Frame3D()
            {
                FrameArms = Transform2D();
                FrameLegs = Transform2D();
                Frame3D = Transform3D();
            }
            ~TorsoPositions() {}

            Transform2D FrameArms;
            Transform2D FrameLegs;
            Transform3D Frame3D;
        };
        TorsoPositions torsoPositionsTransform;         // Active torso position
        Transform2D leftFootPositionTransform;          // Active left foot position
        Transform2D rightFootPositionTransform;         // Active right foot position
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
        float  gainArms;                                //
        float  gainLegs;                                //
        arma::mat::fixed<3,2> stepLimits;               //              
        arma::vec2 footOffsetCoefficient;               //
        Transform2D uLRFootOffset;                      // standard offset

        /**
         * Arm Position vectors initialized from configuration script, see config file for documentation...
         */
        arma::vec3 armLPostureTransform;                //
        arma::vec3 armRPostureTransform;                //

        /**
         * Ankle Position vectors initialized from configuration script, see config file for documentation...
         */
    //  arma::vec4 ankleImuParamX;                      //
    //  arma::vec4 ankleImuParamY;                      //
    //  arma::vec4 kneeImuParamX;                       //
    //  arma::vec4 hipImuParamY;                        //
    //  arma::vec4 armImuParamX;                        //
    //  arma::vec4 armImuParamY;                        //

        /**
         * Internal timing reference variables...
         */
        double beginStepTime;                                   // The time when the current step begun
        double STAND_SCRIPT_DURATION;                           //
        NUClear::clock::time_point pushTime;                    //
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
         * Motion data initialized from configuration script, see config file for documentation...
         */
    //  double velFastForward;                          //
    //  double velFastTurn;                             //
        
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
        utility::motion::Balancer balancer;                             //
        message::motion::kinematics::KinematicsModel kinematicsModel;   //

        /**
         * Balance parameters initialized from configuration script, see config file for documentation...
         */
        double balanceAmplitude;                //
        double balanceWeight;                   //
        double balanceOffset;                   //
        double balancePGain;                    //
        double balanceIGain;                    //
        double balanceDGain;                    //

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
        void generateAndSaveStandScript();
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
        void start();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void requestStop();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void stop();
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
        Transform2D getLeftFootPosition();
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

