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
        static constexpr const char* ONTRIGGER_FTMN_INF = "Walk Engine - Received update (Balanced Foot Position) Info";
        static constexpr const char* ONTRIGGER_TRSM_INF = "Walk Engine - Received update (Balanced Torso Position) Info";
        static constexpr const char* ONTRIGGER_HEAD_INF = "Walk Engine - Received update (Balanced Head Position) Info";

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
                Frame3D = Transform2D();
            }
            ~TorsoPositions() {}

            Transform2D FrameArms;
            Transform2D FrameLegs;
            Transform3D Frame3D;
        };
        TorsoPositions torsoPositionsTransform;         // Active torso position
        Transform2D torsoPositionSource;                // Pre-step torso position
        Transform2D torsoPositionDestination;           // Torso step target position
        Transform2D leftFootPositionTransform;          // Active left foot position
        Transform2D leftFootSource;                     // Pre-step left foot position
        Transform2D rightFootPositionTransform;         // Active right foot position
        Transform2D rightFootSource;                    // Pre-step right foot position
        std::queue<Transform2D> leftFootDestination;    // Destination placement Transform2D left foot positions
        std::queue<Transform2D> rightFootDestination;   // Destination placement Transform2D right foot positions
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
        arma::vec2 footOffset;                          //
        Transform2D uLRFootOffset;                      // standard offset

        /**
         * Arm Position vectors initialized from configuration script, see config file for documentation...
         */
        arma::vec3 armLPostureTransform;                // 
        arma::vec3 armLPostureSource;                   //  
        arma::vec3 armLPostureDestination;              //  
        arma::vec3 armRPostureTransform;                //  
        arma::vec3 armRPostureSource;                   //  
        arma::vec3 armRPostureDestination;              //  

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
        void generateAndSaveStandScript(const Sensors& sensors);
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
        void localise(Transform2D position); 
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        std::unique_ptr<std::vector<ServoCommand>> updateWaypoints(const Sensors& sensors);
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
         * Solve the ZMP equation
         */
        arma::vec2 zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime);
        /**
         * Uses ZMP to determine the torso position
         *
         * @return The torso position in Transform2D
         */
        Transform2D zmpTorsoCompensation(double phase, arma::vec4 zmpCoefficients, arma::vec4 zmpParams, double stepTime, double zmpTime, double phase1Zmp, double phase2Zmp, Transform2D uSupport, Transform2D uLeftFootDestination, Transform2D uLeftFootSource, Transform2D uRightFootDestination, Transform2D uRightFootSource);
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
         * @brief [brief description]
         * @details [long description]
         * @return A clamped between 0 and maxvalue, offset by deadband
         * 
         * @param inTorsoPosition [description]
         */
        double linearInterpolationDeadband(double a, double deadband, double maxvalue);
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
        arma::vec3 getLArmSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setLArmSource(arma::vec3 inLArm);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        arma::vec3 getLArmDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setLArmDestination(arma::vec3 inLArm);
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
        arma::vec3 getRArmSource();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setRArmSource(arma::vec3 inRArm);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        arma::vec3 getRArmDestination();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        void setRArmDestination(arma::vec3 inRArm);
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
    };

}  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

