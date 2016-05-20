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
#include "message/motion/WalkCommand.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/Script.h"
#include "message/localisation/FieldObject.h"

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/Balance.h"
<<<<<<< 1fcf3aa7e210a5c300ac935bfe7e3c77c76c1746
#include "message/motion/KinematicsModels.h"


namespace module {
namespace motion {

    /**
     * TODO
     *
     * @author Brendan Annable
     * @author Trent Houliston
     */
=======
#include "utility/motion/RobotModels.h"
#include "utility/motion/Balance.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"

namespace module 
{
namespace motion 
{
>>>>>>> Renaming modular walk -> walk engine + previous walk (refernce module)
    class WalkEngine : public NUClear::Reactor {
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

        // The current state of the walk engine...
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
        } StateOfWalk;

        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main updateWaypoints loop, disabling when not moving will save unnecessary CPU
        ReactionHandle updateHandle;

        // // Whether subsumption has currently interrupted the walk engine
        // bool interrupted;
        // TODO: ???
        bool startFromStep;
        // The time when the current step begun
        double beginStepTime;
        // How to many 'steps' to take before lifting a foot when starting to walk
        int initialStep;
        // Active torso relative positions struct
        struct TorsoPositions 
        {
            Transform2D FrameArms;
            Transform2D FrameLegs;
            Transform3D Frame3D;
        };
        // Active torso position
        TorsoPositions torsoPositionsTransform;
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
        // TODO: ??? Appears to be support foot pre-step position
        Transform2D uSupportMass;
        // Current robot velocity
        Transform2D velocityCurrent;
        // Current velocity command
        Transform2D velocityCommand;
        // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpCoefficients;
        // zmp params m1X, m2X, m1Y, m2Y
        arma::vec4 zmpParameters;
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
        arma::vec3 armLPostureTransform;
        arma::vec3 armLPostureSource;
        arma::vec3 armLPostureDestination;
        arma::vec3 armRPostureTransform;
        arma::vec3 armRPostureSource;
        arma::vec3 armRPostureDestination;

        arma::vec3 getLArmPosition();
        void setLArmPosition(arma::vec3 inLArm);
        arma::vec3 getLArmSource();
        void setLArmSource(arma::vec3 inLArm);
        arma::vec3 getLArmDestination();
        void setLArmDestination(arma::vec3 inLArm);

        arma::vec3 getRArmPosition();
        void setRArmPosition(arma::vec3 inRArm);
        arma::vec3 getRArmSource();
        void setRArmSource(arma::vec3 inRArm);
        arma::vec3 getRArmDestination();
        void setRArmDestination(arma::vec3 inRArm);


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

        message::motion::kinematics::KinematicsModel kinematicsModel;


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
        // end config

        double STAND_SCRIPT_DURATION;
        ReactionHandle generateStandScriptReaction;

        void generateAndSaveStandScript(const Sensors& sensors);
        void configure(const YAML::Node& config);

        void reset();
        void start();
        void requestStop();
        void stop();

        std::unique_ptr<std::vector<ServoCommand>> updateWaypoints(const Sensors& sensors);

        void setVelocity(Transform2D velocity);
        void updateVelocity();

        void localise(Transform2D position);

        std::unique_ptr<std::vector<ServoCommand>> motionLegs(std::vector<std::pair<ServoID, float>> joints);
        std::unique_ptr<std::vector<ServoCommand>> motionArms(double phase);

        /**
         * @return The current velocity
         */
        Transform2D getVelocity();

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

        /**
         * @return get a unix timestamp (in decimal seconds that are accurate to the microsecond)
         */
        double getTime();

        /**
         * @return A clamped between 0 and maxvalue, offset by deadband
         */
        double linearInterpolationDeadband(double a, double deadband, double maxvalue);
    };

}  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

