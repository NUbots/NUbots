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

#ifndef MODULE_MOTION_TORSOMOTIONPLANNER_H
#define MODULE_MOTION_TORSOMOTIONPLANNER_H

#include <nuclear>
#include <armadillo>
#include <algorithm>
#include <chrono>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include "message/support/Configuration.h"
#include "message/behaviour/Action.h"
#include "message/behaviour/ServoCommand.h"
#include "message/motion/WalkCommand.h"
#include "message/motion/FootPlacementCommand.h" 
#include "message/motion/TorsoMotionCommand.h" 
#include "message/input/Sensors.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/Script.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/input/PushDetection.h"

#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/Balance.h"
#include "utility/motion/RobotModels.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/math/angle.h"
#include "utility/math/matrix/Rotation3D.h"

namespace module 
{
namespace motion 
{

    class TorsoMotionPlanner : public NUClear::Reactor {

    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;
        static constexpr const char* CONFIGURATION_PATH = "TorsoMotionPlanner.yaml";
        static constexpr const char* CONFIGURATION_MSSG = "Torso Motion Planner - Configure";
        static constexpr const char* ONTRIGGER_TORSO_CMD = "Torso Motion Planner - Update Torso Position";
        static constexpr const char* ONTRIGGER_TORSO_TGT = "Torso Motion Planner - Received Target Torso Position";
        explicit TorsoMotionPlanner(std::unique_ptr<NUClear::Environment> environment);
    private:
        using LimbID         = message::input::LimbID;
        using ServoCommand   = message::behaviour::ServoCommand;
        using Sensors        = message::input::Sensors;
        using ServoID        = message::input::ServoID;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
        ReactionHandle updateHandle;

        // start state

        // The state of the current walk
        State state;


        struct torso {
            Transform2D uTorso;
            Transform3D torso;
        } torsos;

        // // Whether subsumption has currently interrupted the walk engine
        // bool interrupted;
        // TODO: ???
<<<<<<< 183df72fb88459adef7436f0515b768fde100df7:module/motion/ModularWalkEngine/src/TorsoMotionPlanner.h
<<<<<<< 64307f428050a70febbcdd787f038b25be0bf9ed

=======
=======
>>>>>>> Further Modularization in development hierarchy, reorganised directory structure:module/motion/TorsoMotionPlanner/src/TorsoMotionPlanner.h
        bool startFromStep;
        // The time when the current step begun
        double beginStepTime;
        // Update to step is received
        bool updateStepInstruction;
        // The time when the current is to be completed
        double destinationTime;
        // Destination placement Transform2D values
        std::queue<Transform2D> leftFootDestination
        std::queue<Transform2D> rightFootDestination
        // How to many 'steps' to take before lifting a foot when starting to walk
        int initialStep;
<<<<<<< 183df72fb88459adef7436f0515b768fde100df7:module/motion/ModularWalkEngine/src/TorsoMotionPlanner.h
>>>>>>> Modifications to TorsoMotionPlanner
=======
>>>>>>> Further Modularization in development hierarchy, reorganised directory structure:module/motion/TorsoMotionPlanner/src/TorsoMotionPlanner.h
        // Current torso position
        //Transform2D uTorso;
        // Pre-step torso position
        Transform2D uTorsoSource;
        // Torso step target position
        Transform2D uTorsoDestination;
       // Pre-step left foot position
        Transform2D uLeftFootSource;
        // Left foot step target position
        Transform2D uLeftFootDestination;
        // Pre-step right foot position
        Transform2D uRightFootSource;
        // Right foot step target position
        Transform2D uRightFootDestination;
        // TODO: ??? Appears to be support foot pre-step position
        Transform2D uSupport;
        // Current robot velocity
        arma::vec4 zmpCoefficients;
        // zmp params m1X, m2X, m1Y, m2Y
        arma::vec4 zmpParams;
         // end state

        // start config, see config file for documentation
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

        Transform2D stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor);

        /**
         * @return The current velocity
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
        
        double getTime();

        /**
         * @return A clamped between 0 and maxvalue, offset by deadband
         */
    };

}  // motion
}  // modules

#endif  // MODULE_MOTION_TORSOMOTIONPLANNER_H
