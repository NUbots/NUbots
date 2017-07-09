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

#include <algorithm>
#include <armadillo>
#include <chrono>
#include <cmath>
#include <nuclear>

#include <yaml-cpp/yaml.h>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/FixedWalkCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/localisation/FieldObject.h"
#include "message/motion/FootMotionCommand.h"
#include "message/motion/FootPlacementCommand.h"
#include "message/motion/KinematicsModels.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/TorsoMotionCommand.h"
#include "message/motion/WalkCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/Balance.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace motion {

    class TorsoMotionPlanner : public NUClear::Reactor {
    public:
        /**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;

        static constexpr const char* CONFIGURATION_PATH  = "TorsoMotionPlanner.yaml";
        static constexpr const char* CONFIGURATION_MSSG  = "Torso Motion Planner - Configure";
        static constexpr const char* ONTRIGGER_TORSO_CMD = "Torso Motion Planner - Update Torso Position";
        static constexpr const char* ONTRIGGER_TORSO_TGT = "Torso Motion Planner - Received Target Torso Position";
        static constexpr const char* ONTRIGGER_FOOT_INFO = "Torso Motion Planner - Received Foot Motion Update";

        explicit TorsoMotionPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        using ServoCommand   = message::behaviour::ServoCommand;
        using LimbID         = utility::input::LimbID;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        /**
         * Temporary debugging variables for local output logging...
         */
        bool DEBUG;      //
        int DEBUG_ITER;  //

        /**
         * NUsight feedback initialized from configuration script, see config file for documentation...
         */

        /**
         * Resource abstractions for id and handler instances...
         */
        ReactionHandle
            updateHandle;  // handle(updateWaypoints), disabling when not moving will save unnecessary CPU resources
        ReactionHandle generateStandScriptReaction;  // handle(generateStandAndSaveScript), disabling when not required
                                                     // for capturing standing phase

        /**
         * Decision abstractions and notify variables...
         */
        bool updateStepInstruction;  // Update to step is received

        /**
         * Anthropomorphic metrics for relevant humanoid joints & actuators...
         */
        struct TorsoPositions  // Active torso relative positions struct
        {
            TorsoPositions() : FrameArms(), FrameLegs(), Frame3D() {
                FrameArms = Transform2D();
                FrameLegs = Transform2D();
                Frame3D   = Transform3D();
            }
            ~TorsoPositions() {}

            Transform2D FrameArms;
            Transform2D FrameLegs;
            Transform3D Frame3D;
        };
        TorsoPositions torsoPositionsTransform;  // Active torso position
        Transform2D torsoPositionSource;         // Pre-step torso position
        Transform2D torsoPositionDestination;    // Torso step target position
        Transform2D leftFootPositionTransform;   // Active left foot position
        Transform2D rightFootPositionTransform;  // Active right foot position
        Transform2D leftFootSource;              // Pre-step left foot position
        Transform2D rightFootSource;             // Pre-step right foot position
        Transform2D leftFootDestination;         // Destination placement Transform2D left foot positions
        Transform2D rightFootDestination;        // Destination placement Transform2D right foot positions
        Transform2D m_supportMass;               // Appears to be support foot pre-step position
        LimbID activeForwardLimb;                // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID activeLimbInitial;  // TODO: Former initial non-support leg for deterministic walking approach

        /**
         * Anthropomorphic metrics initialized from configuration script, see config file for documentation...
         */
        double bodyTilt;                    //
        double bodyHeight;                  //
        double stanceLimitY2;               //
        double stepTime;                    //
        double stepHeight;                  //
        float step_height_slow_fraction;    //
        float step_height_fast_fraction;    //
        arma::mat::fixed<3, 2> stepLimits;  //
        arma::vec2 footOffsetCoefficient;   //
        Transform2D uLRFootOffset;          // standard offset

        /**
         * Arm Position vectors initialized from configuration script, see config file for documentation...
         */
        arma::vec3 armLPostureTransform;    //
        arma::vec3 armLPostureSource;       //
        arma::vec3 armLPostureDestination;  //
        arma::vec3 armRPostureTransform;    //
        arma::vec3 armRPostureSource;       //
        arma::vec3 armRPostureDestination;  //

        /**
         * Internal timing reference variables...
         */
        double beginStepTime;                               // The time when the current step begun
        double footMotionPhase;                             // Phase representation of foot motion state
        double STAND_SCRIPT_DURATION;                       //
        NUClear::clock::time_point pushTime;                //
        NUClear::clock::time_point lastVeloctiyUpdateTime;  //

        /**
         * Motion data for relevant humanoid actuators...
         */
        double velocityHigh;                    //
        double accelerationTurningFactor;       //
        arma::mat::fixed<3, 2> velocityLimits;  //
        arma::vec3 accelerationLimits;          //
        arma::vec3 accelerationLimitsHigh;      //
        Transform2D velocityCurrent;            // Current robot velocity
        Transform2D velocityCommand;            // Current velocity command

        /**
         * Motion data initialized from configuration script, see config file for documentation...
         */
        //  double velFastForward;                          //
        //  double velFastTurn;                             //

        /**
         * Dynamic analysis parameters for relevant motion planning...
         */
        arma::vec4 zmpCoefficients;  // zmp expoential coefficients aXP aXN aYP aYN
        arma::vec4 zmpParameters;    // zmp params m1X, m2X, m1Y, m2Y

        /**
         * Dynamic analysis parameters initialized from configuration script, see config file for documentation...
         */
        double zmpTime;       //
        double phase1Single;  //
        double phase2Single;  //

        /**
         * Balance & Kinematics module initialization...
         */
        message::motion::KinematicsModel kinematicsModel;  //

        /**
         * The last foot goal rotation...
         */
        UnitQuaternion lastFootGoalRotation;  //
        UnitQuaternion footGoalErrorSum;      //

        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void updateTorsoPosition();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor);
        /**
         * @brief [Solve the ZMP equation]
         * @details [long description]
         * @return [description]
         */
        arma::vec2 zmpSolve(double zs,
                            double z1,
                            double z2,
                            double x1,
                            double x2,
                            double phase1Single,
                            double phase2Single,
                            double stepTime,
                            double zmpTime);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        arma::vec4 zmpTorsoCoefficients();
        /**
         * Uses ZMP to determine the torso position
         *
         * @return The torso position in Transform2D
         */
        Transform2D zmpTorsoCompensation(double phase,
                                         arma::vec4 zmpCoefficients,
                                         arma::vec4 zmpParams,
                                         double stepTime,
                                         double zmpTime,
                                         double phase1Zmp,
                                         double phase2Zmp,
                                         Transform2D uLeftFootSource,
                                         Transform2D uRightFootSource);
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
         * @return [description]
         */
        double getMotionPhase();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void setMotionPhase(double inMotionPhase);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        arma::vec4 getZmpParams();
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void setZmpParams(arma::vec4 inZmpParams);
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

}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_TORSOMOTIONPLANNER_H
