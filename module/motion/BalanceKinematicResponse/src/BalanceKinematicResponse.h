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

#ifndef MODULE_MOTION_BALANCEKINEMATICRESPONSE_H
#define MODULE_MOTION_BALANCEKINEMATICRESPONSE_H

#include <nuclear>
#include <armadillo>

#include <yaml-cpp/yaml.h>

#include "extension/Configuration.h"

#include "message/behaviour/proto/FixedWalkCommand.h"
#include "message/behaviour/proto/ServoCommand.h"
#include "message/behaviour/proto/Subsumption.h"

#include "message/input/proto/Sensors.h"
#include "message/input/proto/PushDetection.h"
#include "message/input/proto/PostureRecognition.h"

#include "message/motion/proto/KinematicsModels.h"
#include "message/motion/proto/WalkCommand.h"
#include "message/motion/proto/FootMotionCommand.h" 
#include "message/motion/proto/TorsoMotionCommand.h"  
#include "message/motion/proto/BalanceCommand.h" 
#include "message/motion/proto/HeadCommand.h" 
#include "message/motion/proto/ServoTarget.h"

#include "message/localisation/proto/FieldObject.h"

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

#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module 
{
namespace motion 
{
    class BalanceKinematicResponse : public NUClear::Reactor 
    {
    public:
    	/**
         * The number of servo updates performnced per second
         * TODO: Probably be a global config somewhere, waiting on NUClear to support runtime on<Every> arguments
         */
        static constexpr size_t UPDATE_FREQUENCY = 90;

        static constexpr const char* CONFIGURATION_PATH = "BalanceKinematicResponse.yaml";
        static constexpr const char* CONFIGURATION_MSSG = "Balance Response Planner - Configure";
        static constexpr const char* ONTRIGGER_FTMN_INF = "Balance Response Planner - Received Update (Active Foot Position) Info";
        static constexpr const char* ONTRIGGER_TRSM_INF = "Balance Response Planner - Received Update (Active Torso Position) Info";
        static constexpr const char* ONTRIGGER_HEAD_INF = "Balance Response Planner - Received Update (Active Head Position) Info";
        static constexpr const char* ONTRIGGER_BLNC_CMD = "Balance Response Planner - Update Robot Posture";

        explicit BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment);
    private:
        using LimbID         = message::behaviour::proto::Subsumption::Limb::Value;
        using ServoCommand   = message::behaviour::proto::ServoCommand;
        using Sensors        = message::input::proto::Sensors;
        using ServoID        = message::input::proto::Sensors::ServoID::Value;
        using Transform2D    = utility::math::matrix::Transform2D;
        using Transform3D    = utility::math::matrix::Transform3D;
        using UnitQuaternion = utility::math::geometry::UnitQuaternion;

        /**
         * Temporary debugging variables for local output logging...
         */ 
        bool DEBUG;                         //
        int  DEBUG_ITER;                    //
        int  initialStep;                   // TODO: How to many 'steps' to take before lifting a foot when starting to walk

        /**
         * NUsight feedback initialized from configuration script, see config file for documentation...
         */
        bool balanceEnabled;                    //
        bool hipRollCompensationEnabled;        //
        bool ankleTorqueCompensationEnabled;    //
        bool armRollCompensationEnabled;        //
        bool toeTipCompensationEnabled;         //
        bool supportCompensationEnabled;        //
        bool balanceOptimiserEnabled;           //
        bool pushRecoveryEnabled;               //
        bool emitLocalisation;                  //
        bool emitFootPosition;                  //
        bool armMotionEnabled;                  // Determines if the upper body can move the arms throughout motion.

        /**
         * Resource abstractions for id and handler instances...
         */
        ReactionHandle updateHandle;                    // handle(updateWaypoints), disabling when not moving will save unnecessary CPU resources
        ReactionHandle updateOptimiser;                 // handle(updateOptimiser), disabling when not required will save unnecessary CPU resources
        ReactionHandle generateStandScriptReaction;     // handle(generateStandAndSaveScript), disabling when not required for capturing standing phase

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
        Transform2D leftFootPosition2D;                 // Transform2D state of left foot position    
        Transform2D rightFootPosition2D;                // Transform2D state of right foot position
        Transform3D leftFootPositionTransform;          // Active left foot position    
        Transform3D rightFootPositionTransform;         // Active right foot position
        Transform2D uSupportMass;                       // Appears to be support foot pre-step position
        LimbID activeForwardLimb;                       // The leg that is 'swinging' in the step, opposite of the support foot
        LimbID activeLimbInitial;                       // TODO: Former initial non-support leg for deterministic walking approach

        /**
         * Anthropomorphic metrics initialized from configuration script, see config file for documentation...
         */
        double bodyTilt;                                // 
        double bodyHeight;                              //
        double supportFront;                            //
        double supportFront2;                           //
        double supportBack;                             //
        double supportSideX;                            //
        double supportSideY;                            //
        double supportTurn;                             //    
        double stanceLimitY2;                           //
        double stepTime;                                //
        double stepHeight;                              //
        float  step_height_slow_fraction;               //
        float  step_height_fast_fraction;               //
        arma::mat::fixed<3,2> stepLimits;               //              
        arma::vec2 footOffsetCoefficient;               //
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
        arma::vec4 ankleImuParamX;                      //
        arma::vec4 ankleImuParamY;                      //
        arma::vec4 kneeImuParamX;                       //
        arma::vec4 hipImuParamY;                        //
        arma::vec4 armImuParamX;                        //
        arma::vec4 armImuParamY;                        //

        /**
         * Internal timing reference variables...
         */
        double beginStepTime;                                   // The time when the current step begun
        double footMotionPhase;                                 // Phase representation of foot motion state
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
        double velFastForward;                          //
        double velFastTurn;                             //   
        
        /**
         * Dynamic analysis parameters for relevant motion planning...
         */

        /**
         * Dynamic analysis parameters initialized from configuration script, see config file for documentation...
         */
        double phase1Single;                            //
        double phase2Single;                            //

        /**
         * Balance & Post-alignment parameters used for humanoid stability techniques...
         */

        double rollParameter;                //
        double pitchParameter;               //
        double yawParameter;                 // 
        double toeTipParameter;              //
        double hipRollParameter;             //
        double armRollParameter;             //
        double hipCompensationScale;         //
        double toeCompensationScale;         //
        double ankleCompensationScale;       //
        double armCompensationScale;         //
        double supportCompensationScale;     //
        double hipCompensationMax;         //
        double toeCompensationMax;         //
        double ankleCompensationMax;       //
        double armCompensationMax;         //
        double supportCompensationMax;     //


        /**
         * Balance & Kinematics module initialization...
         */
        utility::motion::Balancer balancer;                             //
        message::motion::proto::KinematicsModel kinematicsModel;   //

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
         * The last foot goal rotation...
         */
        UnitQuaternion lastFootGoalRotation;            //
        UnitQuaternion footGoalErrorSum;                //

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
         * 
         * @param inTorsoPosition [description]
         */
        void localise(Transform2D position); 
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void updateBodyPushRecovery();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void updateBody(const Sensors& sensors);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void updateLowerBody(const Sensors& sensors);
        /**
         * @brief [brief description]std::pair
         * @details [long description]
         * @return [description]
         */
        void updateUpperBody(const Sensors& sensors);
        /**
         * @brief [brief description]std::pair
         * @details [long description]
         * @return [description]
         */
        void armRollCompensation(const Sensors& sensors);
        /**
         * @brief [brief description]std::pair
         * @details [long description]
         * @return [description]
         */
        void ankleTorqueCompensation(/*const Sensors& sensors*/);
        /**
         * @brief [brief description]std::pair
         * @details [long description]
         * @return [description]
         */
        void toeTipCompensation(/*const Sensors& sensors*/);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void hipRollCompensation(const Sensors& sensors);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        void supportMassCompensation(const Sensors& sensors);
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
         */
        void postureInitialize();  
        /**
         * @brief [brief description]
         * @details [get a unix timestamp (in decimal seconds that are accurate to the microsecond)]
         * @return [description]
         */
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
         */
        double getTime();
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
         * @details [get a unix timestamp (in decimal seconds that are accurate to the microsecond)]
         * @return A clamped value between 0 and maxvalue, offset by deadband
         */
        double linearInterpolationDeadband(double a, double deadband, double maxvalue);
        /**
         * @brief [brief description]
         * @details [long description]
         */
        double getRollParameter();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void setRollParameter(double inRollParameter);
        /**
         * @brief [brief description]
         * @details [long description]
         */
        double getPitchParameter();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void setPitchParameter(double inPitchParameter);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        double getYawParameter();
        /**
         * @brief [brief description]
         * @details [long description]
         */
        void setYawParameter(double inYawParameter);
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inTorsoPosition [description]
         */
        double getHipCompensationScale();

        double getAnkleCompensationScale();

        double getToeCompensationScale();

        double getArmCompensationScale();
        
        double getSupportCompensationScale();

        double getHipCompensationMax();

        double getAnkleCompensationMax();

        double getToeCompensationMax();

        double getArmCompensationMax();
        
        double getSupportCompensationMax();


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
         * @return [description]
         */
        Transform2D getLeftFootPosition2D();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inLeftFootPosition [description]
         */
        void setLeftFootPosition2D(const Transform2D& inLeftFootPosition);
        /**
         * @brief [brief description]
         * @details [long description]
         * @return [description]
         */
        Transform2D getRightFootPosition2D();
        /**
         * @brief [brief description]
         * @details [long description]
         * 
         * @param inRightFootPosition [description]
         */
        void setRightFootPosition2D(const Transform2D& inRightFootPosition);
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

#endif  // MODULE_MOTION_BALANCEKINEMATICRESPONSE_H
