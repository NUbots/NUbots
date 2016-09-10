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
/*===========================================================================================================*/
/*----------------------------------------CONSTANTS AND DEFINITIONS------------------------------------------*/
/*===========================================================================================================*/
//      INCLUDE(S)
/*===========================================================================================================*/
#include "BalanceKinematicResponse.h"
/*===========================================================================================================*/
//      NAMESPACE(S)
/*===========================================================================================================*/
namespace module 
{
namespace motion 
{
/*=======================================================================================================*/
//      UTILIZATION REFERENCE(S)
/*=======================================================================================================*/
    using message::input::LimbID;
    using message::motion::FootMotionUpdate;
    using message::motion::TorsoMotionUpdate;
    using message::motion::EnableBalanceResponse;
    using message::motion::DisableBalanceResponse;
    using message::support::Configuration;

    using utility::support::Expression;
    using message::motion::kinematics::KinematicsModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
/*=======================================================================================================*/
//      NUCLEAR METHOD: FootPlacementPlanner
/*=======================================================================================================*/
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
        , DEBUG(false), DEBUG_ITER(0), initialStep(0)
        , balanceEnabled(0.0), emitLocalisation(false), emitFootPosition(false)
        , updateHandle(), generateStandScriptReaction(), subsumptionId(1)
        , torsoPositionsTransform(), torsoPositionSource(), torsoPositionDestination()
        , leftFootPositionTransform(), leftFootSource(), rightFootPositionTransform()
        , rightFootSource(), leftFootDestination(), rightFootDestination(), uSupportMass()
        , activeForwardLimb(), activeLimbInitial(LimbID::LEFT_LEG)
        , bodyTilt(0.0), bodyHeight(0.0), stanceLimitY2(0.0), stepTime(0.0), stepHeight(0.0)
        , step_height_slow_fraction(0.0f), step_height_fast_fraction(0.0f)
        , stepLimits(arma::fill::zeros), footOffsetCoefficient(arma::fill::zeros), uLRFootOffset()
        , armLPostureTransform(), armLPostureSource(), armLPostureDestination()
        , armRPostureTransform(), armRPostureSource(), armRPostureDestination()
        , beginStepTime(0.0)
        , STAND_SCRIPT_DURATION(0.0), pushTime(), lastVeloctiyUpdateTime()
        , velocityHigh(0.0), accelerationTurningFactor(0.0), velocityLimits(arma::fill::zeros)
        , accelerationLimits(arma::fill::zeros), accelerationLimitsHigh(arma::fill::zeros)
        , velocityCurrent(), velocityCommand()
        , zmpCoefficients(arma::fill::zeros), zmpParameters(arma::fill::zeros)
        , zmpTime(0.0), phase1Single(0.0), phase2Single(0.0)
        , toeTipCompensation(), hipRollCompensation()
        , balancer(), kinematicsModel()
        , balanceAmplitude(0.0), balanceWeight(0.0), balanceOffset(0.0)
        , balancePGain(0.0), balanceIGain(0.0), balanceDGain(0.0)
        , lastFootGoalRotation(), footGoalErrorSum()       
    {

        //Configure balance kinematic response...
        on<Configuration>("BalanceKinematicResponse.yaml").then("Balance Response Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        updateHandle = on<Every<1 /*RESTORE AFTER DEBUGGING: UPDATE_FREQUENCY*/, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Balance Response Planner - Update Robot Posture", [this] /*(const Sensors& sensors)*/
        {
            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Update Robot Posture(0)"); }
            //hipCompensation();
            //supportMassCompensation();
            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Update Robot Posture(1)"); }
        });//RESTORE AFTER DEBUGGING: .disable();

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<FootMotionUpdate>>().then("Balance Response Planner - Received Update (Active Foot Position) Info", [this] 
        {
            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Foot Position) Info(0)"); }

            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Foot Position) Info(1)"); }
        });

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<TorsoMotionUpdate>>().then("Balance Response Planner - Received Update (Active Torso Position) Info", [this] 
        {
            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Torso Position) Info(0)"); }

            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Torso Position) Info(1)"); }
        });

        /*Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<HeadMotionUpdate>>().then("Balance Response Planner - Received Update (Active Head Position) Info", [this] 
        {
            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(0)"); }

            if(DEBUG) { NUClear::log("Messaging: Balance Kinematic Response - Received Update (Active Head Position) Info(1)"); }
        });*/

        //on<Trigger<NewStepTargetInfo>>().then([this]){};            

        on<Trigger<EnableBalanceResponse>>().then([this] (const EnableBalanceResponse& command) 
        {
            subsumptionId = command.subsumptionId;
            updateHandle.enable();
        });

        //If balance response no longer requested, cease updating...
        on<Trigger<DisableBalanceResponse>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
/*=======================================================================================================*/
//      METHOD: hipCompensation
/*=======================================================================================================*/
	void BalanceKinematicResponse::hipCompensation(const Sensors& sensors, arma::vec3 footPhases, LimbID activeForwardLimb, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
            //Evaluate scaled minimum distance of y(=1) phase position to the range [0,1] for hip roll parameter compensation... 
            double yBoundedMinimumPhase = std::min({1.0, footPhases[1] / 0.1, (1 - footPhases[1]) / 0.1});

            //Rotate foot around hip by the given hip roll compensation...
            if (activeForwardLimb == LimbID::LEFT_LEG) 
            {
                rightFootT = rightFootT.rotateZLocal(-hipRollCompensation * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::R_HIP_ROLL)->second);
            }
            else 
            {
                leftFootT  = leftFootT.rotateZLocal( hipRollCompensation  * yBoundedMinimumPhase, sensors.forwardKinematics.find(ServoID::L_HIP_ROLL)->second);
            }
        }
    }
/*=======================================================================================================*/
//      METHOD: supportMassCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::supportMassCompensation(const Sensors& sensors, LimbID activeForwardLimb, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
        	//Apply balance transformation to stipulated support actuator...
            balancer.balance(kinematicsModel, activeForwardLimb == LimbID::LEFT_LEG ? rightFootT : leftFootT
                           , activeForwardLimb == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG
                           , sensors);
        }
    }  
/*=======================================================================================================*/
//      METHOD: updateLowerBody
/*=======================================================================================================*/
    std::pair<Transform3D, Transform3D> BalanceKinematicResponse::updateLowerBody(const Transform2D& torsoWorld, const Transform2D& leftFootLocal, const Transform2D& rightFootLocal) 
    {
        // Transform feet targets to be relative to the robot torso...
        Transform3D leftFootTorso  =  leftFootLocal.worldToLocal(torsoWorld);
        Transform3D rightFootTorso = rightFootLocal.worldToLocal(torsoWorld);

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            //emit(graph("Right foot position", rightFootTorso.translation()));
            //emit(graph("Left  foot position",  leftFootTorso.translation()));
        }

        return {leftFootTorso, rightFootTorso};
    }
/*=======================================================================================================*/
//      NAME: updateUpperBody
/*=======================================================================================================*/
    void BalanceKinematicResponse::updateUpperBody(/*const Sensors& sensors*/) 
    {
        /*
        //BEGIN: Update Position of Arms
            // Converts the phase into a sine wave that oscillates between 0 and 1 with a period of 2 phases
            double easing = std::sin(M_PI * phase - M_PI / 2.0) / 2.0 + 0.5;
            if (activeForwardLimb == LimbID::LEFT_LEG) 
            {
                easing = -easing + 1.0; // Gets the 2nd half of the sine wave
            }

            // Linearly interpolate between the start and end positions using the easing parameter
            arma::vec3 qLArmActual = easing * qLArmStart + (1.0 - easing) * qLArmEnd;
            arma::vec3 qRArmActual = (1.0 - easing) * qRArmStart + easing * qRArmEnd;

            // Start arm/leg collision/prevention
            double rotLeftA = normalizeAngle(uLeftFoot.angle() - uTorso.angle());
            double rotRightA = normalizeAngle(uTorso.angle() - uRightFoot.angle());
            Transform2D leftLegTorso = uTorso.worldToLocal(uLeftFoot);
            Transform2D rightLegTorso = uTorso.worldToLocal(uRightFoot);
            double leftMinValue = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2 + std::max(0.0, leftLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
            double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2 - std::max(0.0, -rightLegTorso.y() - 0.04) / 0.02 * (6 * M_PI / 180);
            // update shoulder pitch to move arm away from body
            qLArmActual[1] = std::max(leftMinValue, qLArmActual[1]);
            qRArmActual[1] = std::min(rightMinValue, qRArmActual[1]);
            // End arm/leg collision/prevention
        //END: Update Position of Arms  
        */

        /*
        //DEBUGGING: Emit relative torsoWorldMetrics position with respect to world model... 
        if (emitLocalisation) 
        {
            localise(uTorsoWorld);
        }
        */

        //emit(motionArms(phase));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    double BalanceKinematicResponse::getTime() 
    {
        if(DEBUG) { printf("System Time:%f\n\r", double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den))); }
        return (double(NUClear::clock::now().time_since_epoch().count()) * (1.0 / double(NUClear::clock::period::den)));
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getZmpParams
/*=======================================================================================================*/    
    arma::vec4 BalanceKinematicResponse::getZmpParams()
    {
        setZmpParams
        ({
            (getSupportMass().x() - getTorsoPositionArms().x()) / (stepTime * phase1Single),
            (getTorsoDestination().x() - getSupportMass().x()) / (stepTime * (1 - phase2Single)),
            (getSupportMass().y() - getTorsoPositionArms().y()) / (stepTime * phase1Single),
            (getTorsoDestination().y() - getSupportMass().y()) / (stepTime * (1 - phase2Single)),
        });
        return (zmpParameters);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setZmpParams
/*=======================================================================================================*/
    void BalanceKinematicResponse::setZmpParams(arma::vec4 inZmpParams)
    {
        zmpParameters = inZmpParams;
    }       
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmPosition
/*=======================================================================================================*/    
    arma::vec3 BalanceKinematicResponse::getLArmPosition()
    {
        return (armLPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmPosition
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setLArmPosition(arma::vec3 inLArm)
    {
        armLPostureTransform = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmSource
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getLArmSource()
    {
        return (armLPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmSource
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setLArmSource(arma::vec3 inLArm)
    {
        armLPostureSource = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLArmDestination
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getLArmDestination()
    {
        return (armLPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLArmDestination
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setLArmDestination(arma::vec3 inLArm)
    {
        armLPostureDestination = inLArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmPosition
/*=======================================================================================================*/ 
    arma::vec3 BalanceKinematicResponse::getRArmPosition()
    {
        return (armRPostureTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmPosition
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setRArmPosition(arma::vec3 inRArm)
    {
        armRPostureTransform = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmSource
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getRArmSource()
    {
        return (armRPostureSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmSource
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setRArmSource(arma::vec3 inRArm)
    {
        armRPostureSource = inRArm;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRArmDestination
/*=======================================================================================================*/     
    arma::vec3 BalanceKinematicResponse::getRArmDestination()
    {
        return (armRPostureDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRArmDestination
/*=======================================================================================================*/     
    void BalanceKinematicResponse::setRArmDestination(arma::vec3 inRArm)
    {
        armRPostureDestination = inRArm;
    }      
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D BalanceKinematicResponse::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setTorsoPositionLegs
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setTorsoPositionArms
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setTorsoPosition3D
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoSource()
    {
        return (torsoPositionSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setTorsoSource
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTorsoDestination
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setTorsoDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setSupportMass
/*=======================================================================================================*/
    void BalanceKinematicResponse::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLeftFootPosition
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootPosition
/*=======================================================================================================*/
    void BalanceKinematicResponse::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootSource
/*=======================================================================================================*/
    void BalanceKinematicResponse::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }        
/*=======================================================================================================*/
//      METHOD: getLeftFootDestination
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootDestination()
    {
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
//      METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        leftFootDestination.push(inLeftFootDestination);
    }

//      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootDestination()
    {
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        rightFootDestination.push(inRightFootDestination);
    }    
/*=======================================================================================================*/
//      METHOD: configure
/*=======================================================================================================*/
    void BalanceKinematicResponse::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        setLArmSource(stance["arms"]["left"]["start"].as<arma::vec>());
        setLArmDestination(stance["arms"]["left"]["end"].as<arma::vec>());
        setRArmSource(stance["arms"]["right"]["start"].as<arma::vec>());
        setRArmDestination(stance["arms"]["right"]["end"].as<arma::vec>());
        //setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());
        // gToe/heel overlap checking values
        stanceLimitY2 = kinematicsModel.Leg.LENGTH_BETWEEN_LEGS() - stance["limit_margin_y"].as<Expression>();

        auto& walkCycle = config["walk_cycle"];
        stepTime = walkCycle["step_time"].as<Expression>();
        zmpTime = walkCycle["zmp_time"].as<Expression>();
        hipRollCompensation = walkCycle["hip_roll_compensation"].as<Expression>();
        stepHeight = walkCycle["step"]["height"].as<Expression>();
        stepLimits = walkCycle["step"]["limits"].as<arma::mat::fixed<3,2>>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<arma::mat::fixed<3,2>>();
        velocityHigh = velocity["high_speed"].as<Expression>();

        auto& acceleration = walkCycle["acceleration"];
        accelerationLimits = acceleration["limits"].as<arma::vec>();
        accelerationLimitsHigh = acceleration["limits_high"].as<arma::vec>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        auto& balance = walkCycle["balance"];
        balanceEnabled = balance["enabled"].as<bool>();
        // balanceAmplitude = balance["amplitude"].as<Expression>();
        // balanceWeight = balance["weight"].as<Expression>();
        // balanceOffset = balance["offset"].as<Expression>();

        balancer.configure(balance);

        
        /* TODO
        // gCompensation parameters
        toeTipCompensation = config["toeTipCompensation"].as<Expression>();
        ankleMod = {-toeTipCompensation, 0};

        // gGyro stabilization parameters
        ankleImuParamX = config["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY = config["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX = config["kneeImuParamX"].as<arma::vec>();
        hipImuParamY = config["hipImuParamY"].as<arma::vec>();
        armImuParamX = config["armImuParamX"].as<arma::vec>();
        armImuParamY = config["armImuParamY"].as<arma::vec>();

        // gSupport bias parameters to reduce backlash-based instability
        velFastForward = config["velFastForward"].as<Expression>();
        velFastTurn = config["velFastTurn"].as<Expression>();
        supportFront = config["supportFront"].as<Expression>();
        supportFront2 = config["supportFront2"].as<Expression>();
        supportBack = config["supportBack"].as<Expression>();
        supportSideX = config["supportSideX"].as<Expression>();
        supportSideY = config["supportSideY"].as<Expression>();
        supportTurn = config["supportTurn"].as<Expression>();
        */
    }          
}  // motion
}  // modules   
