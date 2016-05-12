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
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
/*=======================================================================================================*/
/*      NUCLEAR METHOD: FootPlacementPlanner
/*=======================================================================================================*/
    BalanceKinematicResponse::BalanceKinematicResponse(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        //Configure foot motion planner...
        on<Configuration>("BalanceKinematicResponse.yaml").then("Balance Response Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Balance Response Planner - Update Robot Posture", [this](const Sensors& sensors) 
        {
            //hipCompensation();
            //supportMassCompensation();
        }).disable();

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<FootMotionUpdate>>().then("Balance Response Planner - Received Update (Active Foot Position) Info", [this] 
        {
            
        });

        //Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<TorsoMotionUpdate>>().then("Balance Response Planner - Received Update (Active Torso Position) Info", [this] 
        {
            
        });

        /*Aim to avoid dependancy on target position to enhance statelessness and adaptive balance compensation...
        on<Trigger<HeadMotionUpdate>>().then("Balance Response Planner - Received Update (Active Head Position) Info", [this] 
        {
            
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
/*      METHOD: hipCompensation
/*=======================================================================================================*/
	void BalanceKinematicResponse::hipCompensation(const Sensors& sensors, arma::vec3 footPhases, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
            //Evaluate scaled minimum distance of y(=1) phase position to the range [0,1] for hip roll parameter compensation... 
            double yBoundedMinimumPhase = std::min({1.0, footPhases[1] / 0.1, (1 - footPhases[1]) / 0.1});

            //Rotate foot around hip by the given hip roll compensation...
            if (swingLeg == LimbID::LEFT_LEG) 
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
/*      METHOD: supportMassCompensation
/*=======================================================================================================*/
    void BalanceKinematicResponse::supportMassCompensation(const Sensors& sensors, LimbID swingLeg, Transform3D rightFootT, Transform3D leftFootT) 
    {
        //If feature enabled, apply balance compensation through support actuator...
        if (balanceEnabled) 
        {
        	//Apply balance transformation to stipulated support actuator...
            balancer.balance(swingLeg == LimbID::LEFT_LEG ? rightFootT : leftFootT
                           , swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG
                           , sensors);
        }
    }  
/*=======================================================================================================*/
//      METHOD: updateLowerBody
/*=======================================================================================================*/
    std::pair<Transform3D, Transform3D> BalanceKinematicResponse::updateLowerBody(double phase, auto torsoWorld, const Transform2D& leftFootLocal, const Transform2D& rightFootLocal) 
    {
        // Transform feet targets to be relative to the robot torso...
        Transform3D leftFootTorso  =  leftFootLocal.worldToLocal(torsoWorld);
        Transform3D rightFootTorso = rightFootLocal.worldToLocal(torsoWorld);

        //DEBUGGING: Emit relative feet position with respect to robot torso model... 
        if (emitFootPosition)
        {
            emit(graph("Right foot position", rightFootTorso.translation()));
            emit(graph("Left  foot position",  leftFootTorso.translation()));
        }

        return {leftFootTorso, rightFootTorso};
    }
/*=======================================================================================================*/
//      NAME: updateUpperBody
/*=======================================================================================================*/
    void BalanceKinematicResponse::updateUpperBody(double phase, const Sensors& sensors) 
    {
        /*
        //BEGIN: Update Position of Arms
            // Converts the phase into a sine wave that oscillates between 0 and 1 with a period of 2 phases
            double easing = std::sin(M_PI * phase - M_PI / 2.0) / 2.0 + 0.5;
            if (swingLeg == LimbID::LEFT_LEG) 
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
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getDestinationTime
/*=======================================================================================================*/
    double BalanceKinematicResponse::getDestinationTime()
    {
        return (destinationTime);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setDestinationTime
/*=======================================================================================================*/
    void BalanceKinematicResponse::setDestinationTime(double inDestinationTime)
    {
        destinationTime = inDestinationTime;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getMotionPhase
/*=======================================================================================================*/    
    double BalanceKinematicResponse::getMotionPhase()
    {
        return (footMotionPhase);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setMotionPhase
/*=======================================================================================================*/
    void BalanceKinematicResponse::setMotionPhase(double inMotionPhase)  
    {
        footMotionPhase = inMotionPhase;
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
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D BalanceKinematicResponse::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionLegs
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionArms
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPosition3D
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoSource()
    {
        return (torsoPositionSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoSource
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoDestination
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setSupportMass
/*=======================================================================================================*/
    void BalanceKinematicResponse::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootPosition
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootPosition
/*=======================================================================================================*/
    void BalanceKinematicResponse::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootSource
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
        setNewStepReceived(false);
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
//      METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        setNewStepReceived(true);
        leftFootDestination.push(inLeftFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
    Transform2D BalanceKinematicResponse::getRightFootDestination()
    {
        setNewStepReceived(false);
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void BalanceKinematicResponse::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        setNewStepReceived(true);
        rightFootDestination.push(inRightFootDestination);
    }    
/*=======================================================================================================*/
/*      METHOD: configure
/*=======================================================================================================*/
    void BalanceKinematicResponse::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArmStart = stance["arms"]["left"]["start"].as<arma::vec>();
        qLArmEnd = stance["arms"]["left"]["end"].as<arma::vec>();
        qRArmStart = stance["arms"]["right"]["start"].as<arma::vec>();
        qRArmEnd = stance["arms"]["right"]["end"].as<arma::vec>();
        //setFootOffsetCoefficient(stance["foot_offset"].as<arma::vec>());
        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();

        auto& gains = stance["gains"];
        gainArms = gains["arms"].as<Expression>();
        gainLegs = gains["legs"].as<Expression>();

        for(ServoID i = ServoID(0); i < ServoID::NUMBER_OF_SERVOS; i = ServoID(int(i)+1))
        {
            if(int(i) < 6)
            {
                jointGains[i] = gainArms;
            } 
            else 
            {
                jointGains[i] = gainLegs;
            }
        }

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

        for(auto& gain : balance["servo_gains"])
        {
            float p = gain["p"].as<Expression>();
            ServoID sr = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::RIGHT);
            ServoID sl = message::input::idFromPartialString(gain["id"].as<std::string>(),message::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }
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
        STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }          
}  // motion
}  // modules   
