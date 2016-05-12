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
#include "TorsoMotionPlanner.h"
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
    using message::input::PushDetection;
    using message::input::ServoID;
    using message::input::Sensors;
    using message::input::LimbID;
    using message::behaviour::ServoCommand;
    using message::behaviour::WalkOptimiserCommand;
    using message::behaviour::WalkConfigSaved;
    // using message::behaviour::RegisterAction;
    // using message::behaviour::ActionPriorites;
    using message::input::LimbID;
    using message::motion::WalkCommand;
    using message::motion::WalkStartCommand;
    using message::motion::WalkStopCommand;
    using message::motion::WalkStopped;
    using message::motion::FootStepTarget;
    using message::motion::FootMotionUpdate;
    using message::motion::TorsoMotionUpdate;
    using message::motion::EnableTorsoMotion;
    using message::motion::DisableTorsoMotion;
    using message::motion::ServoTarget;
    using message::motion::Script;
    using message::support::SaveConfiguration;
    using message::support::Configuration;

    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::angle::normalizeAngle;
    using utility::nubugger::graph;
    using utility::support::Expression;
/*=======================================================================================================*/
//      NUCLEAR METHOD: TorsoMotionPlanner
/*=======================================================================================================*/
    TorsoMotionPlanner::TorsoMotionPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
    {
        //Configure foot motion planner...
        on<Configuration>("TorsoMotionPlanner.yaml").then("Torso Motion Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Transform analytical torso positions in accordance with the stipulated targets...
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Torso Motion Planner - Update Torso Position", [this](const Sensors& sensors) 
        {
            updateTorsoPosition();
        }).disable();

        //In the event of a new foot step target specified by the foot placement planning module...
        on<Trigger<FootStepTarget>>().then("Torso Motion Planner - Received Target Torso Position", [this] (const FootStepTarget& target) 
        {
            if(target.supportMass == LimbID::LEFT_LEG)
            {
                setLeftFootDestination(target.targetDestination);
            }
            else
            {
                setRightFootDestination(target.targetDestination);
            }
            setDestinationTime(target.targetTime); 
            zmpTorsoCoefficients();
        });

        //In the process of actuating a foot step and emitting updated positional data...
        on<Trigger<FootMotionUpdate>>().then("Torso Motion Planner - Received Foot Motion Update", [this] (const FootMotionUpdate& info) 
        {
            setMotionPhase(info.phase);
        });

        on<Trigger<EnableTorsoMotion>>().then([this] (const EnableTorsoMotion& command) 
        {
            subsumptionId = command.subsumptionId;
            updateHandle.enable();
        });

        //If torso motion no longer requested, cease updating...
        on<Trigger<DisableTorsoMotion>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
/*=======================================================================================================*/
//      METHOD: updateTorsoPosition
/*=======================================================================================================*/
    void TorsoMotionPlanner::updateTorsoPosition()
    {
        setTorsoPositionArms(zmpTorsoCompensation(getMotionPhase(), zmpTorsoCoefficients(), getZmpParams(), stepTime, zmpTime, phase1Single, phase2Single, getLeftFootSource(), getRightFootSource()));
        setTorsoPositionLegs(zmpTorsoCompensation(getMotionPhase(), zmpTorsoCoefficients(), getZmpParams(), stepTime, zmpTime, phase1Single, phase2Single, getLeftFootSource(), getRightFootSource()));
        Transform2D uTorsoWorld = getTorsoPositionArms().localToWorld({-DarwinModel::Leg::HIP_OFFSET_X, 0, 0});
        setTorsoPosition3D(arma::vec6({uTorsoWorld.x(), uTorsoWorld.y(), bodyHeight, 0, bodyTilt, uTorsoWorld.angle()}));
        emit(std::make_unique<TorsoMotionUpdate>(getTorsoPositionArms(), getTorsoPositionLegs(), getTorsoPosition3D())); 
    }
/*=======================================================================================================*/
//      METHOD: stepTorso
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::stepTorso(Transform2D uLeftFoot, Transform2D uRightFoot, double shiftFactor) 
    {
        Transform2D uLeftFootSupport  = uLeftFoot.localToWorld({-footOffset[0], -footOffset[1], 0});
        Transform2D uRightFootSupport = uRightFoot.localToWorld({-footOffset[0], footOffset[1], 0});
        return uLeftFootSupport.interpolate(shiftFactor, uRightFootSupport);
    }
/*=======================================================================================================*/
//      METHOD: zmpTorsoCoefficients
/*=======================================================================================================*/
    arma::vec4 TorsoMotionPlanner::zmpTorsoCoefficients()
    {
        arma::vec4 zmpCoefficients;
        setTorsoDestination(stepTorso(getLeftFootDestination(), getRightFootDestination(), 0.5));
        
        // Compute ZMP coefficients...
        zmpCoefficients.rows(0,1) = zmpSolve(getSupportMass().x(), getTorsoSource().x(), getTorsoDestination().x(), getTorsoSource().x(), getTorsoDestination().x(), phase1Single, phase2Single, stepTime, zmpTime);
        zmpCoefficients.rows(2,3) = zmpSolve(getSupportMass().y(), getTorsoSource().y(), getTorsoDestination().y(), getTorsoSource().y(), getTorsoDestination().y(), phase1Single, phase2Single, stepTime, zmpTime);
        
        return (zmpCoefficients);
    }
/*=======================================================================================================*/
//      METHOD: zmpSolve
/*=======================================================================================================*/
    arma::vec2 TorsoMotionPlanner::zmpSolve(double zs, double z1, double z2, double x1, double x2, double phase1Single, double phase2Single, double stepTime, double zmpTime) 
    {
        /*
        Solves ZMP equations.
        The resulting form of x is
        x(t) = z(t) + aP*exp(t/zmpTime) + aN*exp(-t/zmpTime) - zmpTime*mi*sinh((t-Ti)/zmpTime)
        where the ZMP point is piecewise linear:
        z(0) = z1, z(T1 < t < T2) = zs, z(stepTime) = z2
        */
        double T1 = stepTime * phase1Single;
        double T2 = stepTime * phase2Single;
        double m1 = (zs - z1) / T1;
        double m2 = -(zs - z2) / (stepTime - T2);

        double c1 = x1 - z1 + zmpTime * m1 * std::sinh(-T1 / zmpTime);
        double c2 = x2 - z2 + zmpTime * m2 * std::sinh((stepTime - T2) / zmpTime);
        double expTStep = std::exp(stepTime / zmpTime);
        double aP = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        double aN = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }
/*=======================================================================================================*/
//      METHOD: zmpTorsoCompensation
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::zmpTorsoCompensation(double phase, arma::vec4 zmpTorsoCoefficients, arma::vec4 zmpParams, double stepTime, double zmpTime, double phase1Single, double phase2Single, Transform2D uLeftFootSource, Transform2D uRightFootSource) 
    {
        //Note that phase is the only variable updated during a step
        Transform2D com = {0, 0, 0};
        double expT = std::exp(stepTime * phase / zmpTime);
        com.x() = getSupportMass().x() + zmpTorsoCoefficients[0] * expT + zmpTorsoCoefficients[1] / expT;
        com.y() = getSupportMass().y() + zmpTorsoCoefficients[2] * expT + zmpTorsoCoefficients[3] / expT;
        if (phase < phase1Single) 
        {
            com.x() += zmpParams[0] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[0] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
            com.y() += zmpParams[1] * stepTime * (phase - phase1Single) -zmpTime * zmpParams[1] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
        } 
        else if (phase > phase2Single) 
        {
            com.x() += zmpParams[2] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[2] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
            com.y() += zmpParams[3] * stepTime * (phase - phase2Single) -zmpTime * zmpParams[3] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com.angle() = phase * (getLeftFootDestination().angle() + getRightFootDestination().angle()) / 2 + (1 - phase) * (uLeftFootSource.angle() + uRightFootSource.angle()) / 2;
        return com;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    double TorsoMotionPlanner::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getDestinationTime
/*=======================================================================================================*/
    double TorsoMotionPlanner::getDestinationTime()
    {
        return (destinationTime);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setDestinationTime
/*=======================================================================================================*/
    void TorsoMotionPlanner::setDestinationTime(double inDestinationTime)
    {
        destinationTime = inDestinationTime;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getMotionPhase
/*=======================================================================================================*/    
    double TorsoMotionPlanner::getMotionPhase()
    {
        return (footMotionPhase);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setMotionPhase
/*=======================================================================================================*/
    void TorsoMotionPlanner::setMotionPhase(double inMotionPhase)  
    {
        footMotionPhase = inMotionPhase;
    } 
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getZmpParams
/*=======================================================================================================*/    
    arma::vec4 TorsoMotionPlanner::getZmpParams()
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
    void TorsoMotionPlanner::setZmpParams(arma::vec4 inZmpParams)
    {
        zmpParameters = inZmpParams;
    }       
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getTorsoPositionArms()
    {
        return (torsoPositionsTransform.FrameArms);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getTorsoPositionLegs()
    {
        return (torsoPositionsTransform.FrameLegs);
    }        
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoPosition
/*=======================================================================================================*/
    Transform3D TorsoMotionPlanner::getTorsoPosition3D()
    {
        return (torsoPositionsTransform.Frame3D);
    }            
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionLegs
/*=======================================================================================================*/
    void TorsoMotionPlanner::setTorsoPositionLegs(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameLegs = inTorsoPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPositionArms
/*=======================================================================================================*/
    void TorsoMotionPlanner::setTorsoPositionArms(const Transform2D& inTorsoPosition)
    {
        torsoPositionsTransform.FrameArms = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoPosition3D
/*=======================================================================================================*/
    void TorsoMotionPlanner::setTorsoPosition3D(const Transform3D& inTorsoPosition)
    {
        torsoPositionsTransform.Frame3D = inTorsoPosition;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoSource
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getTorsoSource()
    {
        return (torsoPositionSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoSource
/*=======================================================================================================*/
    void TorsoMotionPlanner::setTorsoSource(const Transform2D& inTorsoSource)
    {
        torsoPositionSource = inTorsoSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getTorsoDestination
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getTorsoDestination()
    {
        return (torsoPositionDestination);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setTorsoDestination
/*=======================================================================================================*/
    void TorsoMotionPlanner::setTorsoDestination(const Transform2D& inTorsoDestination)
    {
        torsoPositionDestination = inTorsoDestination;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getSupportMass
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getSupportMass()
    {
        return (uSupportMass);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setSupportMass
/*=======================================================================================================*/
    void TorsoMotionPlanner::setSupportMass(const Transform2D& inSupportMass)
    {
        uSupportMass = inSupportMass;
    }    
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootPosition
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getLeftFootPosition()
    {
        return (leftFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootPosition
/*=======================================================================================================*/
    void TorsoMotionPlanner::setLeftFootPosition(const Transform2D& inLeftFootPosition)
    {
        leftFootPositionTransform = inLeftFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootPosition
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getRightFootPosition()
    {
        return (rightFootPositionTransform);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootPosition
/*=======================================================================================================*/
    void TorsoMotionPlanner::setRightFootPosition(const Transform2D& inRightFootPosition)
    {
        rightFootPositionTransform = inRightFootPosition;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void TorsoMotionPlanner::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootSource
/*=======================================================================================================*/
    void TorsoMotionPlanner::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }        
/*=======================================================================================================*/
//      METHOD: getLeftFootDestination
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getLeftFootDestination()
    {
        setNewStepReceived(false);
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
//      METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void TorsoMotionPlanner::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        setNewStepReceived(true);
        leftFootDestination.push(inLeftFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
    Transform2D TorsoMotionPlanner::getRightFootDestination()
    {
        setNewStepReceived(false);
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void TorsoMotionPlanner::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        setNewStepReceived(true);
        rightFootDestination.push(inRightFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: isNewStepReceived
/*=======================================================================================================*/
    bool TorsoMotionPlanner::getNewStepReceived()
    {
        return (updateStepInstruction);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setNewStepReceived
/*=======================================================================================================*/
    void TorsoMotionPlanner::setNewStepReceived(bool inUpdateStepInstruction)
    {
        updateStepInstruction = inUpdateStepInstruction;
    }    
/*=======================================================================================================*/
/*      METHOD: configure
/*=======================================================================================================*/
    void TorsoMotionPlanner::configure(const YAML::Node& config)
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
        //stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();

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

