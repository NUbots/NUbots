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
#include "FootMotionPlanner.h"
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
    using message::motion::FootStepTarget;
    using message::motion::FootMotionUpdate;
    using message::motion::EnableFootMotion;
    using message::motion::DisableFootMotion;
    using message::motion::FootStepCompleted;
    using message::support::Configuration;

    using utility::support::Expression;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
/*=======================================================================================================*/
//      NUCLEAR METHOD: FootMotionPlanner
/*=======================================================================================================*/
    FootMotionPlanner::FootMotionPlanner(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) 
    {    	
        //Configure foot motion planner...
        on<Configuration>("FootMotionPlanner.yaml").then("Foot Motion Planner - Configure", [this] (const Configuration& config) 
        {
            configure(config.config);
        });

        //Transform analytical foot positions in accordance with the stipulated targets...
        updateHandle = on<Every<1 /*RESTORE AFTER DEBUGGING: UPDATE_FREQUENCY*/, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
        .then("Foot Motion Planner - Update Foot Position", [this](const Sensors& sensors) 
        {
            if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Update Foot Position(0)"); }
            updateFootPosition(getMotionPhase(), getLeftFootDestination(), getRightFootDestination());
            if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Update Foot Position(1)"); }
        });//RESTORE AFTER DEBUGGING: .disable();

        //In the event of a new foot step target specified by the foot placement planning module...
        on<Trigger<FootStepTarget>>().then("Foot Motion Planner - Received Target Foot Position", [this] (const FootStepTarget& target) 
        {
            if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Received Target Foot Position(0)"); }
            if(target.supportMass == LimbID::LEFT_LEG)
            {
                setLeftFootDestination(target.targetDestination);
            }
            else
            {
                setRightFootDestination(target.targetDestination);
            }
            setDestinationTime(target.targetTime); 
                std::cout << "Destination Time - FMP:" << getDestinationTime() << "\n\r";//debugging
            if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Received Target Foot Position(1)"); }
        });

        //If foot motion is requested, enable updating...
        on<Trigger<EnableFootMotion>>().then([this] (const EnableFootMotion& command) 
        {
            subsumptionId = command.subsumptionId;
            updateHandle.enable();
        });

        //If foot motion no longer requested, cease updating...
        on<Trigger<DisableFootMotion>>().then([this] 
        {
            updateHandle.disable(); 
        });
    }
/*=======================================================================================================*/
//      METHOD: updateFootPosition
/*=======================================================================================================*/
    void FootMotionPlanner::updateFootPosition(double phase, const Transform2D& leftFootDestination, const Transform2D& rightFootDestination) 
    {       
        // Active left foot position
        Transform2D leftFootPositionTransform;
        // Active right foot position
        Transform2D rightFootPositionTransform;
        //Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
        arma::vec3 footPhases = footPhase(phase, phase1Single, phase2Single);

        //Lift foot by amount depending on walk speed
        if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - footPhase limits and calculations"); }
        auto& limit = (velocityCurrent.x() > velocityHigh ? accelerationLimitsHigh : accelerationLimits); // TODO: use a function instead
        float speed = std::min(1.0, std::max(std::abs(velocityCurrent.x() / limit[0]), std::abs(velocityCurrent.y() / limit[1])));
        float scale = (step_height_fast_fraction - step_height_slow_fraction) * speed + step_height_slow_fraction;
        footPhases[2] *= scale;

        if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Interpolate Transform2D"); }
        //Interpolate Transform2D from start to destination - deals with flat resolved movement in (x,y) coordinates
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            //Vector field function??
            rightFootPositionTransform = getRightFootSource().interpolate(footPhases[0], rightFootDestination);
        }
        else
        {
            //Vector field function??
            leftFootPositionTransform  = getLeftFootSource().interpolate(footPhases[0],   leftFootDestination);
        }
        
        if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Instantiate FootLocal Variables"); }
        //Translates foot motion into z dimension for stepping in three-dimensional space...
        Transform3D leftFootLocal  = leftFootPositionTransform;
        Transform3D rightFootLocal = rightFootPositionTransform;

        if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Translate Z for support foot"); }
        //Lift swing leg - manipulate(update) z component of foot position to action movement with a varying altitude locus...
        if (swingLeg == LimbID::RIGHT_LEG) 
        {
            rightFootLocal = rightFootLocal.translateZ(stepHeight * footPhases[2]);
        }
        else
        {
            leftFootLocal  = leftFootLocal.translateZ(stepHeight * footPhases[2]);
        }      

        //DEBUGGING: Emit relative feet position phase with respect to robot state... 
        if (emitFootPosition)
        {
            emit(graph("Foot Phase Motion", phase));
        }

        if(DEBUG) { NUClear::log("Messaging: Foot Motion Planner - Emit FootMotionUpdate"); }
        //Broadcast struct of updated foot motion data at corresponding phase identity...
        emit(std::make_unique<FootMotionUpdate>(phase, leftFootLocal, rightFootLocal));
    }
/*=======================================================================================================*/
//      METHOD: footPhase
/*=======================================================================================================*/
    arma::vec3 FootMotionPlanner::footPhase(double phase, double phase1Single, double phase2Single) 
    {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phase1Single, 0.0) / (phase2Single - phase1Single), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, phaseSingle, zf};
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getTime
/*=======================================================================================================*/
    double FootMotionPlanner::getTime() 
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count() * 1E-6;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getDestinationTime
/*=======================================================================================================*/
    double FootMotionPlanner::getDestinationTime()
    {
        return (destinationTime);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setDestinationTime
/*=======================================================================================================*/
    void FootMotionPlanner::setDestinationTime(double inDestinationTime)
    {
        destinationTime = inDestinationTime;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootSource
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getLeftFootSource()
    {
        return (leftFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootSource
/*=======================================================================================================*/
    void FootMotionPlanner::setLeftFootSource(const Transform2D& inLeftFootSource)
    {
        leftFootSource = inLeftFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootSource
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getRightFootSource()
    {
        return (rightFootSource);
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootSource
/*=======================================================================================================*/
    void FootMotionPlanner::setRightFootSource(const Transform2D& inRightFootSource)
    {
        rightFootSource = inRightFootSource;
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getLeftFootDestination
/*=======================================================================================================*/
    Transform2D FootMotionPlanner::getLeftFootDestination()
    {
        return (leftFootDestination.front());
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setLeftFootDestination
/*=======================================================================================================*/
    void FootMotionPlanner::setLeftFootDestination(const Transform2D& inLeftFootDestination)
    {
        leftFootDestination.push(inLeftFootDestination);
    }
<<<<<<< 10dc0520add22a7f4f7ad7e432ea63bdcaef581f
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
<<<<<<< cdf96fbdd5a07a2529fcd01d4c07dd1dfe64510f
=======
>>>>>>> Message Headers, emit structs, and further encapsulation of
    /*=======================================================================================================*/
    //      NAME: getRightFootDestination
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
=======
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: getRightFootDestination
/*=======================================================================================================*/
>>>>>>> Documentation Changes to Foot Motion Planner
    Transform2D FootMotionPlanner::getRightFootDestination()
    {
        return (rightFootDestination.front());
    }
/*=======================================================================================================*/
/*      ENCAPSULATION METHOD: setRightFootDestination
/*=======================================================================================================*/
    void FootMotionPlanner::setRightFootDestination(const Transform2D& inRightFootDestination)
    {
        rightFootDestination.push(inRightFootDestination);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: isNewStepReceived
/*=======================================================================================================*/
    bool FootMotionPlanner::getNewStepReceived()
    {
        return (updateStepInstruction);
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: setNewStepReceived
/*=======================================================================================================*/
    void FootMotionPlanner::setNewStepReceived(bool inUpdateStepInstruction)
    {
        updateStepInstruction = inUpdateStepInstruction;
    }
/*=======================================================================================================*/
//      ENCAPSULATION METHOD: getMotionPhase
/*=======================================================================================================*/
    double FootMotionPlanner::getMotionPhase()
    {
        // Obtain current system time...
        double currentTime = getTime();
        // The percentage completed of the current step, range: [0,1]...
        double motionPhase = 1 - ((getDestinationTime() - currentTime) / stepTime);
        // Bind phase value to range [0,1], emit status if step completed...
        if (motionPhase > 1)
        {
            motionPhase = std::fmod(motionPhase, 1);
            if(getNewStepReceived())
            {
                emit(std::make_unique<FootStepCompleted>(true));
            }
        }
        return (motionPhase);
    }
/*=======================================================================================================*/
//      METHOD: configure
/*=======================================================================================================*/
    void FootMotionPlanner::configure(const YAML::Node& config)
    {
        emitLocalisation = config["emit_localisation"].as<bool>();

<<<<<<< 98fb8c92f4155568ba3378095590c40273990270
        return {xf, phaseSingle, zf};
    }
<<<<<<< f10da9c4b0232f8ba743f03d26325ddc863f1f48
=======

>>>>>>> Added messages between modules
=======
>>>>>>> Message Headers, emit structs, and further encapsulation of
    /*=======================================================================================================*/
    //      NAME: updateFootPosition
    /*=======================================================================================================*/
    /*
     *      @input  : <TODO: INSERT DESCRIPTION>
     *      @output : <TODO: INSERT DESCRIPTION>
     *      @pre-condition  : <TODO: INSERT DESCRIPTION>
     *      @post-condition : <TODO: INSERT DESCRIPTION>
    */
    void FootMotionPlanner::updateFootPosition(double phase, auto leftFootDestination, auto rightFootDestination) 
    {
        //Instantiate unitless phases for x(=0), y(=1) and z(=2) foot motion...
        arma::vec3 footPhases = footPhase(phase, phase1Single, phase2Single);
=======
        auto& stance = config["stance"];
        bodyHeight = stance["body_height"].as<Expression>();
        bodyTilt = stance["body_tilt"].as<Expression>();
        qLArmStart = stance["arms"]["left"]["start"].as<arma::vec>();
        qLArmEnd = stance["arms"]["left"]["end"].as<arma::vec>();
        qRArmStart = stance["arms"]["right"]["start"].as<arma::vec>();
        qRArmEnd = stance["arms"]["right"]["end"].as<arma::vec>();
        footOffset = stance["foot_offset"].as<arma::vec>();
        // gToe/heel overlap checking values
        stanceLimitY2 = DarwinModel::Leg::LENGTH_BETWEEN_LEGS - stance["limit_margin_y"].as<Expression>();
>>>>>>> Debugging for compilation of FootMotionPlanner

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
